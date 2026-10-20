#include "rx_spectrum.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// 4096-point complex FFT, 50% overlap, Hann window.
// The FFT uses a 1/N-scaled Q15 radix-2 implementation to avoid floating-point
// work in the hot path on RP2040 (Cortex-M0+ has no hardware FPU).
namespace {
constexpr uint16_t FFT_N = 4096;
constexpr uint16_t FFT_MASK = FFT_N - 1;
constexpr uint16_t FFT_HALF = FFT_N / 2;
constexpr uint16_t HOP_N = FFT_N / 2;
constexpr uint8_t PEAKS_OUT = 24;
constexpr uint8_t PEAK_SEPARATION_BINS = 3;
constexpr size_t TX_CAPACITY = 1400;
constexpr size_t TX_CHUNK_MAX = 32;

static int16_t ring_i[FFT_N];
static int16_t ring_q[FFT_N];
static int16_t fft_i[FFT_N];
static int16_t fft_q[FFT_N];
static int16_t window_q15[FFT_N];
static int16_t tw_re[FFT_HALF];
static int16_t tw_im[FFT_HALF];

static uint16_t ring_write = 0;
static uint16_t samples_seen = 0;
static uint16_t samples_since_frame = 0;
static volatile bool frame_due = false;
static uint32_t frame_seq = 0;
static uint32_t skipped_frames = 0;
static float fs_hz = 15625.0f;

static char txbuf[TX_CAPACITY];
static size_t tx_len = 0;
static size_t tx_pos = 0;
static bool running = false;

struct Peak {
  uint16_t bin;
  uint64_t power;
};
static Peak peaks[PEAKS_OUT];

static inline int16_t sat16(int32_t x) {
  if (x > 32767) return 32767;
  if (x < -32768) return -32768;
  return (int16_t)x;
}

static inline uint64_t bin_power(uint16_t k) {
  const int32_t re = fft_i[k];
  const int32_t im = fft_q[k];
  return (uint64_t)((int64_t)re * re + (int64_t)im * im);
}

static void init_tables() {
  for (uint16_t n = 0; n < FFT_N; ++n) {
    const float w = 0.5f - 0.5f * cosf(6.2831853071795864769f *
                                      (float)n / (float)(FFT_N - 1));
    window_q15[n] = sat16((int32_t)(w * 32767.0f + 0.5f));
  }
  for (uint16_t k = 0; k < FFT_HALF; ++k) {
    const float a = -6.2831853071795864769f * (float)k / (float)FFT_N;
    tw_re[k] = sat16((int32_t)(cosf(a) * 32767.0f));
    tw_im[k] = sat16((int32_t)(sinf(a) * 32767.0f));
  }
}

// In-place radix-2 decimation-in-time complex FFT.
// Each stage is divided by two, so intermediate values remain in Q15 range.
static void fft_q15() {
  uint16_t j = 0;
  for (uint16_t i = 1; i < FFT_N; ++i) {
    uint16_t bit = FFT_HALF;
    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j ^= bit;
    if (i < j) {
      int16_t t;
      t = fft_i[i]; fft_i[i] = fft_i[j]; fft_i[j] = t;
      t = fft_q[i]; fft_q[i] = fft_q[j]; fft_q[j] = t;
    }
  }

  for (uint16_t len = 2; len <= FFT_N; len <<= 1) {
    const uint16_t half = len >> 1;
    const uint16_t tw_step = FFT_N / len;
    for (uint16_t base = 0; base < FFT_N; base += len) {
      for (uint16_t j2 = 0; j2 < half; ++j2) {
        const uint16_t tw = j2 * tw_step;
        const int32_t wr = tw_re[tw];
        const int32_t wi = tw_im[tw];
        const uint16_t a = base + j2;
        const uint16_t b = a + half;

        const int32_t br = fft_i[b];
        const int32_t bi = fft_q[b];
        const int32_t tr = (br * wr - bi * wi) >> 15;
        const int32_t ti = (br * wi + bi * wr) >> 15;
        const int32_t ar = fft_i[a];
        const int32_t ai = fft_q[a];

        fft_i[a] = sat16((ar + tr) >> 1);
        fft_q[a] = sat16((ai + ti) >> 1);
        fft_i[b] = sat16((ar - tr) >> 1);
        fft_q[b] = sat16((ai - ti) >> 1);
      }
    }
  }
}

static void take_frame_snapshot_and_fft() {
  // ring_write points to the next slot to overwrite; that is the oldest sample
  // when the ring is full.
  uint16_t src = ring_write;
  for (uint16_t n = 0; n < FFT_N; ++n) {
    const int16_t wi = window_q15[n];
    const int16_t xi = ring_i[src];
    const int16_t xq = ring_q[src];
    fft_i[n] = (int16_t)(((int32_t)xi * wi) >> 15);
    fft_q[n] = (int16_t)(((int32_t)xq * wi) >> 15);
    src = (src + 1) & FFT_MASK;
  }
  fft_q15();
}

static void find_peaks(uint64_t* max_power_out) {
  for (uint8_t p = 0; p < PEAKS_OUT; ++p) {
    peaks[p].bin = 0;
    peaks[p].power = 0;
  }

  uint64_t maxp = 1;
  for (uint16_t k = 0; k < FFT_N; ++k) {
    const uint64_t p = bin_power(k);
    if (p > maxp) maxp = p;
  }

  // Greedy top-local-max selection, suppressing nearby bins to avoid reporting
  // the same Hann-window main lobe as several separate peaks.
  for (uint8_t p = 0; p < PEAKS_OUT; ++p) {
    uint16_t best_k = 0;
    uint64_t best_p = 0;
    for (uint16_t k = 0; k < FFT_N; ++k) {
      const uint16_t left = (k == 0) ? (FFT_N - 1) : (k - 1);
      const uint16_t right = (k + 1) & FFT_MASK;
      const uint64_t pk = bin_power(k);
      if (pk < bin_power(left) || pk <= bin_power(right)) continue;

      bool too_close = false;
      for (uint8_t q = 0; q < p; ++q) {
        int32_t d = (int32_t)k - (int32_t)peaks[q].bin;
        if (d < 0) d = -d;
        if (d > (int32_t)FFT_HALF) d = FFT_N - d;
        if (d < PEAK_SEPARATION_BINS) {
          too_close = true;
          break;
        }
      }
      if (!too_close && pk > best_p) {
        best_p = pk;
        best_k = k;
      }
    }
    if (best_p == 0) break;
    peaks[p].bin = best_k;
    peaks[p].power = best_p;
  }
  *max_power_out = maxp;
}

static int32_t bin_to_signed(uint16_t k) {
  return (k >= FFT_HALF) ? ((int32_t)k - FFT_N) : (int32_t)k;
}

static void build_packet() {
  uint64_t maxp = 1;
  find_peaks(&maxp);

  tx_len = 0;
  tx_pos = 0;
  int n = snprintf(txbuf, sizeof(txbuf),
                   "SPEC,seq=%lu,fs=%.3f,n=%u,df=%.6f,span=%.3f\n",
                   (unsigned long)frame_seq, (double)fs_hz,
                   (unsigned)FFT_N, (double)(fs_hz / FFT_N),
                   (double)fs_hz);
  if (n < 0 || (size_t)n >= sizeof(txbuf)) {
    tx_len = tx_pos = 0;
    return;
  }
  tx_len = (size_t)n;

  for (uint8_t p = 0; p < PEAKS_OUT; ++p) {
    if (peaks[p].power == 0) break;
    const float rel_db = 10.0f * log10f((float)peaks[p].power / (float)maxp);
    const float freq = (float)bin_to_signed(peaks[p].bin) * fs_hz / FFT_N;
    n = snprintf(txbuf + tx_len, sizeof(txbuf) - tx_len,
                 "P,%.2f,%.2f\n", (double)freq, (double)rel_db);
    if (n < 0 || (size_t)n >= sizeof(txbuf) - tx_len) break;
    tx_len += (size_t)n;
  }

  n = snprintf(txbuf + tx_len, sizeof(txbuf) - tx_len,
               "END,skipped=%lu\n", (unsigned long)skipped_frames);
  if (n > 0 && (size_t)n < sizeof(txbuf) - tx_len) tx_len += (size_t)n;
}

} // namespace

void rx_spectrum_start(float sample_rate_hz) {
  running = false;
  fs_hz = (sample_rate_hz > 0.0f) ? sample_rate_hz : 15625.0f;
  memset(ring_i, 0, sizeof(ring_i));
  memset(ring_q, 0, sizeof(ring_q));
  memset(fft_i, 0, sizeof(fft_i));
  memset(fft_q, 0, sizeof(fft_q));
  ring_write = 0;
  samples_seen = 0;
  samples_since_frame = 0;
  frame_due = false;
  frame_seq = 0;
  skipped_frames = 0;
  tx_len = tx_pos = 0;
  init_tables();
  running = true;
}

void rx_spectrum_stop(void) {
  running = false;
  frame_due = false;
  tx_len = tx_pos = 0;
}

void rx_spectrum_process(const int16_t* xi, const int16_t* xq, uint8_t n) {
  if (!running || !xi || !xq) return;
  for (uint8_t k = 0; k < n; ++k) {
    ring_i[ring_write] = xi[k];
    ring_q[ring_write] = xq[k];
    ring_write = (ring_write + 1) & FFT_MASK;
    if (samples_seen < FFT_N) {
      ++samples_seen;
      if (samples_seen == FFT_N) {
        // Первый кадр готов сразу после первых 4096 отсчётов.
        frame_due = true;
        samples_since_frame = 0;
      }
    } else {
      ++samples_since_frame;
      if (samples_since_frame >= HOP_N) {
        samples_since_frame = 0;
        if (frame_due) {
          ++skipped_frames;
        } else {
          frame_due = true;
        }
      }
    }
  }
}

void rx_spectrum_poll(void) {
  if (!running) return;

  // Nonblocking serial drain: never wait for the host to read the USB CDC port.
  if (tx_pos < tx_len) {
    int avail = Serial.availableForWrite();
    if (avail > 0) {
      size_t count = (size_t)avail;
      if (count > TX_CHUNK_MAX) count = TX_CHUNK_MAX;
      if (count > tx_len - tx_pos) count = tx_len - tx_pos;
      const size_t sent = Serial.write(
          reinterpret_cast<const uint8_t*>(txbuf + tx_pos), count);
      tx_pos += sent;
      if (tx_pos >= tx_len) tx_len = tx_pos = 0;
    }
    return;
  }

  if (frame_due && samples_seen >= FFT_N) {
    frame_due = false;
    take_frame_snapshot_and_fft();
    ++frame_seq;
    build_packet();
  }
}
