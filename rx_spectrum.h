#pragma once
#include <stdint.h>

// Full-band complex FFT spectrum analyzer for RP2040_beacon.
// Call start() after rx_dsp_start()/rx_capture_start().
// Feed each dequeued DSP output block from Core 0, then call poll() frequently.

void rx_spectrum_start(float sample_rate_hz);
void rx_spectrum_stop(void);
void rx_spectrum_process(const int16_t* xi, const int16_t* xq, uint8_t n);
void rx_spectrum_poll(void);
