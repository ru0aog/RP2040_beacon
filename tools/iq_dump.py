# Утилита составления спектрограммы по поступающим сигналам
# Порт - СОМ22

import sys
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

port = sys.argv[1] if len(sys.argv) > 1 else "COM22"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

FS = 15625
TARGET = round(FS * secs)

# Предполагаемый формат кадра:
# 'I', 'Q', uint8 length, length * (int16 I, int16 Q)
BAUD = 2_000_000
READ_TIMEOUT = 5.0
SYNC_TIMEOUT = 15.0

def read_exact(s, size):
    """Прочитать ровно size байт или сообщить о тайм-ауте."""
    data = bytearray()
    deadline = time.monotonic() + READ_TIMEOUT

    while len(data) < size:
        chunk = s.read(size - len(data))
        if chunk:
            data.extend(chunk)
        elif time.monotonic() >= deadline:
            raise TimeoutError(
                f"UART timeout: получено {len(data)} из {size} байт"
            )

    return bytes(data)


def sync(s):
    """Поиск IQ с сохранением перекрывающихся совпадений."""
    state = 0
    deadline = time.monotonic() + SYNC_TIMEOUT

    while time.monotonic() < deadline:
        b = s.read(1)

        if not b:
            continue

        c = b[0]

        if state == 0:
            state = 1 if c == ord("I") else 0
        else:
            if c == ord("Q"):
                return
            # Текущий байт I может быть началом следующего маркера.
            state = 1 if c == ord("I") else 0

    raise TimeoutError("Не найден маркер IQ")


s = serial.Serial(port, BAUD, timeout=1)
s.reset_input_buffer()

chunks = []
got = 0

try:
    while got < TARGET:
        sync(s)

        n = read_exact(s, 1)[0]
        if n == 0:
            continue

        raw = read_exact(s, n * 4)

        # Порядок байтов: little-endian.
        a = np.frombuffer(raw, dtype="<i2")

        if len(a) != 2 * n:
            continue

        # Нормируем int16 в диапазон примерно [-1, 1).
        a = a.astype(np.float32) / 32768.0
        blk = a[0::2] + 1j * a[1::2]

        take = min(n, TARGET - got)
        chunks.append(blk[:take].copy())
        got += take

        if got % (FS * 2) < n:
            print(
                f"\rПолучено {got}/{TARGET} отсчётов "
                f"({got / FS:.2f} с)",
                end="",
                flush=True,
            )

finally:
    s.close()

iq = np.concatenate(chunks) if chunks else np.empty(0, np.complex64)

print(f"\nПолучено {len(iq)} отсчётов ({len(iq) / FS:.2f} с)")

SEG = 4096
if len(iq) < SEG:
    sys.exit(
        f"Недостаточно данных для спектра: нужно минимум {SEG} отсчётов. "
        "Проверь UART, формат кадров и режим передачи IQ."
    )

# --------------------------------------------------
# Welch PSD: окно Hann, перекрытие 50%
# IQ нормирован на 32768, поэтому мощность в dBFS/Hz.
# --------------------------------------------------
HOP = SEG // 2
w = np.hanning(SEG).astype(np.float32)
window_power = np.sum(w * w)

nseg = 1 + (len(iq) - SEG) // HOP
acc = np.zeros(SEG, dtype=np.float64)

for k in range(nseg):
    start = k * HOP
    x = iq[start:start + SEG] * w
    X = np.fft.fft(x)
    acc += np.abs(X) ** 2

# Плотность спектральной мощности.
Pxx = acc / (nseg * FS * window_power)

freq = np.fft.fftshift(np.fft.fftfreq(SEG, d=1 / FS))
Pxx = np.fft.fftshift(Pxx)
Pxx_db = 10 * np.log10(np.maximum(Pxx, 1e-20))

# --------------------------------------------------
# Спектрограмма
# --------------------------------------------------
L = 1024
H = 256
w2 = np.hanning(L).astype(np.float32)
window_power2 = np.sum(w2 * w2)

if len(iq) >= L:
    ns = 1 + (len(iq) - L) // H
    S = np.empty((ns, L), dtype=np.float32)

    for k in range(ns):
        start = k * H
        x = iq[start:start + L] * w2
        X = np.fft.fftshift(np.fft.fft(x))
        psd = np.abs(X) ** 2 / (FS * window_power2)
        S[k] = 10 * np.log10(np.maximum(psd, 1e-20))

    freqs = np.fft.fftshift(np.fft.fftfreq(L, d=1 / FS))
    spectrogram_duration = ns * H / FS
else:
    S = None

# --------------------------------------------------
# Графики
# --------------------------------------------------
fig, (ax1, ax2) = plt.subplots(
    2, 1, figsize=(12, 8), constrained_layout=True
)

ax1.plot(freq, Pxx_db, linewidth=0.8)
ax1.set_xlim(-2500, 2500)
ax1.grid(True)
ax1.set_title(f"IQ spectrum — Fs={FS} Hz, N={len(iq)}")
ax1.set_xlabel("Частота, Гц")
ax1.set_ylabel("PSD, dBFS/Hz")

if S is not None:
    ax2.imshow(
        S.T,
        aspect="auto",
        origin="lower",
        interpolation="nearest",
        extent=[
            0,
            spectrogram_duration,
            freqs[0],
            freqs[-1],
        ],
    )
    ax2.set_ylim(-2500, 2500)
    ax2.set_title("IQ spectrogram")
    ax2.set_xlabel("Время, с")
    ax2.set_ylabel("Частота, Гц")
else:
    ax2.set_visible(False)

plt.savefig("iq_spectrum.png", dpi=150)
print("Сохранено: iq_spectrum.png")
plt.show()

