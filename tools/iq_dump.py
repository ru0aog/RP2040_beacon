# 
# Утилита составления спектрограммы по поступающим сигналам
# 
# По умолчанию используется COM22 и запись 60 секунд
# Можно указать порт и длительность:
# python iq_spectrum.py COM22 120
# 

import sys
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

# --------------------------------------------------
# Настройки
# --------------------------------------------------
port = sys.argv[1] if len(sys.argv) > 1 else "COM22"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

FS = 15625
TARGET = round(FS * secs)

BAUD = 2_000_000
READ_TIMEOUT = 5.0
SYNC_TIMEOUT = 15.0

# Основной спектр: высокое частотное разрешение
SEG = 32768
HOP = SEG // 2

# Диапазон отображения спектра и спектрограммы
F_MIN = -1000
F_MAX = -500

# Спектрограмма: короткое окно для наблюдения переключений
SPEC_L = 1024
SPEC_HOP = 256


# --------------------------------------------------
# Чтение заданного количества байтов из UART
# --------------------------------------------------
def read_exact(s, size):
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


# --------------------------------------------------
# Поиск маркера IQ
# --------------------------------------------------
def sync(s):
    state = 0
    deadline = time.monotonic() + SYNC_TIMEOUT

    while time.monotonic() < deadline:
        b = s.read(1)

        if not b:
            continue

        c = b[0]

        if state == 0:
            state = 1 if c == ord("I") else 0
        elif c == ord("Q"):
            return
        else:
            # Если текущий байт I, он может начать новый маркер
            state = 1 if c == ord("I") else 0

    raise TimeoutError("Не найден маркер IQ")


# --------------------------------------------------
# Сбор IQ
#
# Предполагаемый формат кадра:
# 'I', 'Q', uint8 length,
# затем length пар int16 I, int16 Q
# --------------------------------------------------
chunks = []
got = 0
last_report = 0

print(f"Порт: {port}, скорость: {BAUD}")
print(f"План записи: {secs:.1f} с, {TARGET} IQ-отсчётов")

try:
    with serial.Serial(port, BAUD, timeout=1) as s:
        s.reset_input_buffer()

        while got < TARGET:
            sync(s)

            n = read_exact(s, 1)[0]

            if n == 0:
                continue

            raw = read_exact(s, n * 4)
            a = np.frombuffer(raw, dtype="<i2")

            if len(a) != 2 * n:
                continue

            # Нормирование int16
            a = a.astype(np.float32) / 32768.0

            # Комплексные отсчёты I + jQ
            blk = a[0::2] + 1j * a[1::2]

            take = min(n, TARGET - got)
            chunks.append(blk[:take].copy())
            got += take

            if got - last_report >= FS * 2:
                print(
                    f"\rПолучено: {got}/{TARGET} "
                    f"({got / FS:.1f} с)",
                    end="",
                    flush=True,
                )
                last_report = got

finally:
    pass

iq = (
    np.concatenate(chunks)
    if chunks
    else np.empty(0, dtype=np.complex64)
)

print(f"\nПолучено {len(iq)} отсчётов "
      f"({len(iq) / FS:.2f} с)")

if len(iq) < SEG:
    sys.exit(
        f"Недостаточно данных для спектра: нужно минимум "
        f"{SEG} отсчётов ({SEG / FS:.2f} с)."
    )


# --------------------------------------------------
# Welch PSD
# Окно Hann, перекрытие 50%
# --------------------------------------------------
w = np.hanning(SEG).astype(np.float64)
window_power = np.sum(w * w)

nseg = 1 + (len(iq) - SEG) // HOP
acc = np.zeros(SEG, dtype=np.float64)

for k in range(nseg):
    start = k * HOP
    x = iq[start:start + SEG] * w
    X = np.fft.fft(x)
    acc += np.abs(X) ** 2

# PSD в единицах dBFS/Hz
Pxx = acc / (nseg * FS * window_power)

freq = np.fft.fftfreq(SEG, d=1 / FS)
freq = np.fft.fftshift(freq)
Pxx = np.fft.fftshift(Pxx)

Pxx_db = 10 * np.log10(np.maximum(Pxx, 1e-20))

band = (freq >= F_MIN) & (freq <= F_MAX)


# --------------------------------------------------
# Спектрограмма
# --------------------------------------------------
S = None

if len(iq) >= SPEC_L:
    w2 = np.hanning(SPEC_L).astype(np.float64)
    window_power2 = np.sum(w2 * w2)

    ns = 1 + (len(iq) - SPEC_L) // SPEC_HOP
    S = np.empty((ns, SPEC_L), dtype=np.float32)

    for k in range(ns):
        start = k * SPEC_HOP
        x = iq[start:start + SPEC_L] * w2

        X = np.fft.fftshift(np.fft.fft(x))
        psd = np.abs(X) ** 2 / (FS * window_power2)

        S[k] = 10 * np.log10(np.maximum(psd, 1e-20))

    freqs = np.fft.fftshift(
        np.fft.fftfreq(SPEC_L, d=1 / FS)
    )

    times = np.arange(ns) * SPEC_HOP / FS


# --------------------------------------------------
# Построение графиков
# --------------------------------------------------
fig, (ax1, ax2) = plt.subplots(
    2, 1,
    figsize=(13, 9),
    constrained_layout=True
)

# Основной спектр
ax1.plot(freq[band], Pxx_db[band], linewidth=0.8)
ax1.set_xlim(F_MIN, F_MAX)
ax1.set_xticks(np.arange(F_MIN, F_MAX + 1, 250))
ax1.grid(True, which="both")

ax1.set_title(
    f"IQ spectrum | Fs={FS} Hz | FFT={SEG} | "
    f"df={FS / SEG:.3f} Hz | "
    f"averages={nseg} | duration={len(iq) / FS:.1f} s"
)
ax1.set_xlabel("Частота, Гц")
ax1.set_ylabel("PSD, dBFS/Hz")

# Спектрограмма
if S is not None:
    ax2.imshow(
        S.T,
        aspect="auto",
        origin="lower",
        interpolation="nearest",
        extent=[
            0,
            len(iq) / FS,
            freqs[0],
            freqs[-1],
        ],
    )

    ax2.set_ylim(F_MIN, F_MAX)
    ax2.set_title(
        f"IQ spectrogram | FFT={SPEC_L} | "
        f"df={FS / SPEC_L:.2f} Hz"
    )
    ax2.set_xlabel("Время, с")
    ax2.set_ylabel("Частота, Гц")
else:
    ax2.set_visible(False)

plt.savefig("iq_spectrum.png", dpi=150)
print("Сохранено: iq_spectrum.png")

plt.show()
