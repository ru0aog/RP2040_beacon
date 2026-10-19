# Утилита составления спектрограммы по поступающим сигналам  
# По умолчанию используется COM22 и запись 15 секунд  
# python iq_spectrum.py COM22 60  
  
import sys  
import time  
import serial  
import numpy as np  
import matplotlib.pyplot as plt  
  
# --------------------------------------------------  
# Настройки  
# --------------------------------------------------  
port = sys.argv[1] if len(sys.argv) > 1 else "COM15"  
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0  
  
FS = 15625  
TARGET = round(FS * secs)  
BLK = 64                    # отсчётов I/Q в одном кадре (фиксировано)  
  
BAUD = 2_000_000  
READ_TIMEOUT = 5.0  
SYNC_TIMEOUT = 15.0  
  
# Основной спектр: высокое частотное разрешение  
SEG = 32768                 # df = Fs/SEG ≈ 0.477 Гц  
HOP = SEG // 2              # перекрытие 50%  
  
# Спектрограмма: низкое разрешение, хорошая динамика во времени  
SPEC_L = 1024               # df ≈ 15.26 Гц  
SPEC_H = 256                # шаг по времени ~16.4 мс  
  
# Диапазон отображения: полная полоса для первого прогона  
F_MIN = -2500.0  
F_MAX = +2500.0  
# после подтверждения тонов сузить: F_MIN=-1000, F_MAX=-500  
  
# --------------------------------------------------  
# Чтение и синхронизация  
# --------------------------------------------------  
def read_exact(s, size):  
    data = bytearray()  
    deadline = time.monotonic() + READ_TIMEOUT  
    while len(data) < size:  
        chunk = s.read(size - len(data))  
        if chunk:  
            data.extend(chunk)  
        elif time.monotonic() >= deadline:  
            raise TimeoutError(f"timeout: {len(data)}/{size} bytes")  
    return bytes(data)  
  
  
def sync(s, bad):  
    """Поиск 'I','Q' посимвольно. Возвращает обновлённый счётчик ложных маркеров."""  
    deadline = time.monotonic() + SYNC_TIMEOUT  
    state = 0  
    while True:  
        b = s.read(1)  
        if not b:  
            if time.monotonic() >= deadline:  
                raise TimeoutError("sync: no data")  
            continue  
        if state == 0:  
            state = 1 if b == b'I' else 0  
        else:  
            if b == b'Q':  
                return bad  
            # промах: байт не 'Q'. Если это 'I' — остаёмся в state=1  
            state = 1 if b == b'I' else 0  
            bad += 1   # ложная 'I' — считаем для статистики  
  
  
s = serial.Serial(port, BAUD, timeout=1)  
s.reset_input_buffer()  
  
iq = np.empty(TARGET, dtype=np.complex64)  
have = 0  
bad = 0  
need_sync = True  
  
print("Захват... Ctrl+C для отмены")  
try:  
    while have < TARGET:  
        if need_sync:  
            bad = sync(s, bad)  
            need_sync = False  
  
        n = read_exact(s, 1)[0]        # байт длины из заголовка 'I','Q',n  
        if n != BLK:  
            bad += 1  
            need_sync = True  
            continue  
  
        raw = read_exact(s, n * 4)     # n пар int16 (I,Q) little-endian  
  
        # контроль целостности: сразу за payload должен идти 'IQ'  
        if read_exact(s, 2) != b'IQ':  
            bad += 1  
            need_sync = True  
            continue  
        need_sync = False              # маркер уже съеден  
  
        v = np.frombuffer(raw, dtype='<i2')  
        i = v[0::2].astype(np.float32) / 32768.0  
        q = v[1::2].astype(np.float32) / 32768.0  
        blk = i + 1j * q  
  
        take = min(n, TARGET - have)  
        iq[have:have + take] = blk[:take]  
        have += take  
  
        if have % (FS * 2) < n:        # прогресс раз в ~2 с  
            print(f"  {have}/{TARGET}  bad={bad}")  
except KeyboardInterrupt:  
    pass  
  
iq = iq[:have]  
loss = 100.0 * (1.0 - have / TARGET) if TARGET else 0.0  
print(f"Принято {have} отсчётов ({have/FS:.1f} с), bad={bad}, loss={loss:.1f}%")  
np.save("iq.npy", iq)  
  
if have < SEG:  
    sys.exit("Отсчётов меньше одного SEG — спектр не строится")  
  
# --------------------------------------------------  
# Основной спектр (Welch на чистом numpy)  
# --------------------------------------------------  
w = np.hanning(SEG).astype(np.float64)  
window_power = np.sum(w * w)  
  
nseg = 1 + (len(iq) - SEG) // HOP  
Pxx = np.zeros(SEG, dtype=np.float64)  
for k in range(nseg):  
    seg = iq[k * HOP : k * HOP + SEG] * w  
    X = np.fft.fftshift(np.fft.fft(seg))  
    Pxx += (np.abs(X) ** 2)  
Pxx /= (nseg * FS * window_power)      # PSD, dBFS/Hz  
Pxx_db = 10.0 * np.log10(Pxx + 1e-20)  
freq = np.fft.fftshift(np.fft.fftfreq(SEG, 1.0 / FS))  
  
# Пики из УСРЕДНЁННОГО PSD (а не из FFT всей записи)  
band = (freq >= F_MIN) & (freq <= F_MAX)  
f_band = freq[band]  
p_band = Pxx_db[band]  
order = np.argsort(p_band)[::-1]  
picked = []  
for idx in order:  
    f0 = f_band[idx]  
    if all(abs(f0 - pf) > 5.0 for pf, _ in picked):   # не ближе 5 Гц  
        picked.append((f0, p_band[idx]))  
    if len(picked) == 8:  
        break  
print("top peaks (усреднённый PSD):")  
for f0, p0 in picked:  
    print(f"  {f0:+8.1f} Hz  {p0:6.1f} dBFS/Hz")  
  
# --------------------------------------------------  
# Спектрограмма  
# --------------------------------------------------  
ws = np.hanning(SPEC_L).astype(np.float32)  
nspec = (len(iq) - SPEC_L) // SPEC_H + 1  
S = None  
if nspec > 0:  
    S = np.empty((nspec, SPEC_L), dtype=np.float32)  
    for k in range(nspec):  
        seg = iq[k * SPEC_H : k * SPEC_H + SPEC_L] * ws  
        S[k] = np.log10(np.abs(np.fft.fftshift(np.fft.fft(seg))) ** 2 + 1e-12)  
freqs = np.fft.fftshift(np.fft.fftfreq(SPEC_L, 1.0 / FS))  
  
# --------------------------------------------------  
# Графики  
# --------------------------------------------------  
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True)  
  
ax1.plot(f_band, p_band, linewidth=0.8)  
ax1.grid(True)  
ax1.set_title(  
    f"IQ spectrum | Fs={FS} Hz | df={FS/SEG:.3f} Hz | "  
    f"avg={nseg} | dur={len(iq)/FS:.1f} s | bad={bad}"  
)  
ax1.set_xlabel("Частота, Гц")  
ax1.set_ylabel("PSD, dBFS/Hz")  
  
if S is not None:  
    ax2.imshow(S.T, aspect="auto", origin="lower",  
               interpolation="nearest",  
               extent=[0, len(iq)/FS, freqs[0], freqs[-1]])  
    ax2.set_ylim(F_MIN, F_MAX)  
    ax2.set_title(f"IQ spectrogram | FFT={SPEC_L} | df={FS/SPEC_L:.2f} Hz")  
    ax2.set_xlabel("Время, с")  
    ax2.set_ylabel("Частота, Гц")  
else:  
    ax2.set_visible(False)  
  
plt.savefig("iq_spectrum.png", dpi=150)  
print("Сохранено: iq_spectrum.png")  
plt.show()
