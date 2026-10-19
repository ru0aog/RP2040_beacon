import serial, sys, numpy as np  
s = serial.Serial(sys.argv[1], 2000000, timeout=5)  
N = 64  
out = open(sys.argv[2], 'wb')  
while True:  
    while s.read(1) != b'I' or s.read(1) != b'Q':  # синхронизация по магику  
        pass  
    i = np.frombuffer(s.read(N*2), np.int16)  
    q = np.frombuffer(s.read(N*2), np.int16)  
    (i + 1j*q).astype(np.complex64).tofile(out)
