#!/usr/bin/env python3
"""UART-декодер для дампа логического анализатора (sigrok CSV, переходы).
Гипотеза: 115200 8N1, idle=high. Декодируем оба канала, ищем DUML-кадры (0x55)."""
import numpy as np

CSV = "зарядка_батарей_dji_hub.csv"
BAUD = 115200
BIT = 1.0 / BAUD  # длительность бита, с

# --- загрузка переходов ---
times, ch0, ch1 = [], [], []
with open(CSV) as f:
    for line in f:
        if not line or line[0] == ";" or line.startswith("SystemTime"):
            continue
        p = line.rstrip("\r\n").split(",")
        if len(p) != 4:
            continue
        try:
            times.append(float(p[1])); ch0.append(int(p[2])); ch1.append(int(p[3]))
        except ValueError:
            continue
t = np.array(times)
chans = {"CH0": np.array(ch0, dtype=np.int8), "CH1": np.array(ch1, dtype=np.int8)}

# доля времени в high (для проверки полярности idle)
total = t[-1] - t[0]
for name, c in chans.items():
    dt = np.diff(t)
    high_time = dt[c[:-1] == 1].sum()
    print(f"{name}: доля high = {high_time/total*100:.1f}%  (старт уровень={c[0]})")
print()

def level_at(c, query_t):
    """уровень канала в момент query_t (step-функция по переходам)"""
    idx = np.searchsorted(t, query_t, side="right") - 1
    idx = np.clip(idx, 0, len(c) - 1)
    return c[idx]

def decode_uart(c, idle=1):
    """Возвращает список (time, byte). Старт-бит = фронт idle->!idle."""
    out = []
    # позиции переходов 1->0 (старт-биты при idle=1)
    if idle == 1:
        starts = np.flatnonzero((c[:-1] == 1) & (c[1:] == 0)) + 1
    else:
        starts = np.flatnonzero((c[:-1] == 0) & (c[1:] == 1)) + 1
    last_end = -1.0
    for si in starts:
        st = t[si]
        if st < last_end:  # внутри уже декодированного байта
            continue
        # сэмплируем 8 бит данных в серединах, LSB first
        byte = 0
        for b in range(8):
            sample_t = st + (1.5 + b) * BIT
            bit = level_at(c, sample_t)
            if idle == 0:
                bit ^= 1
            byte |= (int(bit) & 1) << b
        # стоп-бит
        stop_t = st + 9.5 * BIT
        stop = level_at(c, stop_t)
        if idle == 0:
            stop ^= 1
        ok = (stop == 1)
        out.append((st, byte, ok))
        last_end = st + 10 * BIT  # 1 start + 8 data + 1 stop
    return out

for name, c in chans.items():
    dec = decode_uart(c, idle=1)
    nbytes = len(dec)
    nbad = sum(1 for _,_,ok in dec if not ok)
    sof = sum(1 for _,b,_ in dec if b == 0x55)
    print(f"{name}: декодировано {nbytes} байт, framing-ошибок={nbad} ({nbad/max(1,nbytes)*100:.1f}%), 0x55={sof}")
    # первые 64 байта
    hexs = " ".join(f"{b:02X}" for _,b,_ in dec[:64])
    print(f"   первые байты: {hexs}")
    print()
