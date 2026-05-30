#!/usr/bin/env python3
"""DJI DUML парсер поверх UART-декода обоих каналов.
Кадр: 55 | len_lo | (len_hi:2b)+(ver:6b) | crc8(hdr) | src | dst | seq(2) | attr | cmdset | cmdid | payload | crc16(2)
CRC-таблицы из dji-firmware-tools."""
import numpy as np

import sys
CSV = sys.argv[1] if len(sys.argv) > 1 else "зарядка_батарей_dji_hub.csv"
BAUD = 115200
BIT = 1.0 / BAUD

# ---- CRC таблицы DJI ----
TBL_CRC8 = [
0x00,0x5e,0xbc,0xe2,0x61,0x3f,0xdd,0x83,0xc2,0x9c,0x7e,0x20,0xa3,0xfd,0x1f,0x41,
0x9d,0xc3,0x21,0x7f,0xfc,0xa2,0x40,0x1e,0x5f,0x01,0xe3,0xbd,0x3e,0x60,0x82,0xdc,
0x23,0x7d,0x9f,0xc1,0x42,0x1c,0xfe,0xa0,0xe1,0xbf,0x5d,0x03,0x80,0xde,0x3c,0x62,
0xbe,0xe0,0x02,0x5c,0xdf,0x81,0x63,0x3d,0x7c,0x22,0xc0,0x9e,0x1d,0x43,0xa1,0xff,
0x46,0x18,0xfa,0xa4,0x27,0x79,0x9b,0xc5,0x84,0xda,0x38,0x66,0xe5,0xbb,0x59,0x07,
0xdb,0x85,0x67,0x39,0xba,0xe4,0x06,0x58,0x19,0x47,0xa5,0xfb,0x78,0x26,0xc4,0x9a,
0x65,0x3b,0xd9,0x87,0x04,0x5a,0xb8,0xe6,0xa7,0xf9,0x1b,0x45,0xc6,0x98,0x7a,0x24,
0xf8,0xa6,0x44,0x1a,0x99,0xc7,0x25,0x7b,0x3a,0x64,0x86,0xd8,0x5b,0x05,0xe7,0xb9,
0x8c,0xd2,0x30,0x6e,0xed,0xb3,0x51,0x0f,0x4e,0x10,0xf2,0xac,0x2f,0x71,0x93,0xcd,
0x11,0x4f,0xad,0xf3,0x70,0x2e,0xcc,0x92,0xd3,0x8d,0x6f,0x31,0xb2,0xec,0x0e,0x50,
0xaf,0xf1,0x13,0x4d,0xce,0x90,0x72,0x2c,0x6d,0x33,0xd1,0x8f,0x0c,0x52,0xb0,0xee,
0x32,0x6c,0x8e,0xd0,0x53,0x0d,0xef,0xb1,0xf0,0xae,0x4c,0x12,0x91,0xcf,0x2d,0x73,
0xca,0x94,0x76,0x28,0xab,0xf5,0x17,0x49,0x08,0x56,0xb4,0xea,0x69,0x37,0xd5,0x8b,
0x57,0x09,0xeb,0xb5,0x36,0x68,0x8a,0xd4,0x95,0xcb,0x29,0x77,0xf4,0xaa,0x48,0x16,
0xe9,0xb7,0x55,0x0b,0x88,0xd6,0x34,0x6a,0x2b,0x75,0x97,0xc9,0x4a,0x14,0xf6,0xa8,
0x74,0x2a,0xc8,0x96,0x15,0x4b,0xa9,0xf7,0xb6,0xe8,0x0a,0x54,0xd7,0x89,0x6b,0x35,
]
TBL_CRC16 = [
0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78,
]

def crc8(data, seed=0x77):
    for b in data:
        seed = TBL_CRC8[(seed ^ int(b)) & 0xFF]
    return seed & 0xFF

def crc16(data, seed=0x3692):
    for b in data:
        seed = ((seed >> 8) & 0xFF) ^ TBL_CRC16[(seed ^ int(b)) & 0xFF]
    return seed & 0xFFFF

# ---- загрузка + UART декод ----
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

def level_at(c, qt):
    idx = np.clip(np.searchsorted(t, qt, side="right") - 1, 0, len(c) - 1)
    return c[idx]

def decode_uart(c):
    starts = np.flatnonzero((c[:-1] == 1) & (c[1:] == 0)) + 1
    out_t, out_b = [], []
    last_end = -1.0
    for si in starts:
        st = t[si]
        if st < last_end:
            continue
        byte = 0
        for b in range(8):
            byte |= (int(level_at(c, st + (1.5 + b) * BIT)) & 1) << b
        out_t.append(st); out_b.append(byte)
        last_end = st + 9.5 * BIT
    return np.array(out_t), np.array(out_b, dtype=np.uint8)

def extract_frames(tt, bb):
    """Сканируем поток, ищем 0x55, валидируем по CRC."""
    frames = []
    i = 0
    n = len(bb)
    while i < n - 4:
        if bb[i] != 0x55:
            i += 1; continue
        length = ((int(bb[i+2]) & 0x03) << 8) | int(bb[i+1])
        if length < 4 or i + length > n:
            i += 1; continue
        pkt = bb[i:i+length]
        # header CRC-8 over первые 3 байта, хранится в 4-м
        if crc8(pkt[0:3]) != pkt[3]:
            i += 1; continue
        # CRC-16 над всем кадром кроме последних 2 байт
        c16 = crc16(pkt[0:length-2])
        stored16 = int(pkt[length-2]) | (int(pkt[length-1]) << 8)
        if c16 != stored16:
            i += 1; continue
        frames.append((tt[i], length, bytes(pkt)))
        i += length
    return frames

all_frames = []
for name, c in chans.items():
    tt, bb = decode_uart(c)
    fr = extract_frames(tt, bb)
    print(f"{name}: байт={len(bb)}, валидных DUML-кадров (CRC OK)={len(fr)}")
    for ts, ln, pkt in fr:
        all_frames.append((ts, name, pkt))

print()
all_frames.sort(key=lambda x: x[0])
print(f"ВСЕГО валидных кадров: {len(all_frames)}")
print()

# Разбор полей и сводка
from collections import Counter
pairs = Counter()       # (src,dst)
cmds = Counter()        # (cmdset, cmdid)
def fld(pkt):
    src = pkt[4]; dst = pkt[5]
    seq = pkt[6] | (pkt[7] << 8)
    attr = pkt[8]; cmdset = pkt[9]; cmdid = pkt[10]
    payload = pkt[11:-2]
    return src, dst, seq, attr, cmdset, cmdid, payload

for ts, name, pkt in all_frames:
    if len(pkt) < 13:
        continue
    src, dst, seq, attr, cmdset, cmdid, payload = fld(pkt)
    pairs[(src, dst)] += 1
    cmds[(cmdset, cmdid)] += 1

print("=== Пары Src->Dst (топ) ===")
for (s, d), n in pairs.most_common(12):
    print(f"  {s:#04x} -> {d:#04x} : {n}")
print()
print("=== Команды CmdSet:CmdId (топ) ===")
for (cs, ci), n in cmds.most_common(20):
    print(f"  set={cs:#04x} id={ci:#04x} : {n}")
print()
print("=== По одному примеру payload каждой команды (ответы 0x0b) ===")
seen = {}
for ts, name, pkt in all_frames:
    if len(pkt) < 13:
        continue
    src, dst, seq, attr, cmdset, cmdid, payload = fld(pkt)
    key = (cmdset, cmdid, attr & 0x80)
    if key in seen:
        continue
    seen[key] = True
    role = "RSP" if (attr & 0x80) else "REQ"
    asc = "".join(chr(b) if 32 <= b < 127 else "." for b in payload)
    print(f"  set={cmdset:#04x} id={cmdid:#04x} {role} {src:#04x}->{dst:#04x} pl[{len(payload):2}]={payload.hex()}  |{asc}|")

# ---- Реверс структуры 0x0d:0x02 (динамический статус) по дисперсии полей ----
print()
print("=== Поля 0x0d:0x02 RSP: разбор как uint16 LE, что меняется во времени ===")
samples = []  # (ts, payload)
for ts, name, pkt in all_frames:
    if len(pkt) < 13:
        continue
    src, dst, seq, attr, cmdset, cmdid, payload = fld(pkt)
    if cmdset == 0x0d and cmdid == 0x02 and (attr & 0x80) and len(payload) >= 44:
        samples.append((ts, payload))
print(f"сэмплов 0x0d:0x02 RSP: {len(samples)}")
if samples:
    L = min(len(p) for _, p in samples)
    L -= L % 2
    arr = np.array([[p[i] | (p[i+1] << 8) for i in range(0, L, 2)] for _, p in samples], dtype=np.int64)
    ts0 = samples[0][0]
    print(f"  off  first   last    min    max   (uint16 LE; * = меняется)")
    for j in range(arr.shape[1]):
        col = arr[:, j]
        changes = "*" if col.min() != col.max() else " "
        # signed-интерпретация для возможного тока
        signed = col.copy()
        signed[signed >= 32768] -= 65536
        print(f"  [{2*j:2}] {col[0]:6} {col[-1]:6} {col.min():6} {col.max():6} {changes}  signed[{signed.min()}..{signed.max()}]")
    # Временной ряд по ОФСЕТАМ skylab_hub/battery.rs (эталон):
    #   ток i32[6], остаток u32[14], темп u16[18], ячейки[20], SoC отображаемый[21], raw[3]
    print()
    print("  Временной ряд (каждый ~15-й сэмпл) — офсеты skylab: I, remain, T, SoC%, raw%")
    def s16(p, o): v = p[o] | (p[o+1] << 8); return v - 65536 if v >= 32768 else v
    def s32(p, o):
        v = p[o] | (p[o+1] << 8) | (p[o+2] << 16) | (p[o+3] << 24)
        return v - (1 << 32) if v >= (1 << 31) else v
    for k in range(0, len(samples), max(1, len(samples)//15)):
        ts, p = samples[k]
        cur = s32(p, 6)
        rem = p[14] | (p[15] << 8) | (p[16] << 16) | (p[17] << 24)
        temp = (p[18] | (p[19] << 8)) / 10.0
        cells = p[20]
        soc_disp = p[21]
        soc_raw = p[3]
        print(f"    t={ts-ts0:7.3f}  I={cur:6}mA  rem={rem:5}mAh  T={temp:4.1f}C  {cells}S  SoC={soc_disp:3}% (raw {soc_raw}%)")

# ---- Поиск счётчика циклов заряда: разбор 0d:01 / 0d:c0 / 0d:da по uint16 ----
print()
print("=== Разбор статических/статус-команд по uint16 LE (ищем cycle count, ~десятки) ===")
shown = set()
for ts, name, pkt in all_frames:
    if len(pkt) < 13:
        continue
    src, dst, seq, attr, cmdset, cmdid, payload = fld(pkt)
    if not (attr & 0x80):
        continue
    if (cmdset, cmdid) not in [(0x0d, 0x01), (0x0d, 0xc0), (0x0d, 0xda)]:
        continue
    if (cmdset, cmdid) in shown:
        continue
    shown.add((cmdset, cmdid))
    p = payload
    L = len(p) - (len(p) % 2)
    u16 = [(p[i] | (p[i+1] << 8)) for i in range(0, L, 2)]
    print(f"\n  {cmdset:#04x}:{cmdid:#04x} payload[{len(p)}]:")
    line = "    "
    for j, v in enumerate(u16):
        line += f"[{2*j}]={v} "
        if (j+1) % 6 == 0:
            print(line); line = "    "
    if line.strip():
        print(line)
    # подсветим байты-кандидаты в диапазоне 1..99 (возможные циклы)
    cand = [(i, p[i]) for i in range(len(p)) if 1 <= p[i] <= 99]
    print(f"    байты в диапазоне 1..99 (кандидаты на циклы): {cand}")

# ---- Версия прошивки/HW: 00:01 (GetVersion) + версионный блок в 0d:01 ----
# DJI-доки (dji_test_userlibs.dll, Air3 factory): 00:01 = GetVersion, ответ несёт
# factory code + версию. Тот же блок дублируется в 0d:01 (статика).
print()
print("=== Версия прошивки/HW (00:01 GetVersion и блок в 0d:01) ===")
def find_rsp(cs, ci):
    for ts, name, pkt in all_frames:
        if len(pkt) < 13:
            continue
        s, d, q, a, css, cid, pl = fld(pkt)
        if css == cs and cid == ci and (a & 0x80):
            return pl
    return None
v = find_rsp(0x00, 0x01)
if v:
    asc = "".join(chr(b) if 32 <= b < 127 else "." for b in v)
    print(f"  00:01 payload[{len(v)}]: {v.hex()}")
    print(f"           ascii: |{asc}|")
b1 = find_rsp(0x0d, 0x01)
def vblock(p):
    # ищем сигнатуру версионного дескриптора '04 00 01 00 24 00 00 2e' / общий хвост
    sig = bytes.fromhex("0400010024")
    i = p.find(sig)
    return (i, p[i:i+8]) if i >= 0 else (-1, b"")
for nm, pl in [("00:01", v), ("0d:01", b1)]:
    if pl:
        off, blk = vblock(pl)
        if off >= 0:
            print(f"  {nm}: версионный блок @off{off}: {blk.hex()}  (байты: {' '.join(str(x) for x in blk)})")

# ---- Реверс 0d:c0 и 0d:da по дисперсии во времени + сверка с 0d:02 ----
print()
print("=== Реверс 0d:c0 / 0d:da: дисперсия u16-полей во времени ===")
# опорные значения из 0d:02 (последний сэмпл): V_pack, I, rem, temp
ref = None
for ts, name, pkt in all_frames[::-1]:
    if len(pkt) < 13:
        continue
    s, d, q, a, cs, ci, pl = fld(pkt)
    if cs == 0x0d and ci == 0x02 and (a & 0x80) and len(pl) >= 22:
        ref = pl
        break
def collect(cs, ci):
    out = []
    for ts, name, pkt in all_frames:
        if len(pkt) < 13: continue
        s, d, q, a, c, i, pl = fld(pkt)
        if c == cs and i == ci and (a & 0x80):
            out.append((ts, pl))
    return out
for cs, ci in [(0x0d, 0xc0), (0x0d, 0xda)]:
    samp = collect(cs, ci)
    if not samp:
        continue
    L = min(len(p) for _, p in samp); L -= L % 2
    arr = np.array([[p[i] | (p[i+1] << 8) for i in range(0, L, 2)] for _, p in samp], dtype=np.int64)
    print(f"\n  {cs:#04x}:{ci:#04x}  ({len(samp)} сэмплов, payload {L}+ байт)")
    print(f"   off  first   last    min    max  изм?")
    for j in range(arr.shape[1]):
        col = arr[:, j]
        chg = "ИЗМ" if col.min() != col.max() else "   "
        sg = col.copy(); sg[sg >= 32768] -= 65536
        note = ""
        # сверка с известными величинами
        for label, val in [("Vpack~15600", 15400), ("I~3237", 3237), ("rem~3360", 3360),
                            ("temp225", 225), ("design7100", 7100)]:
            if abs(int(col[-1]) - val) <= 60:
                note = f"<-?{label}"
        print(f"   [{2*j:2}] {col[0]:6} {col[-1]:6} {col.min():6} {col.max():6}  {chg} {note}")

# ---- Временной ряд: сводим 0d:02(I), 0d:da([8],[10]) и 0d:c0([14]) ----
print()
print("=== Сводный ряд: ток(0d:02) vs поля 0d:da/0d:c0 — проверка гипотез ===")
def series(cs, ci, offs):
    res = []
    for ts, name, pkt in all_frames:
        if len(pkt) < 13: continue
        s, d, q, a, c, i, pl = fld(pkt)
        if c == cs and i == ci and (a & 0x80):
            vals = {o: (pl[o] | (pl[o+1] << 8)) if o+1 < len(pl) else None for o in offs}
            res.append((ts, vals))
    return res
import bisect
da = series(0x0d, 0xda, [2, 4, 8, 10, 12])
c0 = series(0x0d, 0xc0, [14])
i02 = series(0x0d, 0x02, [6])  # ток i16-нижняя часть
ts02 = [t for t, _ in i02]
ts_da = [t for t, _ in da]
ts0 = all_frames[0][0]
print("    t      I(0d02)  da[8]    da[10]V  da[2]  da[4]  da[12]  c0[14]")
for k in range(0, len(da), max(1, len(da)//15)):
    t, v = da[k]
    # ближайший ток из 0d:02
    j = min(range(len(ts02)), key=lambda x: abs(ts02[x] - t))
    cur = i02[j][1][6]
    # ближайший c0[14]
    jc = min(range(len(c0)), key=lambda x: abs(c0[x][0] - t)) if c0 else None
    c14 = c0[jc][1][14] if c0 else None
    print(f"   {t-ts0:6.2f}  {cur:6}   {v[8]:6}   {v[10]:6}   {v[2]:5}  {v[4]:5}  {v[12]:5}   {c14}")
