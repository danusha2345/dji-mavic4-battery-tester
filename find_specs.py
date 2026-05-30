#!/usr/bin/env python3
"""Поиск известных уставок (ёмкость 6654 мАч, номинал 14320 мВ и др.) в payload'ах
батареи — known-plaintext атака для опознания полей. Плюс снятие диагностического
каталога гейджа (0x0e:0x21 имена / 0x0e:0x22 значения) и выравнивание имя↔значение.
"""
import sys, time
sys.path.insert(0, "/home/danik/Projects_and_coding/dji_battary")
import bat_serial as B

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

# известные эталоны (значение -> что это)
TARGETS = {
    6654:  "ёмкость(польз.) cfcc?",
    14320: "номинал 14.32В",
    7100:  "design cap",
    3464:  "остаток(rem)",
    17:    "циклы",
    3853:  "банка~",
    3849:  "банка~",
    15407: "пакет V",
    16800: "макс 4x4.2В",
}

def scan(label, p):
    """Ищем все TARGETS в payload p как u16LE/u16BE/u32LE, печатаем совпадения."""
    hits = []
    for o in range(len(p)):
        if o+1 < len(p):
            le = p[o] | (p[o+1] << 8)
            be = (p[o] << 8) | p[o+1]
            if le in TARGETS: hits.append(f"off{o} u16LE={le} ({TARGETS[le]})")
            if be in TARGETS and be != le: hits.append(f"off{o} u16BE={be} ({TARGETS[be]})")
        if o+3 < len(p):
            l32 = p[o] | (p[o+1]<<8) | (p[o+2]<<16) | (p[o+3]<<24)
            if l32 in TARGETS: hits.append(f"off{o} u32LE={l32} ({TARGETS[l32]})")
    if hits:
        print(f"  [{label}] " + "; ".join(hits))

ser = B.open_port(PORT)
seq = 0x400
print("=== 1) базовые read-команды: ищем эталоны ===")
basics = {
    "0d:01 static": (0x0d, 0x01, bytes.fromhex("01"+"00"*8)),
    "0d:02 status": (0x0d, 0x02, b"\x01"),
    "0d:03 cells":  (0x0d, 0x03, bytes.fromhex("01000000")),
    "0d:c0 limits": (0x0d, 0xc0, bytes.fromhex("01"+"00"*8)),
    "0d:da thermo": (0x0d, 0xda, b"\x01"),
    "0d:17":        (0x0d, 0x17, b"\x01"),
    "0d:20":        (0x0d, 0x20, b"\x01"),
    "0d:e4":        (0x0d, 0xe4, b"\x01"),
    "0d:d2 buf263": (0x0d, 0xd2, b"\x01"),
}
store = {}
for name, (cs, ci, pl) in basics.items():
    rsp, _ = B.send_recv(ser, cs, ci, pl, seq, 0.25); seq += 1
    if rsp:
        p = rsp[11:-2]; store[name] = p
        print(f"  {name} pl[{len(p)}]={p.hex()}")
        scan(name, p)

print("\n=== 2) диагностический каталог 0x0e (имена + значения) ===")
names_frame = vals_frame = None
for cid in (0xa2, 0xa3, 0xa4, 0xad, 0xa7):
    _, frames = B.send_recv(ser, 0x0d, cid, b"\x01", seq, 0.5); seq += 1
    for pkt in frames:
        f = B.parse_fields(pkt)
        if f["cmdset"] == 0x0e and f["cmdid"] == 0x21 and (names_frame is None or len(f["payload"]) > len(names_frame)):
            names_frame = f["payload"]
        if f["cmdset"] == 0x0e and f["cmdid"] == 0x22 and (vals_frame is None or len(f["payload"]) > len(vals_frame)):
            vals_frame = f["payload"]

names = []
if names_frame:
    # формат: магия(4) + ASCII-токены через \n
    txt = names_frame[4:] if names_frame[:2] == b"\xd1\xed" else names_frame
    names = [t.decode("ascii", "replace") for t in txt.split(b"\n") if t]
    print(f"  имён: {len(names)} -> {names}")
if vals_frame:
    print(f"  values frame pl[{len(vals_frame)}]={vals_frame.hex()}")
    scan("0e:22 values", vals_frame)
    body = vals_frame[4:] if vals_frame[:2] == b"\xd1\xed" else vals_frame
    u16 = [(body[i] | (body[i+1] << 8)) for i in range(0, len(body)-1, 2)]
    print(f"  values как u16LE ({len(u16)}): {u16}")
    if names:
        print("\n  === выравнивание имя↔значение (если 1:1 после заголовка) ===")
        for i, v in enumerate(u16):
            nm = names[i] if i < len(names) else "?"
            mark = "  <<<" if v in TARGETS else ""
            print(f"   [{i:2}] {nm:6} = {v}{mark}")

ser.close()
