#!/usr/bin/env python3
"""Live-обмен с батареей DJI (Mavic 4 / WA341) по USB-UART, протокол DUML.

Режимы:
  sniff [сек]              — пассивно слушать линию, декодировать DUML (безопасно)
  poll  [N]                — слать стандартный набор read-команд хаба и парсить ответы
                             (N циклов, по умолчанию бесконечно, Ctrl-C для выхода)
  read  <имя>              — одна команда: status|static|cells|serial|limits|thermo|version
  send  <set> <id> [hexpl] — произвольная команда (set/id в hex), payload в hex

Опции: --port /dev/ttyUSB0  --src 0x4b  --dst 0x0b
CRC-таблицы и офсеты полей — из duml.py / PROTOCOL.md этого проекта.
"""
import sys, time, argparse

# ---------------- CRC (DJI DUML), таблицы из duml.py ----------------
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
    for b in data: seed = TBL_CRC8[(seed ^ b) & 0xFF]
    return seed & 0xFF
def crc16(data, seed=0x3692):
    for b in data: seed = ((seed >> 8) & 0xFF) ^ TBL_CRC16[(seed ^ b) & 0xFF]
    return seed & 0xFFFF

# ---------------- сборка / разбор кадра DUML ----------------
def build_frame(src, dst, cmdset, cmdid, payload=b"", attr=0x40, seq=0, version=1):
    length = 13 + len(payload)                       # 11 шапка + payload + 2 crc16
    hdr = bytes([0x55, length & 0xFF,
                 ((length >> 8) & 0x03) | ((version & 0x3F) << 2)])
    hdr += bytes([crc8(hdr)])
    body = hdr + bytes([src, dst, seq & 0xFF, (seq >> 8) & 0xFF, attr, cmdset, cmdid]) + bytes(payload)
    return body + crc16(body).to_bytes(2, "little")

def parse_fields(pkt):
    return dict(src=pkt[4], dst=pkt[5], seq=pkt[6] | (pkt[7] << 8),
                attr=pkt[8], cmdset=pkt[9], cmdid=pkt[10], payload=pkt[11:-2])

class FrameStream:
    """Накапливает байты, выдаёт валидные (по CRC) DUML-кадры."""
    def __init__(self): self.buf = bytearray()
    def feed(self, data):
        self.buf += data
        out = []
        i = 0
        n = len(self.buf)
        while i < n - 4:
            if self.buf[i] != 0x55:
                i += 1; continue
            length = ((self.buf[i+2] & 0x03) << 8) | self.buf[i+1]
            if length < 13 or length > 1024:
                i += 1; continue
            if i + length > n:
                break                                # ждём ещё байт
            pkt = bytes(self.buf[i:i+length])
            if crc8(pkt[0:3]) != pkt[3]:
                i += 1; continue
            if crc16(pkt[0:length-2]) != (pkt[length-2] | (pkt[length-1] << 8)):
                i += 1; continue
            out.append(pkt); i += length
        del self.buf[:i]
        return out

# ---------------- человекочитаемый разбор известных команд ----------------
def u16(p, o): return p[o] | (p[o+1] << 8) if o+1 < len(p) else None
def s16(p, o):
    v = u16(p, o)
    return v-65536 if v is not None and v >= 32768 else v
def u32(p, o):
    return (p[o] | (p[o+1]<<8) | (p[o+2]<<16) | (p[o+3]<<24)) if o+3 < len(p) else None
def s32(p, o):
    v = u32(p, o)
    return v-(1<<32) if v is not None and v >= (1<<31) else v
def ascii_at(p, o, n=None):
    s = p[o:o+n] if n else p[o:]
    return "".join(chr(b) for b in s if 32 <= b < 127)

def describe(pkt):
    f = parse_fields(pkt)
    p = f["payload"]; cs, ci = f["cmdset"], f["cmdid"]
    role = "RSP" if (f["attr"] & 0x80) else ("REQ" if (f["attr"] & 0x40) else "PUSH")
    head = f"[{role}] {f['src']:#04x}->{f['dst']:#04x} {cs:#04x}:{ci:#04x} seq={f['seq']}"
    # парсим только ответы/пуши батареи (где есть данные)
    if role == "REQ":
        return head + f"  pl={p.hex()}"
    if (cs, ci) == (0x0d, 0x02) and len(p) >= 22:        # динамический статус
        return (head + f"  SoC={p[21]}% (raw {p[3]}%)  I={s32(p,6)}mA"
                f"  rem={u32(p,14)}mAh  T={u16(p,18)/10:.1f}C  {p[20]}S")
    if (cs, ci) == (0x0d, 0x01) and len(p) >= 32:        # статика
        return (head + f"  модель={ascii_at(p,26,6).strip(chr(0))}  design={u16(p,2)}mAh"
                f"  циклы={u16(p,6)}")
    if (cs, ci) == (0x0d, 0x03) and len(p) >= 3:         # напряжения банок
        n = p[2]; cells = [u16(p, 3+2*k) for k in range(n) if 3+2*k+1 < len(p)]
        return head + f"  банок={n}  мВ={cells}  пакет={sum(c for c in cells if c)}мВ"
    if (cs, ci) == (0x0d, 0x04) and len(p) >= 3:         # серийник
        return head + f"  S/N={ascii_at(p,3,p[2])}"
    if (cs, ci) == (0x0d, 0xc0) and len(p) >= 16:        # лимиты/заряд
        return (head + f"  порог={u16(p,2)}мВ  cap={u16(p,4)}мАh  [8]={p[8]}"
                f"  заряд_флаг@14={u16(p,14):#06x}")
    if (cs, ci) == (0x0d, 0xda) and len(p) >= 12:        # термо/зарядка
        return (head + f"  Vпак={u16(p,10)}мВ  Vзар[8]={u16(p,8)}мВ"
                f"  T[2]={u16(p,2)/10:.1f} T[4]={u16(p,4)/10:.1f} T[12]={u16(p,12)/10:.1f}C")
    if (cs, ci) == (0x00, 0x01) and len(p) >= 3:         # версия
        return head + f"  ascii=|{ascii_at(p,2)}|  pl={p.hex()}"
    return head + f"  pl[{len(p)}]={p.hex()}"

# ---------------- набор read-команд (payload как у хаба) ----------------
READS = {
    "status":  (0x0d, 0x02, bytes.fromhex("01")),
    "static":  (0x0d, 0x01, bytes.fromhex("01"+"00"*8)),
    "cells":   (0x0d, 0x03, bytes.fromhex("01000000")),
    "serial":  (0x0d, 0x04, bytes.fromhex("01"+"00"*8)),
    "limits":  (0x0d, 0xc0, bytes.fromhex("01"+"00"*8)),
    "thermo":  (0x0d, 0xda, bytes.fromhex("01")),
    "version": (0x00, 0x01, bytes.fromhex("00")),
}
POLL_ORDER = ["status", "cells", "thermo", "limits", "static", "serial", "version"]

# ---------------- self-test: сборщик == реальный кадр хаба ----------------
def selftest():
    ref = bytes.fromhex("550e04664b0b0200400d02013c26")          # реальный 0d:02 хаба
    got = build_frame(0x4b, 0x0b, 0x0d, 0x02, b"\x01", attr=0x40, seq=2)
    assert got == ref, f"build_frame сломан:\n  ждали {ref.hex()}\n  вышло {got.hex()}"
    ref2 = bytes.fromhex("551104924b0b0400400d03010000003870")   # реальный 0d:03 хаба
    got2 = build_frame(0x4b, 0x0b, 0x0d, 0x03, bytes.fromhex("01000000"), attr=0x40, seq=4)
    assert got2 == ref2, f"build_frame сломан (0d:03):\n  {ref2.hex()}\n  {got2.hex()}"

# ---------------- транспорт ----------------
def open_port(port, baud=115200):
    import serial
    return serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1,
                         timeout=0.1, write_timeout=1)

def drain(ser, fs, seconds):
    """Читать seconds сек, вернуть список разобранных кадров."""
    frames = []
    t_end = time.time() + seconds
    while time.time() < t_end:
        data = ser.read(4096)
        if data:
            frames += fs.feed(data)
    return frames

def cmd_sniff(ser, seconds):
    fs = FrameStream()
    print(f"[sniff] слушаю {seconds:.0f} c на {ser.port} @115200 8N1 ...")
    t_end = time.time() + seconds
    total = raw = 0
    while time.time() < t_end:
        data = ser.read(4096)
        if data:
            raw += len(data)
            for pkt in fs.feed(data):
                total += 1
                print("  " + describe(pkt))
    print(f"[sniff] байт принято: {raw}, валидных DUML-кадров: {total}")
    if raw == 0:
        print("  ⚠ 0 байт — RX не на линии ответов батареи, либо батарея молчит (нажми кнопку).")
    elif total == 0:
        print("  ⚠ байты есть, но ни одного валидного кадра — проверь бод/полярность/землю.")

def request(ser, name, seq, settle=0.25):
    cs, ci, pl = READS[name]
    fs = FrameStream()
    ser.reset_input_buffer()
    ser.write(build_frame(0x4b, 0x0b, cs, ci, pl, attr=0x40, seq=seq))
    ser.flush()
    frames = drain(ser, fs, settle)
    # вернём ответ батареи на эту команду, если есть
    for pkt in frames:
        f = parse_fields(pkt)
        if f["cmdset"] == cs and f["cmdid"] == ci and (f["attr"] & 0x80):
            return pkt, frames
    return None, frames

def send_recv(ser, cmdset, cmdid, payload, seq, settle=0.15, attr=0x40):
    """Послать произвольную команду, вернуть (ответ-на-неё | None, все_кадры)."""
    fs = FrameStream()
    ser.reset_input_buffer()
    ser.write(build_frame(0x4b, 0x0b, cmdset, cmdid, payload, attr=attr, seq=seq))
    ser.flush()
    frames = drain(ser, fs, settle)
    rsp = None
    for pkt in frames:
        f = parse_fields(pkt)
        if f["cmdset"] == cmdset and f["cmdid"] == cmdid and (f["attr"] & 0x80):
            rsp = pkt
    return rsp, frames

def canary(ser):
    """Снимок здоровья батареи: (S/N, циклы, SoC%) — для сверки до/после теста."""
    sn = cyc = soc = None
    r, _ = send_recv(ser, 0x0d, 0x04, bytes.fromhex("01"+"00"*8), 0x101, 0.25)
    if r: sn = ascii_at(r[11:-2], 3, r[13])
    r, _ = send_recv(ser, 0x0d, 0x01, bytes.fromhex("01"+"00"*8), 0x102, 0.25)
    if r: cyc = u16(r[11:-2], 6)
    r, _ = send_recv(ser, 0x0d, 0x02, bytes.fromhex("01"), 0x103, 0.25)
    if r: p = r[11:-2]; soc = p[21] if len(p) > 21 else None
    return sn, cyc, soc

def cmd_sweep(ser, cmdset):
    """Безопасный read-перебор CmdId 0..255 в наборе cmdset. Канарейка до/после."""
    print(f"[sweep] набор {cmdset:#04x}: канарейка ДО ...")
    before = canary(ser)
    print(f"   S/N={before[0]} циклы={before[1]} SoC={before[2]}%")
    if before == (None, None, None):
        print("   ⚠ батарея не отвечает — прерываю sweep."); return
    print(f"[sweep] перебираю {cmdset:#04x}:00..ff read-запросом (payload=01)...")
    responders = {}
    seq = 0x200
    silent_streak = 0
    for cid in range(256):
        rsp, frames = send_recv(ser, cmdset, cid, b"\x01", seq, settle=0.12); seq += 1
        if frames:
            silent_streak = 0
        else:
            silent_streak += 1
        if rsp:
            p = rsp[11:-2]
            responders[cid] = p
            asc = "".join(chr(b) if 32 <= b < 127 else "." for b in p)
            print(f"   {cmdset:#04x}:{cid:#04x}  RSP pl[{len(p):2}]={p.hex()}  |{asc}|")
        elif frames:                                     # ответ есть, но с другим id (push/иное)
            for pkt in frames:
                f = parse_fields(pkt)
                print(f"   {cmdset:#04x}:{cid:#04x}  -> кадр {f['cmdset']:#x}:{f['cmdid']:#x} attr={f['attr']:#x} pl={f['payload'].hex()}")
        # канарейка-страховка: если батарея вдруг замолчала надолго посреди живого набора
        if silent_streak == 24 and responders:
            print(f"   ⚠ 24 команды подряд без единого кадра (id≈{cid:#x}) — пауза, проверяю живость...")
            if canary(ser) == (None, None, None):
                print("   ⚠⚠ батарея перестала отвечать! ОСТАНАВЛИВАЮ sweep."); break
            silent_streak = 0
    print(f"[sweep] набор {cmdset:#04x}: канарейка ПОСЛЕ ...")
    after = canary(ser)
    print(f"   S/N={after[0]} циклы={after[1]} SoC={after[2]}%")
    if after != before:
        print(f"   ⚠ СОСТОЯНИЕ ИЗМЕНИЛОСЬ: до={before} после={after} — разберись, что сработало!")
    else:
        print("   ✓ состояние батареи не изменилось (S/N/циклы/SoC те же)")
    print(f"[sweep] откликнулось команд: {len(responders)} -> {[hex(c) for c in responders]}")
    return responders

def cmd_monitor(ser):
    """Живой компактный дашборд: статус+банки+термо одной строкой, обновление."""
    print("[monitor] Ctrl-C для выхода")
    seq = 0x300
    while True:
        st, _ = send_recv(ser, 0x0d, 0x02, b"\x01", seq); seq += 1
        cl, _ = send_recv(ser, 0x0d, 0x03, bytes.fromhex("01000000"), seq); seq += 1
        th, _ = send_recv(ser, 0x0d, 0xda, b"\x01", seq); seq += 1
        if not st:
            print("  нет ответа..."); time.sleep(0.3); continue
        p = st[11:-2]
        soc, raw = p[21], p[3]; cur = s32(p, 6); rem = u32(p, 14); temp = u16(p, 18)/10; ncell = p[20]
        cells = ""
        if cl:
            q = cl[11:-2]; n = q[2]
            cells = " ".join(str(u16(q, 3+2*k)) for k in range(n))
        vpack = ""
        if th:
            r = th[11:-2]; vpack = f" Vпак={u16(r,10)}мВ"
        flag = "ЗАРЯД" if cur > 50 else ("РАЗРЯД" if cur < -50 else "покой")
        print(f"  SoC={soc:3}%(raw{raw}) I={cur:6}мА {flag:6} rem={rem}мАч T={temp:.1f}C "
              f"{ncell}S [{cells}]мВ{vpack}")
        time.sleep(0.5)

def cmd_read(ser, name):
    if name not in READS:
        print(f"неизвестная команда '{name}'. Доступно: {', '.join(READS)}"); return
    rsp, frames = request(ser, name, seq=1)
    if rsp:
        print("  " + describe(rsp))
    else:
        print(f"  нет ответа на {name}."
              + ("  (другие кадры: " + ", ".join(f"{parse_fields(p)['cmdset']:#x}:{parse_fields(p)['cmdid']:#x}" for p in frames) + ")" if frames else "  тишина — TX не на линии запросов, либо батарея спит."))

def cmd_poll(ser, count):
    seq = 1
    cycle = 0
    print("[poll] Ctrl-C для выхода")
    while count == 0 or cycle < count:
        cycle += 1
        print(f"--- цикл {cycle} ---")
        for name in POLL_ORDER:
            rsp, _ = request(ser, name, seq); seq += 1
            tag = f"{name:8}"
            print(f"  {tag} " + (describe(rsp) if rsp else "нет ответа"))
        time.sleep(0.2)

def cmd_send(ser, cmdset, cmdid, payload):
    fs = FrameStream()
    frame = build_frame(0x4b, 0x0b, cmdset, cmdid, payload, attr=0x40, seq=1)
    print(f"[send] -> {frame.hex()}")
    ser.reset_input_buffer(); ser.write(frame); ser.flush()
    frames = drain(ser, fs, 0.4)
    if not frames:
        print("  нет ответа.")
    for pkt in frames:
        print("  " + describe(pkt))

# ---------------- main ----------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["sniff", "poll", "read", "send", "sweep", "monitor", "selftest"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--port", default="/dev/ttyACM0")
    a = ap.parse_args()

    selftest()                                       # всегда проверяем CRC/сборку
    if a.mode == "selftest":
        print("selftest OK: build_frame побайтно совпал с реальными кадрами хаба"); return

    ser = open_port(a.port)
    try:
        if a.mode == "sniff":
            cmd_sniff(ser, float(a.args[0]) if a.args else 5.0)
        elif a.mode == "poll":
            cmd_poll(ser, int(a.args[0]) if a.args else 0)
        elif a.mode == "read":
            cmd_read(ser, a.args[0] if a.args else "status")
        elif a.mode == "send":
            cs = int(a.args[0], 16); ci = int(a.args[1], 16)
            pl = bytes.fromhex(a.args[2]) if len(a.args) > 2 else b""
            cmd_send(ser, cs, ci, pl)
        elif a.mode == "sweep":
            cmd_sweep(ser, int(a.args[0], 16) if a.args else 0x0d)
        elif a.mode == "monitor":
            cmd_monitor(ser)
    except KeyboardInterrupt:
        print("\n[выход]")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
