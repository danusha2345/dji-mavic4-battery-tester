> ☕ **Support the author / Поддержать автора → [boosty.to/danusha](https://boosty.to/danusha/donate)**

# DJI Mavic 4 (Pro) Smart Battery — UART/DUML tools & ESP32 tester

Reverse-engineered the **DUML-over-UART** protocol of the DJI **Mavic 4 / Mavic 4 Pro** smart
battery (model **WA341**, 4S) and built two things:

* a **Python toolkit** to talk to the battery live over any 3.3 V USB-UART adapter, and
* a **handheld ESP32-S3 tester** with an on-screen dashboard (Waveshare ESP32-S3-Touch-LCD-2).

The battery's fuel-gauge MCU is always powered from the cells, so **it answers over UART even
when the battery is "off"** — no button, no drone, no hub. Three wires (RX/TX/GND) and you read
state of charge, per-cell voltages, cycles, temperature, capacities, serial number and more.

| ESP32 tester reading a DJI Mavic 4 Pro battery | UART tap into the battery contacts (TX / RX / GND) |
|:--:|:--:|
| ![ESP32 tester dashboard: SoC, cells, cycles, pack voltage](photo_2026-05-30_17-41-34.jpg) | ![UART wiring into the battery](photo_2026-05-30_17-41-57.jpg) |

> ⚠️ For your **own** batteries and authorized research/repair only. You talk to the gauge at your
> own risk. Read-only polling is safe; do not blindly send unknown write/config commands.

![lang](https://img.shields.io/badge/docs-EN%20%7C%20RU-blue) · [English](#english) · [Русский](#русский)

---

## English

### What's inside
| File | Purpose |
|------|---------|
| [`PROTOCOL.md`](PROTOCOL.md) | Full protocol write-up — framing, CRC, addressing, command map, field offsets |
| `bat_serial.py` | Live tool: `sniff` / `poll` / `read` / `monitor` / `sweep` / `send` / `selftest` |
| `duml.py`, `uart_decode.py`, `analyze.py` | Offline decoders for logic-analyzer CSV captures |
| `find_specs.py` | Known-plaintext search for spec values across payloads |
| `esp32_tester/` | PlatformIO firmware for the Waveshare ESP32-S3-Touch-LCD-2 tester |
| `*.csv` | Reference logic-analyzer captures (charging in a hub / powered by button) |

### Protocol at a glance
* **UART 115200 8N1**, idle-high TTL (3.3 V). Two unidirectional lines in the OEM hub (request/answer);
  one full-duplex UART (TX+RX+GND) is enough to act as the master.
* **DUML** framing: `55 | len | ver | crc8(hdr) | src | dst | seq | attr | cmdset | cmdid | payload | crc16`.
  Header **CRC-8** seed `0x77`, whole-frame **CRC-16** seed `0x3692` (standard DJI tables).
* Device IDs: `0x0b` = smart battery, `0x4b` = hub master, `0x02` = host/broadcast. **CmdSet `0x0d`** = battery.
* Status byte `[0]` of a reply: `0x00` OK, **`0xe0` = command not supported (NACK)**.

### Useful commands (CmdSet `0x0d`)
| Cmd | Returns |
|-----|---------|
| `0d:02` | dynamic status: SoC %, current, **full-charge capacity (FCC)**, remaining, temperature, pack voltage |
| `0d:03` | per-cell voltages (4× mV) |
| `0d:01` | static: design capacity, **charge cycles**, model |
| `0d:04` | serial number |
| `0d:c0` | charge limits / flags |
| `0d:da` | thermal + charge telemetry |
| `00:01` | firmware GetVersion |

Field offsets (e.g. `0d:02[10]` = FCC, `0d:02[21]` = displayed SoC, `SoC ≈ remaining/FCC`) are in
[`PROTOCOL.md`](PROTOCOL.md). A `0x0e` diagnostic catalog dumps ~50 named gauge variables.

### Wiring (battery ↔ adapter / ESP32)
The gauge idles its TX line **high**. So:

| adapter/board | ↔ battery |
|---|---|
| **RX** | ← battery **TX** pad (idle **HIGH**) |
| **TX** | → battery **RX** pad (idle **LOW**) |
| **GND** | ← common ground (**required**) |

3.3 V logic — no level shifter needed for an ESP32. If you see nothing, you most likely swapped TX/RX
or forgot GND (the line goes silent, not the battery sleeping).

### Python tool
```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install pyserial numpy

.venv/bin/python bat_serial.py selftest                 # verify CRC/build vs real hub frames
.venv/bin/python bat_serial.py monitor --port /dev/ttyUSB0   # live dashboard
.venv/bin/python bat_serial.py read   status --port /dev/ttyUSB0
.venv/bin/python bat_serial.py sweep  0d --port /dev/ttyUSB0  # safe command discovery (with health canary)
.venv/bin/python bat_serial.py send   0d 02 01 --port /dev/ttyUSB0
```

### ESP32 tester (Waveshare ESP32-S3-Touch-LCD-2)
LCD ST7789T3 240×320. Battery on the silk **TX/RX** header (UART0 pins, free because flashing/console
go over USB-C). Shows: SoC %, cycles, temperature, per-cell mV, pack voltage, serial, firmware.

**Power & low power:** tactile button on **GPIO18** (`GPIO18 → button → GND`): hold ≈1.2 s = off
(deep sleep), press = on (wake). Plus 1 Hz polling and 80 MHz CPU, backlight off in sleep, and a bottom
bar for the tester's own LiPo (BAT_ADC GPIO5, ÷3 divider). Note: the board's red **POWER** LED is wired
to the 3.3 V rail through R22 (not a GPIO), so it stays lit in deep sleep (~0.5 mA) — desolder LED2/R22
(or add a hardware load switch) for a truly low-power off.

```bash
cd esp32_tester
pio run -t upload      # PlatformIO; platform = pioarduino (Arduino-ESP32 core 3.x)
pio device monitor
```

| board (silk) | GPIO | → battery |
|---|---|---|
| **TX** | 43 | → battery RX |
| **RX** | 44 | ← battery TX |
| **GND** | — | ← GND |

LCD pins are fixed in `esp32_tester/src/main.cpp` from the board schematic
(MOSI 38 / SCLK 39 / DC 42 / CS 45 / RST 0 / BL 1).

### Verified facts (live, on real hardware)
* Model WA341, 4S, design **7100 mAh**, learned full-charge **~6198 mAh**, nominal **14.32 V**, rated **6654 mAh**.
* `SoC_display = remaining / FCC` (e.g. 3464 / 6198 ≈ 56 % ≈ shown 55 %).
* The label's rated capacity (6654 mAh) and nominal voltage (14.32 V) are **not** stored in telemetry —
  they're spec constants; the gauge exposes design (7100) and learned FCC (~6198).

---

## Русский

Реверс **DUML-по-UART** умной батареи DJI **Mavic 4 / Mavic 4 Pro** (модель **WA341**, 4S) и два инструмента:

* **Python-набор** для живого общения с батареей через любой 3.3 В USB-UART, и
* **карманный тестер на ESP32-S3** с дашбордом на экране (Waveshare ESP32-S3-Touch-LCD-2).

Гейдж-контроллер батареи всегда запитан от банок, поэтому **отвечает по UART даже на «выключенной»
батарее** — без кнопки, дрона и хаба. Три провода (RX/TX/GND) — и читаешь заряд, напряжения банок,
циклы, температуру, ёмкости, серийник и прочее.

> ⚠️ Только для **своих** батарей и авторизованного ремонта/исследования. Общение с гейджем — на свой
> риск. Чтение безопасно; не слать вслепую неизвестные команды записи/конфигурации.

### Что внутри
| Файл | Назначение |
|------|-----------|
| [`PROTOCOL.md`](PROTOCOL.md) | Полное описание протокола — кадр, CRC, адресация, карта команд, офсеты полей |
| `bat_serial.py` | Live-инструмент: `sniff` / `poll` / `read` / `monitor` / `sweep` / `send` / `selftest` |
| `duml.py`, `uart_decode.py`, `analyze.py` | Офлайн-декодеры CSV-дампов логического анализатора |
| `find_specs.py` | Поиск паспортных значений в payload'ах (known-plaintext) |
| `esp32_tester/` | Прошивка PlatformIO для тестера на Waveshare ESP32-S3-Touch-LCD-2 |
| `*.csv` | Эталонные дампы анализатора (зарядка в хабе / включение кнопкой) |

### Протокол кратко
* **UART 115200 8N1**, idle-high TTL (3.3 В). В заводском хабе две однонаправленные линии (запрос/ответ);
  чтобы быть мастером, достаточно одного дуплексного UART (TX+RX+GND).
* Кадр **DUML**: `55 | len | ver | crc8(hdr) | src | dst | seq | attr | cmdset | cmdid | payload | crc16`.
  Заголовок **CRC-8** seed `0x77`, весь кадр **CRC-16** seed `0x3692` (стандартные таблицы DJI).
* Адреса: `0x0b` = батарея, `0x4b` = мастер хаба, `0x02` = хост/broadcast. **CmdSet `0x0d`** = батарея.
* Байт статуса `[0]` ответа: `0x00` OK, **`0xe0` = команда не поддерживается (NACK)**.

### Полезные команды (CmdSet `0x0d`)
| Cmd | Что отдаёт |
|-----|-----------|
| `0d:02` | динамика: SoC %, ток, **полная ёмкость (FCC)**, остаток, температура, напряжение пакета |
| `0d:03` | напряжения банок (4× мВ) |
| `0d:01` | статика: design-ёмкость, **циклы заряда**, модель |
| `0d:04` | серийный номер |
| `0d:c0` | лимиты/флаги заряда |
| `0d:da` | термо- и зарядная телеметрия |
| `00:01` | версия прошивки (GetVersion) |

Офсеты полей (напр. `0d:02[10]` = FCC, `0d:02[21]` = отображаемый SoC, `SoC ≈ остаток/FCC`) — в
[`PROTOCOL.md`](PROTOCOL.md). Команды `0x0e` отдают диагностический каталог ~50 именованных переменных гейджа.

### Подключение (батарея ↔ адаптер / ESP32)
TX-линия гейджа в покое **высокая**, поэтому:

| адаптер/плата | ↔ батарея |
|---|---|
| **RX** | ← пад **TX** батареи (idle **HIGH**) |
| **TX** | → пад **RX** батареи (idle **LOW**) |
| **GND** | ← общая земля (**обязательно**) |

Логика 3.3 В — для ESP32 сдвигатель не нужен. Если тишина — почти всегда перепутаны TX/RX или забыта GND
(молчит линия, а не «спит» батарея).

### Python-инструмент
```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install pyserial numpy

.venv/bin/python bat_serial.py selftest                       # сверка CRC/сборки с реальными кадрами хаба
.venv/bin/python bat_serial.py monitor --port /dev/ttyUSB0    # живой дашборд
.venv/bin/python bat_serial.py read    status --port /dev/ttyUSB0
.venv/bin/python bat_serial.py sweep   0d --port /dev/ttyUSB0 # безопасный перебор команд (с канарейкой здоровья)
.venv/bin/python bat_serial.py send    0d 02 01 --port /dev/ttyUSB0
```

### Тестер на ESP32 (Waveshare ESP32-S3-Touch-LCD-2)
LCD ST7789T3 240×320. Батарея на силк-пины **TX/RX** (пины UART0 свободны — прошивка/консоль идут по USB-C).
Показывает: SoC %, циклы, температуру, банки в мВ, напряжение пакета, серийник, прошивку.

**Питание и энергосбережение:** тактовая кнопка на **GPIO18** (`GPIO18 → кнопка → GND`): удержание ≈1.2 с =
выкл (deep sleep), нажатие = вкл (пробуждение). Плюс опрос 1 Гц и CPU 80 МГц, подсветка off во сне, снизу —
полоска заряда встроенного LiPo (BAT_ADC GPIO5, делитель ÷3). Замечание: красный **POWER**-светодиод платы
жёстко висит на рельсе 3.3 В через R22 (не GPIO), поэтому во сне горит (~0.5 мА) — для настоящего «выкл»
выпаять LED2/R22 (или поставить аппаратный load switch).

```bash
cd esp32_tester
pio run -t upload      # PlatformIO; платформа = pioarduino (Arduino-ESP32 core 3.x)
pio device monitor
```

| плата (силк) | GPIO | → батарея |
|---|---|---|
| **TX** | 43 | → RX батареи |
| **RX** | 44 | ← TX батареи |
| **GND** | — | ← GND |

Пины LCD зашиты в `esp32_tester/src/main.cpp` по схеме платы (MOSI 38 / SCLK 39 / DC 42 / CS 45 / RST 0 / BL 1).

### Проверено на живом железе
* Модель WA341, 4S, design **7100 мАч**, выученная полная ёмкость **~6198 мАч**, номинал **14.32 В**, рейтинг **6654 мАч**.
* `SoC = остаток / FCC` (напр. 3464 / 6198 ≈ 56 % ≈ показанные 55 %).
* Паспортные ёмкость (6654 мАч) и номинал (14.32 В) в телеметрии **не** хранятся — это константы;
  гейдж отдаёт design (7100) и выученную FCC (~6198).

---

*Built with reverse-engineering, a logic analyzer, and [Claude Code](https://claude.com/claude-code).*
