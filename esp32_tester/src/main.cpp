/* DJI Smart-Battery тестер (Mavic 4 / WA341 и совместимые) на UART/DUML.
 * Плата: Waveshare ESP32-S3-Touch-LCD-2 (ST7789T3 240x320).
 * Батарея: на силк-пины TX(GPIO43)->RX батареи, RX(GPIO44)<-TX батареи, GND.
 *   Батарею включать НЕ нужно — гейдж всегда отвечает. 3.3В, без сдвигателя.
 * Протокол/CRC/офсеты портированы из bat_serial.py / PROTOCOL.md этого проекта.
 */
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// ---------- цвета RGB565 (эта версия GFX их не объявляет) ----------
#define BLACK    0x0000
#define WHITE    0xFFFF
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define YELLOW   0xFFE0
#define ORANGE   0xFD20
#define DARKGREY 0x7BEF

// ---------- пины LCD (из схемы ESP32-S3-Touch-LCD-2) ----------
#define LCD_MOSI 38
#define LCD_SCLK 39
#define LCD_DC   42
#define LCD_CS   45
#define LCD_RST   0
#define LCD_BL    1
// ---------- UART к батарее (силк TX/RX = UART0-пины) ----------
#define BAT_TX   43   // -> RX батареи
#define BAT_RX   44   // <- TX батареи
#define BAT_BAUD 115200
// ---------- кнопка питания + ADC встроенного LiPo платы ----------
#define BTN_PWR     18       // тактовая кнопка: GPIO18 -> кнопка -> GND (выведен на гребёнку)
#define PWR_HOLD_MS 1200     // удержание кнопки для выключения
#define BAT_ADC_PIN 5        // BAT_ADC, делитель VBAT/3 (R19 200K / R20 100K)

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0 /*rot*/, true /*IPS*/, 240, 320);
HardwareSerial Bat(1);   // UART1 на пины 43/44

// ============ CRC-таблицы DJI DUML (из duml.py) ============
static const uint8_t TBL8[256] = {
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
0x74,0x2a,0xc8,0x96,0x15,0x4b,0xa9,0xf7,0xb6,0xe8,0x0a,0x54,0xd7,0x89,0x6b,0x35};
static const uint16_t TBL16[256] = {
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
0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78};

static uint8_t crc8(const uint8_t *d, int n, uint8_t s = 0x77) {
  for (int i = 0; i < n; i++) s = TBL8[(s ^ d[i]) & 0xFF];
  return s;
}
static uint16_t crc16(const uint8_t *d, int n, uint16_t s = 0x3692) {
  for (int i = 0; i < n; i++) s = ((s >> 8) & 0xFF) ^ TBL16[(s ^ d[i]) & 0xFF];
  return s;
}

// ---- сборка кадра DUML; возвращает длину ----
static int buildFrame(uint8_t *out, uint8_t cs, uint8_t ci,
                      const uint8_t *pl, int plen, uint16_t seq, uint8_t attr = 0x40) {
  int len = 13 + plen;
  out[0] = 0x55; out[1] = len & 0xFF; out[2] = ((len >> 8) & 0x03) | (1 << 2);
  out[3] = crc8(out, 3);
  out[4] = 0x4b; out[5] = 0x0b;                 // src=мастер, dst=батарея
  out[6] = seq & 0xFF; out[7] = (seq >> 8) & 0xFF;
  out[8] = attr; out[9] = cs; out[10] = ci;
  for (int i = 0; i < plen; i++) out[11 + i] = pl[i];
  uint16_t c = crc16(out, len - 2);
  out[len - 2] = c & 0xFF; out[len - 1] = (c >> 8) & 0xFF;
  return len;
}

// ---- послать запрос и дождаться ответа на ту же команду; вернуть длину payload ----
static int sendRecv(uint8_t cs, uint8_t ci, const uint8_t *pl, int plen,
                    uint8_t *payOut, int maxPay, uint16_t seq, uint32_t to_ms = 220) {
  while (Bat.available()) Bat.read();
  uint8_t fr[40]; int fl = buildFrame(fr, cs, ci, pl, plen, seq);
  Bat.write(fr, fl); Bat.flush();
  uint8_t buf[400]; int n = 0; uint32_t t0 = millis();
  while (millis() - t0 < to_ms) {
    while (Bat.available() && n < (int)sizeof(buf)) buf[n++] = Bat.read();
    for (int i = 0; i + 4 < n; i++) {                 // ищем валидный кадр
      if (buf[i] != 0x55) continue;
      int len = ((buf[i + 2] & 0x03) << 8) | buf[i + 1];
      if (len < 13 || i + len > n) continue;
      if (crc8(&buf[i], 3) != buf[i + 3]) continue;
      uint16_t c = crc16(&buf[i], len - 2);
      if ((c & 0xFF) != buf[i + len - 2] || ((c >> 8) & 0xFF) != buf[i + len - 1]) continue;
      if (buf[i + 9] == cs && buf[i + 10] == ci && (buf[i + 8] & 0x80)) {
        int pn = len - 13; if (pn > maxPay) pn = maxPay;
        for (int k = 0; k < pn; k++) payOut[k] = buf[i + 11 + k];
        return pn;
      }
    }
  }
  return -1;
}

// ---- маленькие распаковщики ----
static uint16_t u16(const uint8_t *p, int o) { return p[o] | (p[o + 1] << 8); }
static int32_t  s32(const uint8_t *p, int o) { return (int32_t)(p[o] | (p[o+1]<<8) | (p[o+2]<<16) | ((uint32_t)p[o+3]<<24)); }
static uint32_t u32(const uint8_t *p, int o) { return p[o] | (p[o+1]<<8) | (p[o+2]<<16) | ((uint32_t)p[o+3]<<24); }

// ---- состояние батареи ----
struct Bat_t {
  bool ok = false;
  int soc = 0, rawSoc = 0, ncell = 0, cycles = 0;
  int32_t current = 0;
  uint32_t remain = 0; uint16_t fcc = 0, design = 0;
  float temp = 0; int packMv = 0; int cell[8] = {0};
  char model[8] = "?", sn[20] = "?";
  uint8_t fw[8] = {0}; bool fwOk = false;
} B;

static const uint8_t PL01[9] = {1,0,0,0,0,0,0,0,0};
static const uint8_t PL1[1]  = {1};
static const uint8_t PL_CELLS[4] = {1,0,0,0};
static const uint8_t PL00[1] = {0};

static bool poll() {
  uint8_t p[64]; uint16_t seq = millis() & 0xFFFF;
  int n;
  Bat_t nb;                                  // во временный, заполним и скопируем при успехе
  n = sendRecv(0x0d, 0x02, PL1, 1, p, sizeof p, seq++);   // динамика
  if (n >= 22) {
    nb.ok = true; nb.packMv = u16(p, 2); nb.rawSoc = p[3]; nb.current = s32(p, 6);
    nb.fcc = u16(p, 10); nb.remain = u32(p, 14); nb.temp = (int16_t)u16(p, 18) / 10.0f;  // i16 ÷10 (signed: ниже 0°C)
    nb.ncell = p[20]; nb.soc = p[21];
  } else return false;
  n = sendRecv(0x0d, 0x03, PL_CELLS, 4, p, sizeof p, seq++); // банки (для отображения)
  if (n >= 3) {
    int c = p[2]; if (c > 8) c = 8; nb.ncell = c;
    for (int k = 0; k < c; k++) nb.cell[k] = u16(p, 3 + 2 * k);
  }
  n = sendRecv(0x0d, 0x01, PL01, 9, p, sizeof p, seq++);   // статика
  if (n >= 32) {
    nb.design = u16(p, 2); nb.cycles = u16(p, 6);
    for (int k = 0; k < 6; k++) { char ch = p[26 + k]; nb.model[k] = (ch >= 32 && ch < 127) ? ch : 0; }
    nb.model[6] = 0;
  }
  n = sendRecv(0x0d, 0x04, PL01, 9, p, sizeof p, seq++);   // серийник
  if (n >= 4) {
    int sl = p[2]; if (sl > 18) sl = 18; int j = 0;
    for (int k = 0; k < sl; k++) { char ch = p[3 + k]; if (ch >= 32 && ch < 127) nb.sn[j++] = ch; }
    nb.sn[j] = 0;
  }
  n = sendRecv(0x00, 0x01, PL00, 1, p, sizeof p, seq++);   // версия прошивки (GetVersion)
  if (n >= 12) { for (int k = 0; k < 8; k++) nb.fw[k] = p[n - 12 + k]; nb.fwOk = true; }
  B = nb;
  return true;
}

// ============ отрисовка ============
#define BG       BLACK
static void label(int x, int y, const char *t, uint16_t col, uint8_t sz = 1) {
  gfx->setTextSize(sz); gfx->setTextColor(col); gfx->setCursor(x, y); gfx->print(t);
}
static void value(int x, int y, int w, uint16_t col, uint8_t sz, const char *fmt, ...) {
  char b[40]; va_list a; va_start(a, fmt); vsnprintf(b, sizeof b, fmt, a); va_end(a);
  gfx->fillRect(x, y, w, 8 * sz, BG);
  gfx->setTextSize(sz); gfx->setTextColor(col); gfx->setCursor(x, y); gfx->print(b);
}

static void drawStatic() {
  gfx->fillScreen(BG);
  label(6, 2, "MAVIC 4 PRO", CYAN, 2);
  label(6, 19, "Battery Tester", DARKGREY, 1);
  gfx->drawFastHLine(0, 28, 240, DARKGREY);
}

static int g_lastOk = -1;
static void drawValues() {
  if ((int)B.ok != g_lastOk) { drawStatic(); g_lastOk = (int)B.ok; }  // полная очистка при смене

  if (!B.ok) {
    value(6, 110, 234, RED, 3, "NO BATTERY");
    value(6, 150, 234, DARKGREY, 1, "TX43->batRX  RX44<-batTX  GND");
    return;
  }
  // число банок
  value(6, 32, 234, YELLOW, 3, "%dS", B.ncell);
  // серийник
  value(6, 60, 234, WHITE, 2, "SN %s", B.sn);
  // крупный SoC (цвет по уровню)
  uint16_t sc = B.soc >= 50 ? GREEN : (B.soc >= 20 ? YELLOW : RED);
  value(6, 84, 234, sc, 6, "%d%%", B.soc);
  // циклы + температура
  value(6, 140, 124, WHITE, 2, "Cycles %d", B.cycles);
  value(132, 140, 108, WHITE, 2, "T %.1fC", B.temp);
  // банки (мВ), 2 в ряд; жёлтым если вышли за 3.30..4.25 В
  value(6, 166, 234, CYAN, 1, "Cells:");
  for (int k = 0; k < B.ncell && k < 4; k++) {
    uint16_t cc = (B.cell[k] >= 4250 || (B.cell[k] && B.cell[k] <= 3300)) ? ORANGE : GREEN;
    value(6 + (k % 2) * 118, 178 + (k / 2) * 18, 112, cc, 2, "C%d %dmV", k + 1, B.cell[k]);
  }
  // напряжение пакета (прямое поле гейджа 0d:02[2])
  value(6, 216, 234, WHITE, 2, "Pack %d.%03dV", B.packMv / 1000, B.packMv % 1000);
  // прошивка батареи: дескриптор -> vNN.N.N.NN (формат сверен с FDR-логом Mavic 4)
  if (B.fwOk)
    value(6, 244, 234, CYAN, 2, "FW v%d.%d.%d.%d", B.fw[7], B.fw[6], B.fw[5], B.fw[4]);
  else
    value(6, 244, 234, DARKGREY, 1, "FW ?");
}

// ---- встроенный LiPo платы: сглаженная полоска-индикатор снизу ----
// Как в офиц. демо Waveshare: BAT_ADC через делитель 200K/100K (÷3) -> напряжение.
// Статуса заряда на плате нет на GPIO (STAT только на красном LED) — показываем уровень.
static void drawBatBar() {
  static float ema = 0;
  uint32_t s = 0;
  for (int i = 0; i < 16; i++) s += analogReadMilliVolts(BAT_ADC_PIN);
  int raw = (int)(s / 16) * 3;                 // делитель VBAT/3
  if (ema < 1) ema = raw;
  ema = ema * 0.9f + raw * 0.1f;               // сглаживание (фикс дёрганья)
  int mv = (int)ema;
  int pct = (mv - 3300) * 100 / 900;           // 3.30В=0%, 4.20В=100%
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  uint16_t c = pct >= 50 ? GREEN : (pct >= 20 ? YELLOW : RED);
  char b[28]; snprintf(b, sizeof b, "Tester batt: %d.%02dV  %d%%", mv / 1000, (mv % 1000) / 10, pct);
  gfx->fillRect(0, 292, 240, 10, BLACK);
  gfx->setTextSize(1); gfx->setTextColor(DARKGREY); gfx->setCursor(6, 294); gfx->print(b);
  int x = 2, y = 308, w = 236, h = 10;
  gfx->drawRect(x, y, w, h, DARKGREY);
  gfx->fillRect(x + 1, y + 1, w - 2, h - 2, BLACK);
  gfx->fillRect(x + 1, y + 1, (w - 2) * pct / 100, h - 2, c);
}

// ---- мягкое выключение: deep sleep, пробуждение по кнопке ----
static void powerOff() {
  gfx->fillScreen(BLACK);
  gfx->setTextSize(3); gfx->setTextColor(WHITE); gfx->setCursor(72, 140);
  gfx->print("OFF");
  delay(500);
  digitalWrite(LCD_BL, LOW);                   // погасить подсветку (главный потребитель)
  gpio_hold_en((gpio_num_t)LCD_BL);            // держать LOW во сне
  gpio_deep_sleep_hold_en();
  while (digitalRead(BTN_PWR) == LOW) delay(10);   // дождаться отпускания
  delay(50);
  rtc_gpio_pullup_en((gpio_num_t)BTN_PWR);     // подтяжка кнопки во сне
  rtc_gpio_pulldown_dis((gpio_num_t)BTN_PWR);
  esp_sleep_enable_ext1_wakeup(1ULL << BTN_PWR, ESP_EXT1_WAKEUP_ANY_LOW);
  esp_deep_sleep_start();                        // ~десятки мкА; включение — нажатием кнопки
}

void setup() {
  Serial.begin(115200);                       // USB-CDC лог
  setCpuFrequencyMhz(80);                     // ниже активное потребление
  pinMode(BTN_PWR, INPUT_PULLUP);
  gpio_hold_dis((gpio_num_t)LCD_BL);          // снять фиксацию подсветки после сна
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1)
    while (digitalRead(BTN_PWR) == LOW) delay(10);   // дождаться отпускания кнопки включения
  pinMode(LCD_BL, OUTPUT); digitalWrite(LCD_BL, HIGH);
  analogReadResolution(12);
  Bat.begin(BAT_BAUD, SERIAL_8N1, BAT_RX, BAT_TX);
  gfx->begin();
  drawStatic();
}

uint32_t lastPoll = 0, btnDownAt = 0;
void loop() {
  // кнопка питания: длинное удержание -> выключение (deep sleep)
  if (digitalRead(BTN_PWR) == LOW) {
    if (!btnDownAt) btnDownAt = millis();
    else if (millis() - btnDownAt > PWR_HOLD_MS) powerOff();
  } else btnDownAt = 0;

  if (millis() - lastPoll > 1000) {           // опрос 1 Гц (экономия)
    lastPoll = millis();
    bool got = poll();
    drawValues();
    drawBatBar();                             // полоска заряда контроллера
    if (got)
      Serial.printf("SoC=%d%% I=%ldmA T=%.1fC %dS pack=%dmV cyc=%d FCC=%d FW v%d.%d.%d.%d %s %s\n",
                    B.soc, (long)B.current, B.temp, B.ncell, B.packMv, B.cycles, B.fcc,
                    B.fw[7], B.fw[6], B.fw[5], B.fw[4], B.model, B.sn);
    else
      Serial.println("нет ответа батареи (проверь TX/RX/GND)");
  }
  delay(20);                                  // не крутить CPU вхолостую (экономия + отзыв кнопки)
}
