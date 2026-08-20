// Touch calibration and verification for the Cheeko Gotchi board.
//
// Why this exists as its own sketch: the on-screen keyboard the capture
// firmware needs depends entirely on accurate touch hit-testing, and SKILL.md
// Gotcha 3 warns this is the expensive part of this board -- the panel is
// rotated 90 degrees relative to the display, each axis has its own scale and
// offset, and the constants published in SKILL.md were "measured on a
// reference unit". Guessing here produces a keyboard where taps land on the
// wrong key, which is miserable to debug from inside a bigger program.
//
// Two stages, no serial input required (the whole thing is driven by tapping):
//
//   1. CALIBRATE -- four crosshairs, one at a time, at known screen points.
//      Deliberately NOT on one diagonal: SKILL.md records that two diagonal
//      points make the fit degenerate, because a swapped-axis mapping and an
//      unswapped one both satisfy the data and you cannot tell them apart.
//      Each tap averages several raw samples, then the per-axis linear fit is
//      solved and printed as copy-pasteable C++.
//
//   2. VERIFY -- a 3x4 grid of buttons sized like real keyboard keys. Tapping
//      one lights it up and logs it. If every tap lights the key actually
//      under your finger, the mapping is good enough for a keyboard.
//
// Flash:
//   FQBN="esp32:esp32:XIAO_ESP32S3:PSRAM=opi,PartitionScheme=tinyuf2_noota,UploadSpeed=115200"
//   arduino-cli compile --fqbn "$FQBN" tama_touch_cal
//   arduino-cli upload  --fqbn "$FQBN" -p /dev/cu.usbmodem101 tama_touch_cal

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>

// ---------------------------------------------------------------- pin map
static constexpr int PIN_TOUCH_IRQ = 1;
static constexpr int PIN_POWER_OFF = 2;
static constexpr int PIN_LCD_DC    = 8;
static constexpr int PIN_LCD_SCLK  = 9;
static constexpr int PIN_LCD_MOSI  = 10;
static constexpr int PIN_I2C_SCL   = 11;
static constexpr int PIN_I2C_SDA   = 12;
static constexpr int PIN_LCD_BL    = 13;
static constexpr int PIN_LCD_CS    = 14;
static constexpr int PIN_LCD_RST   = 17;

static constexpr uint8_t CST810_ADDR = 0x15;

static constexpr int LCD_WIDTH  = 240;
static constexpr int LCD_HEIGHT = 296;

static constexpr uint16_t COLOR_BLACK = 0x0000;
static constexpr uint16_t COLOR_WHITE = 0xffff;
static constexpr uint16_t COLOR_GREEN = 0x07e0;
static constexpr uint16_t COLOR_RED   = 0xf800;
static constexpr uint16_t COLOR_AMBER = 0xfd20;
static constexpr uint16_t COLOR_CYAN  = 0x07ff;
static constexpr uint16_t COLOR_GRAY  = 0x8410;

// ---------------------------------------------------------------- display

static void lcdWriteCommand(uint8_t c) {
  digitalWrite(PIN_LCD_DC, LOW);  digitalWrite(PIN_LCD_CS, LOW);
  SPI.write(c);
  digitalWrite(PIN_LCD_CS, HIGH);
}

static void lcdWriteData(uint8_t d) {
  digitalWrite(PIN_LCD_DC, HIGH); digitalWrite(PIN_LCD_CS, LOW);
  SPI.write(d);
  digitalWrite(PIN_LCD_CS, HIGH);
}

static void lcdSetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  lcdWriteCommand(0x2a);
  lcdWriteData(x0 >> 8); lcdWriteData(x0 & 0xff);
  lcdWriteData(x1 >> 8); lcdWriteData(x1 & 0xff);
  lcdWriteCommand(0x2b);
  lcdWriteData(y0 >> 8); lcdWriteData(y0 & 0xff);
  lcdWriteData(y1 >> 8); lcdWriteData(y1 & 0xff);
  lcdWriteCommand(0x2c);
}

static void lcdFillRect(int x, int y, int w, int h, uint16_t color) {
  if (x < 0 || y < 0 || x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
  w = min(w, LCD_WIDTH - x);
  h = min(h, LCD_HEIGHT - y);
  if (w <= 0 || h <= 0) return;

  static uint8_t rowBuf[LCD_WIDTH * 2];
  const int span = min(w, LCD_WIDTH);
  for (int i = 0; i < span; ++i) {
    rowBuf[i * 2] = color >> 8; rowBuf[i * 2 + 1] = color & 0xff;
  }
  lcdSetWindow(x, y, x + w - 1, y + h - 1);
  digitalWrite(PIN_LCD_DC, HIGH); digitalWrite(PIN_LCD_CS, LOW);
  for (int row = 0; row < h; ++row) SPI.writeBytes(rowBuf, span * 2);
  digitalWrite(PIN_LCD_CS, HIGH);
}

static void lcdFillScreen(uint16_t c) { lcdFillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, c); }

static const uint8_t *glyphForChar(char c) {
  static const uint8_t glyphs[][5] = {
      {0x00,0x00,0x00,0x00,0x00}, {0x7e,0x11,0x11,0x11,0x7e},
      {0x7f,0x49,0x49,0x49,0x36}, {0x3e,0x41,0x41,0x41,0x22},
      {0x7f,0x41,0x41,0x22,0x1c}, {0x7f,0x49,0x49,0x49,0x41},
      {0x7f,0x09,0x09,0x09,0x01}, {0x3e,0x41,0x49,0x49,0x7a},
      {0x7f,0x08,0x08,0x08,0x7f}, {0x00,0x41,0x7f,0x41,0x00},
      {0x20,0x40,0x41,0x3f,0x01}, {0x7f,0x08,0x14,0x22,0x41},
      {0x7f,0x40,0x40,0x40,0x40}, {0x7f,0x02,0x0c,0x02,0x7f},
      {0x7f,0x04,0x08,0x10,0x7f}, {0x3e,0x41,0x41,0x41,0x3e},
      {0x7f,0x09,0x09,0x09,0x06}, {0x3e,0x41,0x51,0x21,0x5e},
      {0x7f,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
      {0x01,0x01,0x7f,0x01,0x01}, {0x3f,0x40,0x40,0x40,0x3f},
      {0x1f,0x20,0x40,0x20,0x1f}, {0x7f,0x20,0x18,0x20,0x7f},
      {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07},
      {0x61,0x51,0x49,0x45,0x43}, {0x3e,0x51,0x49,0x45,0x3e},
      {0x00,0x42,0x7f,0x40,0x00}, {0x62,0x51,0x49,0x49,0x46},
      {0x22,0x41,0x49,0x49,0x36}, {0x18,0x14,0x12,0x7f,0x10},
      {0x27,0x45,0x45,0x45,0x39}, {0x3c,0x4a,0x49,0x49,0x30},
      {0x01,0x71,0x09,0x05,0x03}, {0x36,0x49,0x49,0x49,0x36},
      {0x06,0x49,0x49,0x29,0x1e}, {0x40,0x30,0x0c,0x03,0x00},
      {0x00,0x40,0x40,0x40,0x00}, {0x00,0x60,0x60,0x00,0x00},
      {0x08,0x08,0x08,0x08,0x08}, {0x00,0x36,0x36,0x00,0x00},
  };
  if (c >= 'A' && c <= 'Z') return glyphs[1 + c - 'A'];
  if (c >= 'a' && c <= 'z') return glyphs[1 + c - 'a'];
  if (c >= '0' && c <= '9') return glyphs[27 + c - '0'];
  switch (c) {
    case '/': return glyphs[37];
    case '_': return glyphs[38];
    case '.': return glyphs[39];
    case '-': return glyphs[40];
    case ':': return glyphs[41];
    default:  return glyphs[0];
  }
}

static void lcdDrawChar(int x, int y, char c, uint16_t fg, uint16_t bg, int s) {
  const uint8_t *g = glyphForChar(c);
  for (int col = 0; col < 5; ++col) {
    uint8_t bits = g[col];
    for (int row = 0; row < 7; ++row)
      lcdFillRect(x + col * s, y + row * s, s, s, (bits & (1 << row)) ? fg : bg);
  }
}

static void lcdDrawText(int x, int y, const char *t, uint16_t fg, uint16_t bg, int s) {
  while (*t) { lcdDrawChar(x, y, *t, fg, bg, s); x += 6 * s; ++t; }
}

static void lcdDrawTextCentered(int y, const char *t, uint16_t fg, uint16_t bg, int s) {
  const int cw = 6 * s;
  int len = (int)strlen(t);
  const int maxChars = LCD_WIDTH / cw;
  if (len > maxChars) len = maxChars;
  int x = (LCD_WIDTH - len * cw) / 2;
  if (x < 0) x = 0;
  for (int i = 0; i < len; ++i) { lcdDrawChar(x, y, t[i], fg, bg, s); x += cw; }
}

static void lcdInit() {
  pinMode(PIN_LCD_CS, OUTPUT); pinMode(PIN_LCD_DC, OUTPUT);
  pinMode(PIN_LCD_RST, OUTPUT); pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_CS, HIGH); digitalWrite(PIN_LCD_BL, HIGH);
  digitalWrite(PIN_LCD_RST, LOW);  delay(20);
  digitalWrite(PIN_LCD_RST, HIGH); delay(120);
  lcdWriteCommand(0x01); delay(150);
  lcdWriteCommand(0x11); delay(120);
  lcdWriteCommand(0x36); lcdWriteData(0x48);
  lcdWriteCommand(0x3a); lcdWriteData(0x55);
  lcdWriteCommand(0x20);
  lcdWriteCommand(0x13);
  lcdWriteCommand(0x29);
}

// ---------------------------------------------------------------- touch

static bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

// Raw panel coordinates, exactly as the reference sketches read them.
static bool touchReadRaw(int &rawX, int &rawY) {
  uint8_t d[5] = {};
  if (!i2cReadRegs(CST810_ADDR, 0x02, d, sizeof(d))) return false;
  if ((d[0] & 0x0f) == 0) return false;  // no finger down
  rawX = ((d[1] & 0x0f) << 8) | d[2];
  rawY = ((d[3] & 0x0f) << 8) | d[4];
  return true;
}

// Solved by stage 1 and applied in stage 2. Seeded with SKILL.md's published
// numbers so stage 2 is still usable if calibration is skipped, but these get
// overwritten by whatever this unit actually measures.
struct Mapping {
  // screenX = (rawY - yOff) * xNum / xDen + xAdd
  int yOff = 12,  xNum = 180, xDen = 211, xAdd = 30;
  // screenY = (yInv - rawX) * yNum / yDen + yAdd
  int yInv = 260, yNum = 216, yDen = 222, yAdd = 40;
  bool calibrated = false;
} mapping;

static void touchToScreen(int rawX, int rawY, int &sx, int &sy) {
  sx = constrain((rawY - mapping.yOff) * mapping.xNum / mapping.xDen + mapping.xAdd, 0, LCD_WIDTH - 1);
  sy = constrain((mapping.yInv - rawX) * mapping.yNum / mapping.yDen + mapping.yAdd, 0, LCD_HEIGHT - 1);
}

static int medianOf(int *v, int n) {
  for (int i = 1; i < n; ++i) {          // insertion sort, n is tiny
    int k = v[i], j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; --j; }
    v[j + 1] = k;
  }
  return v[n / 2];
}

// Wait for a clean press and return a representative raw coordinate.
//
// Uses a median of samples taken after a settling delay, not a mean of every
// sample while held. A mean over the whole hold smears in the initial contact
// transient and any finger roll, which is how a corner tap ends up reading
// like a center tap.
static constexpr int TAP_SETTLE_SAMPLES = 8;
static constexpr int TAP_KEEP_SAMPLES   = 21;

static bool captureTap(int &outRawX, int &outRawY, uint32_t timeoutMs = 60000) {
  uint32_t deadline = millis() + timeoutMs;
  int rx, ry;

  while (millis() < deadline) { if (touchReadRaw(rx, ry)) break; delay(5); }
  if (millis() >= deadline) return false;

  for (int i = 0; i < TAP_SETTLE_SAMPLES; ++i) {   // discard the transient
    if (!touchReadRaw(rx, ry)) return false;       // released too fast to trust
    delay(6);
  }

  int xs[TAP_KEEP_SAMPLES], ys[TAP_KEEP_SAMPLES];
  int n = 0;
  while (n < TAP_KEEP_SAMPLES && touchReadRaw(rx, ry)) {
    xs[n] = rx; ys[n] = ry; ++n; delay(6);
  }
  if (n < 5) return false;   // too few good samples, caller should redo

  outRawX = medianOf(xs, n);
  outRawY = medianOf(ys, n);

  while (touchReadRaw(rx, ry)) delay(5);   // wait for release
  delay(150);                              // debounce
  return true;
}

// ---------------------------------------------------------------- stage 1

struct CalPoint { int screenX, screenY; int rawX, rawY; const char *label; };

// Four points, and NOT two-on-a-diagonal. SKILL.md: "if your two calibration
// points lie on the screen diagonal, the maths is degenerate -- a swapped-axis
// mapping and an unswapped one both fit the data, and you cannot tell them
// apart. This wasted a lot of time. Use at least one off-diagonal point."
static CalPoint calPoints[] = {
  {  40,  50, 0, 0, "TOP LEFT" },
  { 200,  50, 0, 0, "TOP RIGHT" },
  {  40, 246, 0, 0, "BOTTOM LEFT" },
  { 200, 246, 0, 0, "BOTTOM RIGHT" },
};
static constexpr int CAL_COUNT = sizeof(calPoints) / sizeof(calPoints[0]);

static void drawCrosshair(int x, int y, uint16_t c) {
  lcdFillRect(x - 14, y - 1, 29, 3, c);
  lcdFillRect(x - 1, y - 14, 3, 29, c);
  lcdFillRect(x - 4, y - 4, 9, 9, c);
}

// A tap is only trustworthy if it agrees with the other tap that shares one of
// its screen coordinates: the two left points must report nearly the same
// rawY, the two top points nearly the same rawX, and so on. Without this
// check, a single sloppy corner tap silently skews the whole fit and the error
// only shows up later as a keyboard that types the wrong letters.
static constexpr int PAIR_TOLERANCE = 35;

static bool tapOnePoint(int i) {
  lcdFillScreen(COLOR_BLACK);
  lcdDrawTextCentered(8, "CALIBRATE", COLOR_CYAN, COLOR_BLACK, 2);
  char line[24];
  snprintf(line, sizeof(line), "TAP %d OF %d", i + 1, CAL_COUNT);
  lcdDrawTextCentered(34, line, COLOR_WHITE, COLOR_BLACK, 1);
  lcdDrawTextCentered(150, calPoints[i].label, COLOR_AMBER, COLOR_BLACK, 1);
  lcdDrawTextCentered(272, "TAP THE CROSS CENTRE", COLOR_GRAY, COLOR_BLACK, 1);
  drawCrosshair(calPoints[i].screenX, calPoints[i].screenY, COLOR_GREEN);

  int rx = 0, ry = 0;
  if (!captureTap(rx, ry)) return false;
  calPoints[i].rawX = rx;
  calPoints[i].rawY = ry;

  Serial.printf("cal[%d] %-13s screen=(%3d,%3d) raw=(%4d,%4d)\n",
                i, calPoints[i].label, calPoints[i].screenX, calPoints[i].screenY, rx, ry);

  drawCrosshair(calPoints[i].screenX, calPoints[i].screenY, COLOR_GRAY);
  delay(150);
  return true;
}

static void runCalibration() {
  for (int attempt = 1; attempt <= 4; ++attempt) {
    for (int i = 0; i < CAL_COUNT; ++i) {
      while (!tapOnePoint(i)) { /* timed out or too brief, ask again */ }
    }

    // Indices: 0=TL 1=TR 2=BL 3=BR
    const int dTopRawX    = abs(calPoints[0].rawX - calPoints[1].rawX);  // share screenY
    const int dBottomRawX = abs(calPoints[2].rawX - calPoints[3].rawX);
    const int dLeftRawY   = abs(calPoints[0].rawY - calPoints[2].rawY);  // share screenX
    const int dRightRawY  = abs(calPoints[1].rawY - calPoints[3].rawY);

    Serial.printf("consistency: topRawX=%d bottomRawX=%d leftRawY=%d rightRawY=%d (tol %d)\n",
                  dTopRawX, dBottomRawX, dLeftRawY, dRightRawY, PAIR_TOLERANCE);

    const bool ok = dTopRawX <= PAIR_TOLERANCE && dBottomRawX <= PAIR_TOLERANCE &&
                    dLeftRawY <= PAIR_TOLERANCE && dRightRawY <= PAIR_TOLERANCE;
    if (ok) break;

    Serial.println("!! taps disagree with each other -- redoing all four");
    lcdFillScreen(COLOR_BLACK);
    lcdDrawTextCentered(100, "TAPS DIDNT", COLOR_AMBER, COLOR_BLACK, 2);
    lcdDrawTextCentered(126, "LINE UP", COLOR_AMBER, COLOR_BLACK, 2);
    lcdDrawTextCentered(170, "TRY AGAIN, AIM AT", COLOR_WHITE, COLOR_BLACK, 1);
    lcdDrawTextCentered(188, "THE CROSS CENTRE", COLOR_WHITE, COLOR_BLACK, 1);
    delay(2600);

    if (attempt == 4) {
      Serial.println("!! still inconsistent after 4 rounds, using the last set anyway");
    }
  }

  // Per-axis linear fit. Left/right pairs isolate the X relationship, top/
  // bottom pairs isolate Y, which is what makes the axis-swap unambiguous.
  const int rawY_left   = (calPoints[0].rawY + calPoints[2].rawY) / 2;
  const int rawY_right  = (calPoints[1].rawY + calPoints[3].rawY) / 2;
  const int rawX_top    = (calPoints[0].rawX + calPoints[1].rawX) / 2;
  const int rawX_bottom = (calPoints[2].rawX + calPoints[3].rawX) / 2;

  const int screenX_left  = calPoints[0].screenX;
  const int screenX_right = calPoints[1].screenX;
  const int screenY_top    = calPoints[0].screenY;
  const int screenY_bottom = calPoints[2].screenY;

  Serial.println();
  Serial.println("=== measured relationships ===");
  Serial.printf("rawY: left(x=%d)=%d  right(x=%d)=%d   -> span %d over %d screen px\n",
                screenX_left, rawY_left, screenX_right, rawY_right,
                rawY_right - rawY_left, screenX_right - screenX_left);
  Serial.printf("rawX: top(y=%d)=%d  bottom(y=%d)=%d   -> span %d over %d screen px\n",
                screenY_top, rawX_top, screenY_bottom, rawX_bottom,
                rawX_bottom - rawX_top, screenY_bottom - screenY_top);

  const int dRawY = rawY_right - rawY_left;
  const int dRawX = rawX_bottom - rawX_top;

  if (abs(dRawY) < 20 || abs(dRawX) < 20) {
    Serial.println("!! one axis barely moved -- axes may not be swapped the way this");
    Serial.println("!! sketch assumes, or the panel is not reporting properly.");
    lcdFillScreen(COLOR_BLACK);
    lcdDrawTextCentered(120, "CAL FAILED", COLOR_RED, COLOR_BLACK, 2);
    lcdDrawTextCentered(150, "SEE SERIAL LOG", COLOR_WHITE, COLOR_BLACK, 1);
    delay(4000);
    return;
  }

  mapping.yOff = rawY_left;
  mapping.xNum = screenX_right - screenX_left;
  mapping.xDen = dRawY;
  mapping.xAdd = screenX_left;

  // rawX runs opposite to screenY on this panel, so the fit is expressed
  // against (yInv - rawX) to keep the divisor positive.
  mapping.yInv = rawX_bottom;
  mapping.yNum = screenY_bottom - screenY_top;
  mapping.yDen = rawX_top - rawX_bottom;
  mapping.yAdd = screenY_bottom;
  mapping.calibrated = true;

  Serial.println();
  Serial.println("=== paste this into the firmware ===");
  Serial.printf("  x = constrain((rawY - %d) * %d / %d + %d, 0, LCD_WIDTH  - 1);\n",
                mapping.yOff, mapping.xNum, mapping.xDen, mapping.xAdd);
  Serial.printf("  y = constrain((%d - rawX) * %d / %d + %d, 0, LCD_HEIGHT - 1);\n",
                mapping.yInv, mapping.yNum, mapping.yDen, mapping.yAdd);
  Serial.println();
  Serial.println("SKILL.md reference unit, for comparison:");
  Serial.println("  x = constrain((rawY -  12) * 180 / 211 + 30, 0, LCD_WIDTH  - 1);");
  Serial.println("  y = constrain((260 - rawX) * 216 / 222 + 40, 0, LCD_HEIGHT - 1);");
  Serial.println();
}

// ---------------------------------------------------------------- stage 2

// 3 cols x 4 rows of keyboard-sized targets. A 10-key QWERTY row on a 240px
// panel gives 24px keys; these are 72px, so if taps miss at THIS size the
// mapping is not good enough for a keyboard and needs rethinking.
static constexpr int GRID_COLS = 3;
static constexpr int GRID_ROWS = 4;
static constexpr int GRID_TOP  = 60;
static constexpr int CELL_W    = LCD_WIDTH / GRID_COLS;
static constexpr int CELL_H    = 44;

static const char *cellLabel(int r, int c) {
  static const char *labels[GRID_ROWS][GRID_COLS] = {
    { "Q", "W", "E" },
    { "A", "S", "D" },
    { "Z", "X", "C" },
    { "1", "2", "3" },
  };
  return labels[r][c];
}

static void drawGrid(int hitR, int hitC) {
  lcdFillScreen(COLOR_BLACK);
  lcdDrawTextCentered(8, "VERIFY", COLOR_CYAN, COLOR_BLACK, 2);
  lcdDrawTextCentered(34, mapping.calibrated ? "USING NEW CAL" : "USING SKILL DEFAULTS",
                      mapping.calibrated ? COLOR_GREEN : COLOR_AMBER, COLOR_BLACK, 1);

  for (int r = 0; r < GRID_ROWS; ++r) {
    for (int c = 0; c < GRID_COLS; ++c) {
      const int x = c * CELL_W, y = GRID_TOP + r * CELL_H;
      const bool hit = (r == hitR && c == hitC);
      lcdFillRect(x + 2, y + 2, CELL_W - 4, CELL_H - 4, hit ? COLOR_GREEN : COLOR_GRAY);
      lcdDrawText(x + CELL_W / 2 - 6, y + CELL_H / 2 - 7, cellLabel(r, c),
                  hit ? COLOR_BLACK : COLOR_WHITE, hit ? COLOR_GREEN : COLOR_GRAY, 2);
    }
  }
  lcdDrawTextCentered(255, "TAP KEYS TO TEST", COLOR_WHITE, COLOR_BLACK, 1);
  lcdDrawTextCentered(272, "HOLD ANY KEY 2S TO RECAL", COLOR_AMBER, COLOR_BLACK, 1);
}

static void runVerify() {
  drawGrid(-1, -1);

  for (;;) {
    int rx, ry;
    if (!touchReadRaw(rx, ry)) { delay(8); continue; }

    uint32_t heldSince = millis();
    int lastRx = rx, lastRy = ry;
    while (touchReadRaw(rx, ry)) {
      lastRx = rx; lastRy = ry;
      if (millis() - heldSince > 2000) {   // long press -> recalibrate
        while (touchReadRaw(rx, ry)) delay(5);
        runCalibration();
        drawGrid(-1, -1);
        break;
      }
      delay(8);
    }
    if (millis() - heldSince > 2000) continue;

    int sx, sy;
    touchToScreen(lastRx, lastRy, sx, sy);

    int r = (sy - GRID_TOP) / CELL_H;
    int c = sx / CELL_W;
    const bool inGrid = (r >= 0 && r < GRID_ROWS && c >= 0 && c < GRID_COLS && sy >= GRID_TOP);

    Serial.printf("raw=(%4d,%4d) -> screen=(%3d,%3d) -> %s\n", lastRx, lastRy, sx, sy,
                  inGrid ? cellLabel(r, c) : "(outside grid)");

    if (inGrid) { drawGrid(r, c); delay(220); drawGrid(-1, -1); }
    delay(80);
  }
}

// ---------------------------------------------------------------- setup

void setup() {
  Serial.begin(115200);

  // Gotcha 8: GPIO 2 HIGH cuts power to the board. Pin it LOW first thing.
  pinMode(PIN_POWER_OFF, OUTPUT);
  digitalWrite(PIN_POWER_OFF, LOW);

  SPI.begin(PIN_LCD_SCLK, -1, PIN_LCD_MOSI, PIN_LCD_CS);  // no MISO on this bus
  SPI.setFrequency(40000000);
  lcdInit();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  pinMode(PIN_TOUCH_IRQ, INPUT);  // polling is fine; the IRQ line is optional

  delay(400);
  Serial.println();
  Serial.println("=== Cheeko Gotchi touch calibration ===");

  // Prove the panel is even on the bus before asking anyone to tap at it.
  Wire.beginTransmission(CST810_ADDR);
  const bool touchPresent = (Wire.endTransmission() == 0);
  Serial.printf("CST810 at 0x15: %s\n", touchPresent ? "present" : "NOT FOUND");

  if (!touchPresent) {
    lcdFillScreen(COLOR_BLACK);
    lcdDrawTextCentered(120, "NO TOUCH", COLOR_RED, COLOR_BLACK, 2);
    lcdDrawTextCentered(150, "CST810 NOT ON I2C", COLOR_WHITE, COLOR_BLACK, 1);
    while (true) delay(1000);
  }

  lcdFillScreen(COLOR_BLACK);
  lcdDrawTextCentered(90,  "TOUCH TEST", COLOR_CYAN, COLOR_BLACK, 2);
  lcdDrawTextCentered(130, "TAP 4 CROSSHAIRS", COLOR_WHITE, COLOR_BLACK, 1);
  lcdDrawTextCentered(148, "AS ACCURATELY AS", COLOR_WHITE, COLOR_BLACK, 1);
  lcdDrawTextCentered(166, "YOU CAN", COLOR_WHITE, COLOR_BLACK, 1);
  lcdDrawTextCentered(210, "TAP TO BEGIN", COLOR_AMBER, COLOR_BLACK, 1);

  int rx, ry;
  captureTap(rx, ry, 120000);

  runCalibration();
  runVerify();
}

void loop() { /* runVerify never returns */ }
