// I2S configuration sweep for the ES7210 mic on the Cheeko Gotchi board.
//
// Why: capture through the "proven" reference config delivers real speech --
// the amplitude envelope and a 120 Hz voice fundamental are clearly present --
// but only 4 int16 out of every 32 carry data. The rest are exact zeros:
//
//     ..........XXXX............................XXXX..................
//
// That is a framing mismatch, not a noise or gain problem. The ESP32 (I2S
// master) and the ES7210 disagree about frame length, so we sample 1 in 8 of
// the stream, which decimates the audio ~4x with no anti-alias filter. The
// envelope and pitch survive; the consonants do not, so whisper hears humming.
//
// Rather than reflash-and-talk for every candidate config, this sweeps them
// automatically. Misalignment shows up as periodic exact zeros whether or not
// anyone is speaking, so no human input is needed -- ambient noise is enough.
//
// A correct config should show ~100% non-zero samples and no repeating zero
// period.

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <esp_idf_version.h>

static constexpr int PIN_POWER_OFF   = 2;
static constexpr int PIN_PA_CTRL     = 4;
static constexpr int PIN_I2S_MCLK    = 5;
static constexpr int PIN_I2S_DOUT    = 6;
static constexpr int PIN_I2S_DIN     = 7;
static constexpr int PIN_I2C_SCL     = 11;
static constexpr int PIN_I2C_SDA     = 12;
static constexpr int PIN_I2S_BCLK    = 15;
static constexpr int PIN_I2S_LRCK    = 16;

static constexpr uint8_t ES8311_ADDR = 0x18;
static constexpr uint8_t ES7210_ADDR = 0x40;

static bool i2cWriteReg(uint8_t a, uint8_t r, uint8_t v) {
  Wire.beginTransmission(a); Wire.write(r); Wire.write(v);
  return Wire.endTransmission() == 0;
}

// Unchanged from the reference sketch: this is the sequence that demonstrably
// produces real (if mis-framed) speech, so it is held constant while the I2S
// side is varied.
static bool es7210Init() {
  bool ok = true;
  const uint8_t seq[][2] = {
    {0x00,0xff},{0x00,0x41},{0x01,0x3f},{0x09,0x30},{0x0a,0x30},{0x23,0x2a},
    {0x22,0x0a},{0x20,0x0a},{0x21,0x2a},{0x08,0x00},{0x40,0x43},{0x41,0x70},
    {0x42,0x70},{0x07,0x20},{0x02,0xc1},{0x11,0x60},{0x12,0x00},{0x01,0x34},
    {0x06,0x00},{0x47,0x08},{0x48,0x08},{0x4b,0x00},{0x4c,0xff},{0x43,0x1a},
    {0x44,0x1a},{0x40,0x43},{0x00,0x71},{0x00,0x41},
  };
  for (auto &kv : seq) ok &= i2cWriteReg(ES7210_ADDR, kv[0], kv[1]);
  return ok;
}

static bool es8311Init() {
  bool ok = true;
  const uint8_t seq[][2] = {
    {0x00,0x1f},{0x00,0x00},{0x00,0x80},{0x01,0x3f},{0x02,0x00},{0x03,0x10},
    {0x04,0x10},{0x05,0x00},{0x06,0x03},{0x07,0x00},{0x08,0xff},{0x09,0x0c},
    {0x0a,0x0c},{0x0d,0x01},{0x0e,0x02},{0x12,0x00},{0x13,0x10},{0x1c,0x6a},
    {0x31,0x00},{0x32,0xbf},{0x37,0x08},
  };
  for (auto &kv : seq) ok &= i2cWriteReg(ES8311_ADDR, kv[0], kv[1]);
  return ok;
}

struct Candidate {
  const char *label;
  i2s_bits_per_sample_t bits;
  i2s_channel_fmt_t chan;
  i2s_comm_format_t comm;
  uint32_t rate;
  uint32_t mclk;        // 0 = let the driver pick
  bool rx_only;
};

// The axes worth varying, given the symptom is frame length rather than gain:
// slot width, how many slots the driver expects, frame justification, and the
// MCLK the codec is clocked against.
static Candidate candidates[] = {
  { "ref 16b 2ch I2S mclk256",   I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_FMT_RIGHT_LEFT, I2S_COMM_FORMAT_STAND_I2S, 16000, 4096000, false },
  { "16b 2ch I2S mclk auto",     I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_FMT_RIGHT_LEFT, I2S_COMM_FORMAT_STAND_I2S, 16000, 0,       false },
  { "16b 2ch I2S rx-only",       I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_FMT_RIGHT_LEFT, I2S_COMM_FORMAT_STAND_I2S, 16000, 4096000, true  },
  { "16b 2ch MSB(left-just)",    I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_FMT_RIGHT_LEFT, I2S_COMM_FORMAT_STAND_MSB, 16000, 4096000, false },
  { "32b 2ch I2S mclk256",       I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_FMT_RIGHT_LEFT, I2S_COMM_FORMAT_STAND_I2S, 16000, 4096000, false },
  { "32b 2ch I2S mclk auto",     I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_FMT_RIGHT_LEFT, I2S_COMM_FORMAT_STAND_I2S, 16000, 0,       false },
  { "32b 2ch MSB(left-just)",    I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_FMT_RIGHT_LEFT, I2S_COMM_FORMAT_STAND_MSB, 16000, 4096000, false },
  { "16b ONLY_LEFT",             I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_FMT_ONLY_LEFT,  I2S_COMM_FORMAT_STAND_I2S, 16000, 4096000, false },
  { "16b ONLY_RIGHT",            I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_FMT_ONLY_RIGHT, I2S_COMM_FORMAT_STAND_I2S, 16000, 4096000, false },
  { "32b ONLY_LEFT",             I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_FMT_ONLY_LEFT,  I2S_COMM_FORMAT_STAND_I2S, 16000, 4096000, false },
  { "16b 2ch I2S 8kHz",          I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_FMT_RIGHT_LEFT, I2S_COMM_FORMAT_STAND_I2S,  8000, 2048000, false },
  { "16b 2ch I2S 24kHz",         I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_FMT_RIGHT_LEFT, I2S_COMM_FORMAT_STAND_I2S, 24000, 6144000, false },
  { "16b 2ch I2S 48kHz",         I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_FMT_RIGHT_LEFT, I2S_COMM_FORMAT_STAND_I2S, 48000, 12288000,false },
};
static constexpr int CAND_COUNT = sizeof(candidates) / sizeof(candidates[0]);

static int16_t buf[4096];

// Smallest p such that the zero/non-zero mask repeats with period p. A correct
// config has no such period (returns 0) because nothing is systematically zero.
static int detectZeroPeriod(const int16_t *s, int n) {
  for (int p = 2; p <= 64; ++p) {
    bool consistent = true;
    for (int off = 0; off < p && consistent; ++off) {
      int zero = 0, total = 0;
      for (int i = off; i < n; i += p) { if (s[i] == 0) ++zero; ++total; }
      const double frac = (double)zero / total;
      // Every position must be almost-always-zero or almost-never-zero for
      // this to be a real framing period rather than coincidence.
      if (frac > 0.05 && frac < 0.95) consistent = false;
    }
    if (consistent) {
      int zeroSlots = 0;
      for (int off = 0; off < p; ++off) {
        int zero = 0, total = 0;
        for (int i = off; i < n; i += p) { if (s[i] == 0) ++zero; ++total; }
        if ((double)zero / total > 0.95) ++zeroSlots;
      }
      if (zeroSlots > 0 && zeroSlots < p) return p;
    }
  }
  return 0;
}

static void tryCandidate(int candIndex) {
  const Candidate &c = candidates[candIndex];
  i2s_driver_uninstall(I2S_NUM_0);
  delay(40);

  i2s_config_t cfg = {};
  cfg.mode = c.rx_only ? (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX)
                       : (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  cfg.sample_rate          = c.rate;
  cfg.bits_per_sample      = c.bits;
  cfg.channel_format       = c.chan;
  cfg.communication_format = c.comm;
  cfg.intr_alloc_flags     = 0;
  cfg.dma_buf_count        = 8;
  cfg.dma_buf_len          = 256;
  cfg.use_apll             = false;
  cfg.tx_desc_auto_clear   = true;
  if (c.mclk) cfg.fixed_mclk = c.mclk;

  i2s_pin_config_t pins = {};
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
  pins.mck_io_num = PIN_I2S_MCLK;
#endif
  pins.bck_io_num   = PIN_I2S_BCLK;
  pins.ws_io_num    = PIN_I2S_LRCK;
  pins.data_out_num = c.rx_only ? I2S_PIN_NO_CHANGE : PIN_I2S_DOUT;
  pins.data_in_num  = PIN_I2S_DIN;

  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr) != ESP_OK) {
    Serial.printf("  %-26s INSTALL FAILED\n", c.label);
    return;
  }
  if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) {
    Serial.printf("  %-26s SET_PIN FAILED\n", c.label);
    i2s_driver_uninstall(I2S_NUM_0);
    return;
  }
  i2s_zero_dma_buffer(I2S_NUM_0);

  // Codecs must be re-initialised after the clocks change under them.
  if (!c.rx_only) es8311Init();
  es7210Init();
  delay(120);

  // Discard the first reads: the codec needs a moment after reconfiguration.
  size_t br = 0;
  for (int i = 0; i < 4; ++i) i2s_read(I2S_NUM_0, buf, sizeof(buf), &br, pdMS_TO_TICKS(200));

  if (i2s_read(I2S_NUM_0, buf, sizeof(buf), &br, pdMS_TO_TICKS(400)) != ESP_OK || br == 0) {
    Serial.printf("  %-26s READ FAILED\n", c.label);
    i2s_driver_uninstall(I2S_NUM_0);
    return;
  }

  const int n = br / sizeof(int16_t);
  int nz = 0; long sumAbs = 0; int peak = 0;
  for (int i = 0; i < n; ++i) {
    const int v = abs((int)buf[i]);
    if (buf[i] != 0) ++nz;
    sumAbs += v;
    if (v > peak) peak = v;
  }
  const int period = detectZeroPeriod(buf, n);

  Serial.printf("  %-26s nonzero %5.1f%%  mean|v| %6ld  peak %6d  zeroPeriod %d%s\n",
                c.label, 100.0 * nz / n, sumAbs / n, peak, period,
                (period == 0 && nz > n * 0.9) ? "   <== CANDIDATE" : "");

  i2s_driver_uninstall(I2S_NUM_0);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_POWER_OFF, OUTPUT);
  digitalWrite(PIN_POWER_OFF, LOW);     // Gotcha 8

  pinMode(PIN_PA_CTRL, OUTPUT);
  digitalWrite(PIN_PA_CTRL, HIGH);      // as the reference leaves it

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  delay(400);

  Serial.println();
  Serial.println("=== ES7210 I2S framing sweep ===");
  Serial.println("Looking for a config with ~100% non-zero samples and no zero period.");
  Serial.println("The reference config is expected to show zeroPeriod 32 with 4 live slots.");
  Serial.println("Make some ambient noise; exact speech is not required.");
  Serial.println();
}

void loop() {
  Serial.println("---- sweep pass ----");
  for (int i = 0; i < CAND_COUNT; ++i) tryCandidate(i);
  Serial.println();
  delay(4000);
}
