// LED Tile v2 demo and bring-up tool for the LedTile library.
//
// Shows the library API (setPixel / show / brightness / gamma) and keeps the
// diagnostic test patterns used to reverse-engineer the panel. All panel
// knowledge lives in lib/LedTile; this file only draws and handles a serial UI.

#include <Arduino.h>

#include <LedTile.h>

// Overridable from build_flags, e.g. -DBCM_BITS=7 -DBCM_BASE_CYCLES=4.
#ifndef BCM_BITS
#define BCM_BITS 8
#endif
#ifndef BCM_BASE_CYCLES
#define BCM_BASE_CYCLES 4
#endif
#ifndef FLOOR_CYCLES
#define FLOOR_CYCLES 2
#endif

static LedTile tile;

static const int WIDTH = LedTile::kWidth;
static const int HEIGHT = LedTile::kHeight;
static const int GROUPS = 8;

enum Colour { COL_B = 0, COL_G = 1, COL_R = 2 };
enum Bank { BANK_TOP = 0, BANK_BOTTOM = 1 };

// ------------------------------------------------------------- test patterns

enum TestMode {
  TEST_CYCLE_RULER,      // raw DMA cycles, to measure the cycle -> column ratio
  TEST_GEOMETRY,         // border, diagonal and corner markers
  TEST_BRIGHTNESS_RAMP,  // BCM dimming: a left-to-right brightness gradient
  TEST_BREATHE,          // BCM dimming: one block fading continuously over time
  TEST_ALL_ON,           // everything white
  TEST_ADDR_ID,          // one distinct colour per address and bank
  TEST_GROUP_MARK,       // bit 0 of all 8 groups at once
  TEST_LOW_BYTE,         // bits 0-7 of all 8 groups at once
  TEST_WORD_WALK,        // one 16-bit chip word at a time
  TEST_BIT_WALK,         // one single LED at a time
  TEST_MODE_COUNT
};

static TestMode mode = TEST_GEOMETRY;
RTC_DATA_ATTR static uint32_t bootCount = 0;  // survives soft reset: shows reboot loops
static uint32_t modeStarted = 0;
static const uint32_t MODE_MS = 6000;

static void px(int x, int y, bool r, bool g, bool b) {
  tile.setPixel(x, y, r ? 255 : 0, g ? 255 : 0, b ? 255 : 0);
}

// Light a whole address/bank band in one colour, bypassing the framebuffer.
static void fillAddr(uint8_t addr, uint8_t bank, bool b, bool g, bool r) {
  for (uint8_t grp = 0; grp < GROUPS; ++grp) {
    tile.rawSetWord(addr, bank, grp, COL_B, b ? 0xFFFF : 0);
    tile.rawSetWord(addr, bank, grp, COL_G, g ? 0xFFFF : 0);
    tile.rawSetWord(addr, bank, grp, COL_R, r ? 0xFFFF : 0);
  }
}

// Drives D1 and D2 high for the first (step + 1) groups of 48 cycles on
// address 0: the leftmost 8 columns of rows 0, 4, 8 and 12 at step 0, 16 at
// step 1, and so on to the full width at step 7.
static void patternCycleRuler(uint32_t step) {
  tile.rawFillCycles(0, ((int)(step % GROUPS) + 1) * 48);
}

// Deliberately asymmetric, so any mirroring, row swap or word swap is obvious.
static void patternGeometry() {
  for (int x = 0; x < WIDTH; ++x) {
    px(x, 0, true, true, true);
    px(x, HEIGHT - 1, true, true, true);
  }
  for (int y = 0; y < HEIGHT; ++y) {
    px(0, y, true, true, true);
    px(WIDTH - 1, y, true, true, true);
  }
  for (int i = 1; i < HEIGHT - 1; ++i) px(i, i, true, false, false);  // red diagonal
  for (int y = 2; y < 5; ++y)                                         // green 3x3
    for (int x = 20; x < 23; ++x) px(x, y, false, true, false);
  px(WIDTH - 2, HEIGHT - 2, false, false, true);  // blue pixel, bottom right
}

// Left = dim, right = full, in four row bands: red, green, blue, white.
static void patternBrightnessRamp() {
  for (int x = 0; x < WIDTH; ++x) {
    const uint8_t level = (uint8_t)(x * 255 / (WIDTH - 1));
    for (int y = 0; y < HEIGHT; ++y) {
      const int band = y / 4;
      tile.setPixel(x, y, (band == 0 || band == 3) ? level : 0,
                    (band == 1 || band == 3) ? level : 0,
                    (band == 2 || band == 3) ? level : 0);
    }
  }
}

// A block fading on a 3 second triangle wave, so the dimming is visibly live.
static void patternBreathe() {
  const uint32_t t = millis() % 3000;
  const uint32_t half = (t < 1500) ? t : (3000 - t);
  const uint8_t level = (uint8_t)(half * 255 / 1500);
  for (int y = 4; y < 12; ++y)
    for (int x = 16; x < 48; ++x) tile.setPixel(x, y, level, level, level);
}

static void patternAllOn() {
  for (uint8_t a = 0; a < 4; ++a) {
    fillAddr(a, BANK_TOP, true, true, true);
    fillAddr(a, BANK_BOTTOM, true, true, true);
  }
}

static void patternAddrId() {
  //                        B      G      R
  fillAddr(0, BANK_TOP,     false, false, true);   // red
  fillAddr(1, BANK_TOP,     false, true,  false);  // green
  fillAddr(2, BANK_TOP,     true,  false, false);  // blue
  fillAddr(3, BANK_TOP,     false, true,  true);   // yellow
  fillAddr(0, BANK_BOTTOM,  true,  false, true);   // magenta
  fillAddr(1, BANK_BOTTOM,  true,  true,  false);  // cyan
  fillAddr(2, BANK_BOTTOM,  true,  true,  true);   // white
  fillAddr(3, BANK_BOTTOM,  false, false, true);   // red, isolated from the top one
}

static void patternGroupMark() {
  for (uint8_t g = 0; g < GROUPS; ++g) tile.rawSetWord(0, BANK_TOP, g, COL_R, 0x0001);
}

static void patternLowByte() {
  for (uint8_t g = 0; g < GROUPS; ++g) tile.rawSetWord(0, BANK_TOP, g, COL_R, 0x00FF);
}

static void patternWordWalk(uint32_t step) {
  const uint8_t idx = step % (GROUPS * 3);
  tile.rawSetWord(0, BANK_TOP, idx / 3, idx % 3, 0xFFFF);
}

static void patternBitWalk(uint32_t step) {
  tile.rawSetWord(0, BANK_TOP, 0, COL_R, 1u << (step % 16));
}

static const char *modeName(TestMode m) {
  switch (m) {
    case TEST_CYCLE_RULER:     return "CYCLE_RULER- raw DMA cycles, measures the ratio";
    case TEST_GEOMETRY:        return "GEOMETRY   - border, diagonal, corner markers";
    case TEST_BRIGHTNESS_RAMP: return "BRIGHT_RAMP- BCM dimming: left-right gradient";
    case TEST_BREATHE:         return "BREATHE    - BCM dimming: live fading block";
    case TEST_ALL_ON:          return "ALL_ON     - every LED white";
    case TEST_ADDR_ID:         return "ADDR_ID    - 8 bands, all different colours";
    case TEST_GROUP_MARK:      return "GROUP_MARK - bit 0 of all 8 groups at once";
    case TEST_LOW_BYTE:        return "LOW_BYTE   - bits 0-7 of all 8 groups at once";
    case TEST_WORD_WALK:       return "WORD_WALK  - one 16-bit chip word at a time";
    case TEST_BIT_WALK:        return "BIT_WALK   - one single LED at a time";
    default:                   return "?";
  }
}

static void reportStep(uint32_t step) {
  if (mode == TEST_CYCLE_RULER) {
    const unsigned groups = (unsigned)(step % GROUPS) + 1;
    Serial.printf("  %u cycle blocks -> expect %u columns lit\n", groups, groups * 8);
  } else if (mode == TEST_WORD_WALK) {
    const uint8_t idx = step % (GROUPS * 3);
    Serial.printf("  word %2u  (group %u, %c)\n", idx, idx / 3, "BGR"[idx % 3]);
  } else if (mode == TEST_BIT_WALK) {
    Serial.printf("  bit %2u\n", (unsigned)(step % 16));
  }
}

static bool isRawMode(TestMode m) {
  return m == TEST_CYCLE_RULER || m == TEST_ALL_ON || m == TEST_ADDR_ID ||
         m == TEST_GROUP_MARK || m == TEST_LOW_BYTE || m == TEST_WORD_WALK ||
         m == TEST_BIT_WALK;
}

static void drawCurrent(uint32_t step) {
  // Raw patterns write below the framebuffer, so they (and the first frame
  // after any mode change) need a hard wipe. Plain redraws, such as BREATHE's
  // 30 ms updates, use the soft clear() so the panel never goes blank mid-frame.
  static TestMode drawnMode = TEST_MODE_COUNT;
  if (isRawMode(mode) || mode != drawnMode) tile.rawClear();
  else tile.clear();
  drawnMode = mode;
  switch (mode) {
    case TEST_CYCLE_RULER:     patternCycleRuler(step); break;
    case TEST_GEOMETRY:        patternGeometry();       break;
    case TEST_BRIGHTNESS_RAMP: patternBrightnessRamp(); break;
    case TEST_BREATHE:         patternBreathe();        break;
    case TEST_ALL_ON:          patternAllOn();          break;
    case TEST_ADDR_ID:         patternAddrId();         break;
    case TEST_GROUP_MARK:      patternGroupMark();      break;
    case TEST_LOW_BYTE:        patternLowByte();        break;
    case TEST_WORD_WALK:       patternWordWalk(step);   break;
    case TEST_BIT_WALK:        patternBitWalk(step);    break;
    default: break;
  }
  tile.show();
}

// ------------------------------------------------------------------- sketch

static void printHelp() {
  Serial.println("keys: n/./, = mode and step, a = auto, c = cycle ruler");
  Serial.println("      + / - = panel clock, r = redraw, d = DMA / bit-bang transport");
  Serial.println("      g = gamma on/off, [ / ] = gamma strength");
  Serial.println("      < / > = brightness");
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("LED Tile v2 demo (LedTile library)");
  Serial.printf("boot #%u\n", (unsigned)++bootCount);

  LedTileConfig cfg;  // default pins match the ESP-WROVER-KIT wiring in the README
  cfg.bcmBits = BCM_BITS;
  cfg.baseCycles = BCM_BASE_CYCLES;
  cfg.floorCycles = FLOOR_CYCLES;
  cfg.log = &Serial;
  if (!tile.begin(cfg)) {
    Serial.printf("FATAL: %s\n", tile.lastError());
    while (true) delay(1000);
  }

  printHelp();
  drawCurrent(0);
  modeStarted = millis();
  Serial.printf("mode: %s\n", modeName(mode));
}

void loop() {
  static uint32_t step = 0;
  static uint32_t lastStep = 0;
  static bool manual = true;  // boot held on GEOMETRY
  bool redraw = false;

  while (Serial.available()) {
    const int c = Serial.read();
    if (c == 'n' || c == 'c') {
      mode = (c == 'c') ? TEST_CYCLE_RULER : (TestMode)((mode + 1) % TEST_MODE_COUNT);
      step = 0;
      manual = true;
      redraw = true;
      Serial.printf("mode: %s\n", modeName(mode));
      reportStep(step);
    } else if (c == '.' || c == ',') {
      if (c == '.') ++step; else if (step) --step;
      manual = true;
      redraw = true;
      reportStep(step);
    } else if (c == 'a') {
      manual = false;
      modeStarted = millis();
      Serial.println("auto-advance resumed");
    } else if (c == '+' || c == '-') {
      const uint32_t hz = tile.clockHz();
      tile.setClockHz((c == '+') ? min<uint32_t>(hz * 2, 4000000) : max<uint32_t>(hz / 2, 250000));
      Serial.printf("panel clock %u Hz, refresh %u Hz\n", (unsigned)tile.clockHz(),
                    (unsigned)tile.refreshHz());
    } else if (c == 'd') {
      const bool toBitBang = tile.transport() == LedTile::TRANSPORT_DMA;
      tile.setTransport(toBitBang ? LedTile::TRANSPORT_BITBANG : LedTile::TRANSPORT_DMA);
      Serial.println(toBitBang ? "transport: bit-bang (same waveform, CPU driven)"
                               : "transport: I2S DMA");
    } else if (c == 'g') {
      tile.setGamma(!tile.gammaEnabled());
      redraw = true;
      Serial.printf("gamma: %s (strength %.0f%%)\n", tile.gammaEnabled() ? "ON" : "OFF",
                    (double)(tile.gammaStrength() * 100.0f));
    } else if (c == '[' || c == ']') {
      tile.setGammaStrength(tile.gammaStrength() + ((c == ']') ? 0.1f : -0.1f));
      redraw = true;
      Serial.printf("gamma strength: %.0f%% (0%%=linear, 100%%=full CIE1931)\n",
                    (double)(tile.gammaStrength() * 100.0f));
    } else if (c == '<' || c == '>') {
      const int b = tile.brightness() + ((c == '>') ? 16 : -16);
      tile.setBrightness((uint8_t)constrain(b, 0, 255));
      redraw = true;
      Serial.printf("brightness: %u\n", (unsigned)tile.brightness());
    } else if (c == 'r') {
      redraw = true;
    } else if (c == '?') {
      printHelp();
    }
  }

  // Heartbeat: millis() climbing means the CPU is alive, so a frozen image is a
  // DMA problem; restarting from zero means the sketch is rebooting.
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 2000) {
    lastBeat = millis();
    if (tile.transport() == LedTile::TRANSPORT_DMA) {
      const uint32_t hz = tile.refreshHz();
      const bool frameOk = tile.waitForFrame(hz ? 2000000UL / hz + 1000 : 1000);
      I2SParallelStatus st;
      tile.dmaStatus(&st);
      Serial.printf("alive t=%us mode=%d frame=%s refresh=%uHz tx=%u err=%u dscr=%08x\n",
                    (unsigned)(millis() / 1000), (int)mode, frameOk ? "OK" : "TIMEOUT",
                    (unsigned)hz, (unsigned)st.txStarted, (unsigned)st.descriptorError,
                    (unsigned)st.linkDscr);
    } else {
      Serial.printf("alive t=%us mode=%d (bit-bang)\n", (unsigned)(millis() / 1000), (int)mode);
    }
  }

  if (!manual) {
    if (millis() - modeStarted > MODE_MS) {
      mode = (TestMode)((mode + 1) % TEST_MODE_COUNT);
      modeStarted = millis();
      step = 0;
      redraw = true;
      Serial.printf("mode: %s\n", modeName(mode));
    }
    if (millis() - lastStep > 700) {  // slow enough to follow the walkers by eye
      lastStep = millis();
      ++step;
      redraw = true;
      reportStep(step);
    }
  }

  // BREATHE animates from millis(), so keep redrawing it while it is on screen.
  static uint32_t lastBreathe = 0;
  if (mode == TEST_BREATHE && millis() - lastBreathe > 30) {
    lastBreathe = millis();
    redraw = true;
  }

  if (redraw) drawCurrent(step);

  if (tile.transport() == LedTile::TRANSPORT_BITBANG) {
    tile.service();
  } else {
    delay(5);  // DMA plays on its own
  }
}
