#include <Arduino.h>
#include <string.h>

// Pin definitions -- see "ESP32 pin mapping" in README.md.
//
// Board: ESP-WROVER-KIT. The onboard ILI9341 LCD must be unplugged, as it shares
// GPIO 5/18/19/21/22/23/25 with the panel. GPIO25 is the LCD's MISO line and will
// contend with CLK if the display is left attached.
//
// Do not use: 6-11 (SPI flash), 16-17 (PSRAM, bonded inside the WROVER module),
// 34-39 (input only, no output driver), 1/3 (UART0 console), 0/2/5/12/15 (strapping).
#define PIN_D1 26   // Serial data, top bank (rows 0-7)
#define PIN_D2 18   // Serial data, bottom bank (rows 8-15)
#define PIN_LAT 19  // Shared latch
#define PIN_OE 21   // Output enable, active LOW
#define PIN_A0 22   // Row-pair select, bit 0
#define PIN_A1 23   // Row-pair select, bit 1
#define PIN_CLK 25  // Shared shift clock

// Panel geometry.
//
// 48 x MBI5034, 16 bits each = 768 outputs. The panel has 64 x 16 x 3 = 3072 LEDs,
// and 3072 / 768 = 4, which is why there are exactly 4 address states. Those 768
// outputs are split across the two banks, so each data line takes 384 bits (48
// bytes) per address. 384 bits / 3 colours = 128 pixels = two rows of 64.
//
// Layout within one 384-bit stream: 8 groups of [B:16, G:16, R:16].
#define PANEL_WIDTH 64
#define PANEL_HEIGHT 16
#define GROUPS_PER_STREAM 8
#define STREAM_BYTES 48

// Row mapping, CONFIRMED on hardware via TEST_ADDR_ID.
//
// Each address drives two rows within a bank, and those two rows are NOT
// adjacent -- they are four apart:
//
//   address 0 -> rows 0 and 4      address 2 -> rows 2 and 6
//   address 1 -> rows 1 and 5      address 3 -> rows 3 and 7
//
//   bank      = row / 8          (0 = top, via D1;  1 = bottom, via D2)
//   rowInBank = row % 8
//   address   = rowInBank % 4
//
// A single 16-bit word lights LEDs in BOTH of an address's two rows at once, so
// the two rows are interleaved within each word rather than occupying separate
// halves of the stream.
//
// CONFIRMED via TEST_GROUP_MARK and TEST_LOW_BYTE: the split is by BYTE, not by
// parity. The low byte of each word (bits 0-7) feeds the HIGHER of the address's
// two rows, and the high byte (bits 8-15) feeds the lower row.
#define ROWS_PER_ADDR 2
#define ADDR_ROW_STRIDE 4

#define BANK_TOP 0
#define BANK_BOTTOM 1
#define COL_B 0
#define COL_G 1
#define COL_R 2

// Shift order, CONFIRMED on hardware via TEST_BIT_WALK: the lit LED sweeps cleanly
// right to left as the bit index rises. A wrong byte order would instead show two
// reversed runs of 8, so MSB-first is correct.
//
// Bit 0 lands at the RIGHT edge and higher bits run leftwards, so a framebuffer
// has to mirror x when it packs pixels into words.
#define SHIFT_MSB_FIRST 1
#define PIXEL0_AT_RIGHT 1

// [address][bank][bytes]
static uint8_t fb[4][2][STREAM_BYTES];

// ---------------------------------------------------------------- low level

static void setAddress(uint8_t addr) {
  digitalWrite(PIN_A0, addr & 0x01);
  digitalWrite(PIN_A1, (addr >> 1) & 0x01);
}

static void latchData() {
  digitalWrite(PIN_LAT, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_LAT, LOW);
}

static inline void clockPulse() {
  digitalWrite(PIN_CLK, HIGH);
  digitalWrite(PIN_CLK, LOW);
}

// Send 384 bits (48 bytes) to D1 and D2 in parallel.
static void sendStream(const uint8_t *dataD1, const uint8_t *dataD2) {
  for (int i = 0; i < STREAM_BYTES; ++i) {
#if SHIFT_MSB_FIRST
    for (int b = 7; b >= 0; --b) {
#else
    for (int b = 0; b < 8; ++b) {
#endif
      digitalWrite(PIN_D1, (dataD1[i] >> b) & 0x01);
      digitalWrite(PIN_D2, (dataD2[i] >> b) & 0x01);
      clockPulse();
    }
  }
}

// One full pass over all four address states.
static void refresh() {
  for (uint8_t addr = 0; addr < 4; ++addr) {
    digitalWrite(PIN_OE, HIGH);
    sendStream(fb[addr][BANK_TOP], fb[addr][BANK_BOTTOM]);
    setAddress(addr);
    latchData();
    digitalWrite(PIN_OE, LOW);
    delayMicroseconds(500);
    digitalWrite(PIN_OE, HIGH);
  }
}

// ------------------------------------------------------------ frame buffer

// Write one 16-bit chip word: group 0-7, colour COL_B / COL_G / COL_R.
static void setWord(uint8_t addr, uint8_t bank, uint8_t group, uint8_t colour,
                    uint16_t bits) {
  uint8_t *p = &fb[addr][bank][group * 6 + colour * 2];
  p[0] = bits >> 8;
  p[1] = bits & 0xFF;
}

static void clearAll() { memset(fb, 0, sizeof(fb)); }

static void fillAddr(uint8_t addr, uint8_t bank, bool b, bool g, bool r) {
  for (uint8_t grp = 0; grp < GROUPS_PER_STREAM; ++grp) {
    setWord(addr, bank, grp, COL_B, b ? 0xFFFF : 0x0000);
    setWord(addr, bank, grp, COL_G, g ? 0xFFFF : 0x0000);
    setWord(addr, bank, grp, COL_R, r ? 0xFFFF : 0x0000);
  }
}

// Set or clear one LED. Every step of this mapping is confirmed on hardware:
//
//   bank      = y / 8           D1 drives rows 0-7, D2 drives rows 8-15
//   rowInBank = y % 8
//   address   = rowInBank % 4   each address serves rowInBank and rowInBank + 4
//   byte      low byte -> the higher row, high byte -> the lower row
//   group     = x / 8           group 0 is leftmost: it ships first, and the
//                               shift chain is fed from the right-hand end
//   bit       = 7 - (x % 8)     higher bits sit further left
//
// If a drawn image ever comes out mirrored, the group ordering is the assumption
// to revisit -- it is inferred from WORD_WALK starting at the left edge.
static void setPixel(uint8_t x, uint8_t y, bool r, bool g, bool b) {
  if (x >= PANEL_WIDTH || y >= PANEL_HEIGHT) return;

  uint8_t bank = y / 8;
  uint8_t rowInBank = y % 8;
  uint8_t addr = rowInBank % ADDR_ROW_STRIDE;
  bool higherRow = rowInBank >= ADDR_ROW_STRIDE;

  uint8_t group = x / 8;
  uint8_t bit = 7 - (x % 8);
  if (!higherRow) bit += 8;

  // setWord packs the high byte at p[0] and the low byte at p[1].
  uint8_t *word = &fb[addr][bank][group * 6];
  uint8_t idx = (bit >= 8) ? 0 : 1;
  uint8_t mask = 1u << (bit & 7);
  const bool on[3] = {b, g, r};  // COL_B, COL_G, COL_R
  for (uint8_t c = 0; c < 3; ++c) {
    if (on[c]) {
      word[c * 2 + idx] |= mask;
    } else {
      word[c * 2 + idx] &= ~mask;
    }
  }
}

// ------------------------------------------------------------- test patterns

enum TestMode {
  TEST_GEOMETRY,    // border, diagonal and corner markers
  TEST_ALL_ON,      // everything white
  TEST_ADDR_ID,     // one distinct colour per address and bank
  TEST_GROUP_MARK,  // bit 0 of all 8 groups at once
  TEST_LOW_BYTE,    // bits 0-7 of all 8 groups at once
  TEST_WORD_WALK,   // one 16-bit chip word at a time
  TEST_BIT_WALK,    // one single LED at a time
  TEST_MODE_COUNT
};

static TestMode mode = TEST_GEOMETRY;
static uint32_t modeStarted = 0;
static const uint32_t MODE_MS = 6000;

// A deliberately asymmetric picture. If setPixel is right this reads correctly;
// any mirroring, row swap or group reversal is obvious at a glance.
static void patternGeometry() {
  for (uint8_t x = 0; x < PANEL_WIDTH; ++x) {
    setPixel(x, 0, true, true, true);
    setPixel(x, PANEL_HEIGHT - 1, true, true, true);
  }
  for (uint8_t y = 0; y < PANEL_HEIGHT; ++y) {
    setPixel(0, y, true, true, true);
    setPixel(PANEL_WIDTH - 1, y, true, true, true);
  }
  // Red diagonal, running down and to the right from inside the top left corner.
  for (uint8_t i = 1; i < PANEL_HEIGHT - 1; ++i) {
    setPixel(i, i, true, false, false);
  }
  // Green 3x3 block left of centre, so "up" and "left" are unambiguous.
  for (uint8_t y = 2; y < 5; ++y) {
    for (uint8_t x = 20; x < 23; ++x) {
      setPixel(x, y, false, true, false);
    }
  }
  // A single blue pixel just inside the bottom right corner.
  setPixel(PANEL_WIDTH - 2, PANEL_HEIGHT - 2, false, false, true);
}

// Every LED of every colour, both banks, all four addresses.
static void patternAllOn() {
  for (uint8_t addr = 0; addr < 4; ++addr) {
    fillAddr(addr, BANK_TOP, true, true, true);
    fillAddr(addr, BANK_BOTTOM, true, true, true);
  }
}

// Eight bands, every one a different colour, so no two address states or banks
// can be confused with one another.
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

// Bit 0 of all 8 groups, in red, on address 0 of the top bank. Eight dots appear
// at once, so the layout is readable in a single glance: their spacing gives the
// group stride, and how they divide between rows 0 and 4 settles the interleave.
static void patternGroupMark() {
  for (uint8_t grp = 0; grp < GROUPS_PER_STREAM; ++grp) {
    setWord(0, BANK_TOP, grp, COL_R, 0x0001);
  }
}

// All 8 groups, low byte only (bits 0-7), red. Bit 0 is known to land on row 4,
// so this separates the two remaining candidate layouts in a single glance:
//
//   byte split     (bits 0-7 -> row 4, bits 8-15 -> row 0)
//                    -> row 4 lit end to end, row 0 completely dark
//
//   bit interleave (even bits -> row 4, odd bits -> row 0)
//                    -> both rows half lit, as a comb of alternating pixels
static void patternLowByte() {
  for (uint8_t grp = 0; grp < GROUPS_PER_STREAM; ++grp) {
    setWord(0, BANK_TOP, grp, COL_R, 0x00FF);
  }
}

// Light exactly one 16-bit word. Where each lit block lands reveals the
// group -> pixel mapping.
static void patternWordWalk(uint32_t step) {
  uint8_t idx = step % (GROUPS_PER_STREAM * 3);  // 24 words per stream
  setWord(0, BANK_TOP, idx / 3, idx % 3, 0xFFFF);
}

// Light exactly one LED. Confirmed: sweeps right to left as the bit index rises.
static void patternBitWalk(uint32_t step) {
  setWord(0, BANK_TOP, 0, COL_R, 1u << (step % 16));
}

static const char *modeName(TestMode m) {
  switch (m) {
    case TEST_GEOMETRY:   return "GEOMETRY   - border, diagonal, corner markers";
    case TEST_ALL_ON:     return "ALL_ON     - every LED white";
    case TEST_ADDR_ID:    return "ADDR_ID    - 8 bands, all different colours";
    case TEST_GROUP_MARK: return "GROUP_MARK - bit 0 of all 8 groups at once";
    case TEST_LOW_BYTE:   return "LOW_BYTE   - bits 0-7 of all 8 groups at once";
    case TEST_WORD_WALK:  return "WORD_WALK  - one 16-bit chip word at a time";
    case TEST_BIT_WALK:   return "BIT_WALK   - one single LED at a time";
    default:              return "?";
  }
}

static void reportStep(uint32_t step) {
  if (mode == TEST_WORD_WALK) {
    uint8_t idx = step % (GROUPS_PER_STREAM * 3);
    Serial.printf("  word %2u  (group %u, %c)\n", idx, idx / 3, "BGR"[idx % 3]);
  } else if (mode == TEST_BIT_WALK) {
    Serial.printf("  bit %2u\n", (unsigned)(step % 16));
  }
}

// ------------------------------------------------------------------- sketch

void setup() {
  pinMode(PIN_D1, OUTPUT);
  pinMode(PIN_D2, OUTPUT);
  pinMode(PIN_LAT, OUTPUT);
  pinMode(PIN_OE, OUTPUT);
  pinMode(PIN_A0, OUTPUT);
  pinMode(PIN_A1, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  digitalWrite(PIN_OE, HIGH);  // outputs off until we have data
  digitalWrite(PIN_LAT, LOW);
  digitalWrite(PIN_CLK, LOW);

  Serial.begin(115200);
  Serial.println();
  Serial.println("LED Tile v2 driver - bring-up test patterns");
  Serial.printf("Shift order: %s\n", SHIFT_MSB_FIRST ? "MSB first" : "LSB first");
  Serial.println("keys: n = next mode, . = next step, , = prev step, a = auto");

  clearAll();
  patternGeometry();
  modeStarted = millis();
  Serial.printf("mode: %s\n", modeName(mode));
}

void loop() {
  static uint32_t step = 0;
  static uint32_t lastStep = 0;
  static bool manual = false;

  // Serial control, so a pattern can be held still and read off carefully.
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'n') {
      mode = (TestMode)((mode + 1) % TEST_MODE_COUNT);
      step = 0;
      manual = true;
      Serial.printf("mode: %s\n", modeName(mode));
      reportStep(step);
    } else if (c == '.') {
      ++step;
      manual = true;
      reportStep(step);
    } else if (c == ',') {
      if (step) --step;
      manual = true;
      reportStep(step);
    } else if (c == 'a') {
      manual = false;
      modeStarted = millis();
      Serial.println("auto-advance resumed");
    } else if (c == '?') {
      Serial.println("n = next mode, . = next step, , = prev step, a = auto");
    }
  }

  if (!manual) {
    if (millis() - modeStarted > MODE_MS) {
      mode = (TestMode)((mode + 1) % TEST_MODE_COUNT);
      modeStarted = millis();
      step = 0;
      Serial.printf("mode: %s\n", modeName(mode));
    }
    // The walking patterns advance slowly enough to follow by eye.
    if (millis() - lastStep > 700) {
      lastStep = millis();
      ++step;
      reportStep(step);
    }
  }

  clearAll();
  switch (mode) {
    case TEST_GEOMETRY:   patternGeometry();     break;
    case TEST_ALL_ON:     patternAllOn();        break;
    case TEST_ADDR_ID:    patternAddrId();       break;
    case TEST_GROUP_MARK: patternGroupMark();    break;
    case TEST_LOW_BYTE:   patternLowByte();      break;
    case TEST_WORD_WALK:  patternWordWalk(step); break;
    case TEST_BIT_WALK:   patternBitWalk(step);  break;
    default: break;
  }

  refresh();
}
