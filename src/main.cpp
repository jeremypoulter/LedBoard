#include <Arduino.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <rom/gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>
#include <string.h>

#include "i2s_parallel.h"

// Pin definitions -- see "ESP32 pin mapping" in README.md.
//
// Board: ESP-WROVER-KIT. The onboard ILI9341 LCD must be unplugged, as it shares
// GPIO 5/18/19/21/22/23/25 with the panel. GPIO25 is the LCD's MISO line and will
// contend with CLK if the display is left attached.
//
// Do not use: 6-11 (SPI flash), 16-17 (PSRAM, bonded inside the WROVER module),
// 34-39 (input only, no output driver), 1/3 (UART0 console), 0/2/5/12/15 (strapping).
//
// These are unchanged from the bit-banged driver: the ESP32 routes I2S through
// the GPIO matrix, so the same wiring works.
#define PIN_D1 26   // Serial data, top bank (rows 0-7)
#define PIN_D2 18   // Serial data, bottom bank (rows 8-15)
#define PIN_LAT 19  // Shared latch
#define PIN_OE 21   // Output enable, active LOW
#define PIN_A0 22   // Row-pair select, bit 0
#define PIN_A1 23   // Row-pair select, bit 1
#define PIN_CLK 25  // Shared shift clock

// Every control signal rides on the I2S bus alongside the data, so latch and
// blanking are timed by the DMA stream rather than by CPU writes. That is the
// whole point of the exercise: timing stops depending on interrupt latency.
#define BUS_D1 (1u << 0)
#define BUS_D2 (1u << 1)
#define BUS_LAT (1u << 2)
#define BUS_OE (1u << 3)  // active LOW at the panel, so a SET bit means blanked
#define BUS_A0 (1u << 4)
#define BUS_A1 (1u << 5)
#define BUS_CLK (1u << 6)
#define BUS_WIDTH 7

// Start slow. 10 MHz over ribbon cable at 3.3V into a 5V-powered panel is
// optimistic, and corrupted shifting looks like a horizontally squashed image.
// Raise it with '+' at runtime to find where this particular rig gives up.
#define I2S_CLOCK_DEFAULT_HZ 2000000UL

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
#define ADDR_COUNT 4

// Row mapping, CONFIRMED on hardware via TEST_ADDR_ID, TEST_GROUP_MARK and
// TEST_LOW_BYTE, then validated end to end by TEST_GEOMETRY.
//
// Each address drives two rows within a bank, four apart:
//
//   address 0 -> rows 0 and 4      address 2 -> rows 2 and 6
//   address 1 -> rows 1 and 5      address 3 -> rows 3 and 7
//
// A single 16-bit word lights LEDs in BOTH of those rows, split by BYTE: the low
// byte (bits 0-7) feeds the higher row, the high byte (bits 8-15) the lower one.
#define ROWS_PER_ADDR 2
#define ADDR_ROW_STRIDE 4

#define BANK_TOP 0
#define BANK_BOTTOM 1
#define COL_B 0
#define COL_G 1
#define COL_R 2

// ---------------------------------------------------------------- BCM layout
//
// The panel is strictly 1 bit per channel per LED -- there is no analogue
// brightness control, as the README's "Coding" section established. Dimming
// therefore means Binary Code Modulation: instead of shifting+latching+
// displaying ONE frame per address, each address is split into BCM_BITS
// sub-passes ("bit planes"), each shifting+latching its own 384-bit stream and
// then displaying it for a duration proportional to its binary weight
// (1, 2, 4, 8... cycles). A pixel's brightness is encoded by which planes its
// bit is set in: lit in every plane gives full brightness; lit only in the
// most significant plane gives roughly half; unlit in all gives off.
//
// BCM_BITS=4 (16 levels per channel) was the conservative starting point, kept
// until TEST_BRIGHTNESS_RAMP and TEST_BREATHE were confirmed working. Raised
// to 5 (32 levels) once real hardware testing found the ACTUAL problem gamma
// correction cannot fix: at 16 levels, consecutive HIGH levels (e.g. 14 vs 15)
// differ by only ~7% in physical brightness, confirmed as "the brighter end
// does not look to change". Gamma can only decide which of the existing
// levels an input maps to -- it cannot invent extra levels where none exist,
// so the only real fix is more of them. 32 levels halves that top-end gap to
// ~3.3%. See the table in README.md for memory/refresh at each value before
// raising further.
#define BCM_BITS 5
// Display length of the least-significant bit-plane, in shift-clock cycles.
//
// This controls overall brightness, not just the dimming resolution, and it is
// easy to get badly wrong: the 384+6 cycle shift+latch overhead repeats for
// EVERY plane (ADDR_BLOCK_CYCLES includes BCM_BITS copies of it), but the
// panel is only actually lit during the display windows. At BCM_BITS=4 and a
// too-small base of 2, the display windows summed to only 30 cycles against
// 1560 of overhead -- 1.9% duty even at full white, technically correct
// dimming but far too dim to look right (confirmed on hardware).
//
// Halved from 32 to 16 alongside the BCM_BITS=4->5 bump: raising BCM_BITS
// alone at the old base would have pushed the DMA buffer to ~235KB, a real
// risk of not finding one contiguous block of the ESP32's internal
// DMA-capable RAM. Halving the base keeps the buffer at ~200KB -- close to
// the ~171KB already confirmed working -- while still gaining the extra bit
// of resolution, at a small, acceptable cost in max brightness (19.7% -> 17.5%
// duty). Raise this independently of BCM_BITS for more brightness at the cost
// of memory (linear in BCM_BASE_CYCLES); see the table in README.md.
#define BCM_BASE_CYCLES 16
static_assert(BCM_BITS >= 1 && BCM_BITS <= 8, "BCM_BITS must fit an 8-bit channel");
static_assert(BCM_BASE_CYCLES >= 1, "a plane needs at least one display cycle");

// A dedicated extra plane, dimmer than BCM_BASE_CYCLES, used ONLY for values
// that the gamma floor (below) would otherwise have to round all the way up
// to the main ladder's smallest level. Confirmed on hardware: without this,
// the floor made "off" jump straight to the full brightness of level 1, with
// no finer step in between -- a real gap, not just the inherent perceptual
// jump of going from true darkness to any light at all.
//
// This is its own full shift+latch+display pass (one more plane means one
// more 390-cycle shift pass, same as any other plane), so it is not free: see
// FLOOR_CYCLES' contribution to ADDR_BLOCK_CYCLES below and the updated
// table in README.md. Kept at a quarter of BCM_BASE_CYCLES, scaled down
// alongside it when BCM_BASE_CYCLES halved from 32 to 16.
#define FLOOR_CYCLES 4
#define TOTAL_PLANES (BCM_BITS + 1)
static_assert(FLOOR_CYCLES >= 1 && FLOOR_CYCLES < BCM_BASE_CYCLES,
              "the floor plane must be dimmer than the smallest main level");

// Display length of plane p, in shift-clock cycles: the main ladder is pure
// powers of two (BCM_BASE_CYCLES << p) for p = 0..BCM_BITS-1; the extra floor
// plane at index BCM_BITS is not part of that progression.
static constexpr int planeWeight(uint8_t p) {
  return (p < BCM_BITS) ? (BCM_BASE_CYCLES << p) : FLOOR_CYCLES;
}

// ---------------------------------------------------------------- DMA layout
//
// Each shift cycle expands to CLK low, low, high, high; non-shift cycles keep CLK low.
#define SHIFT_CYCLES 384
// Keep CLK low before, during and after LAT; the pulse lasts 1 us at the default rate.
#define LATCH_CYCLES 6
#define SAMPLES_PER_CYCLE 4

// Sum of 2^0 .. 2^(BCM_BITS-1), i.e. the total display cycles of the main
// ladder alone (not counting the floor plane) if every plane fired consecutively.
#define BCM_WEIGHT_SUM ((1u << BCM_BITS) - 1)
// One address now costs TOTAL_PLANES complete shift+latch passes (the main
// ladder plus the floor plane), plus the sum of every plane's weighted
// display window.
#define ADDR_BLOCK_CYCLES                                        \
  (TOTAL_PLANES * (SHIFT_CYCLES + LATCH_CYCLES) +                \
   BCM_BASE_CYCLES * BCM_WEIGHT_SUM + FLOOR_CYCLES)
#define DMA_WORDS (ADDR_COUNT * ADDR_BLOCK_CYCLES)
#define WAVEFORM_SAMPLES (DMA_WORDS * SAMPLES_PER_CYCLE)

static uint16_t *dma = NULL;  // [addr][plane][cycle], flattened
static uint32_t *dmaWaveform = NULL;

// Offset, in cycles, of each plane's block within one address's span. Computed
// once at startup since planeWeight() is cheap but not worth repeating on
// every cell() call.
static int planeStart[TOTAL_PLANES];

static void computePlaneLayout() {
  int offset = 0;
  for (int p = 0; p < TOTAL_PLANES; ++p) {
    planeStart[p] = offset;
    offset += SHIFT_CYCLES + LATCH_CYCLES + planeWeight(p);
  }
  if (offset != ADDR_BLOCK_CYCLES) {
    Serial.printf("FATAL: plane layout mismatch (%d != %d)\n", offset,
                  (int)ADDR_BLOCK_CYCLES);
    while (true) {
      delay(1000);
    }
  }
}

static inline uint16_t *cell(uint8_t addr, uint8_t plane, int cycleIdx) {
  return &dma[addr * ADDR_BLOCK_CYCLES + planeStart[plane] + cycleIdx];
}

// Write the control signals for every cycle of every plane. Called once; the
// data bits are layered on top afterwards and never disturb these.
static void buildControl() {
  for (uint8_t a = 0; a < ADDR_COUNT; ++a) {
    for (uint8_t p = 0; p < TOTAL_PLANES; ++p) {
      const int displayCycles = planeWeight(p);
      const int blockCycles = SHIFT_CYCLES + LATCH_CYCLES + displayCycles;
      // Address lines only actually move at the first plane of each address;
      // every later plane of the same address is not a real address
      // transition, so it just holds steady at `a` throughout, matching how
      // the original bit-bang driver only touched A0/A1 once per address.
      const uint8_t shiftAddr = (p == 0) ? (a + ADDR_COUNT - 1) % ADDR_COUNT : a;
      for (int cc = 0; cc < blockCycles; ++cc) {
        const uint8_t address = (cc < SHIFT_CYCLES) ? shiftAddr : a;
        uint16_t addrBits = 0;
        if (address & 0x01) addrBits |= BUS_A0;
        if (address & 0x02) addrBits |= BUS_A1;
        uint16_t v = addrBits;
        // Blanked for the whole shift + latch window, plus one extra cycle at
        // the very end of the block for margin at the sub-block boundary.
        if (cc < SHIFT_CYCLES + LATCH_CYCLES || cc == blockCycles - 1) {
          v |= BUS_OE;
        }
        if (cc == SHIFT_CYCLES + 1 || cc == SHIFT_CYCLES + 2) {
          v |= BUS_LAT;
        }
        *cell(a, p, cc) = v;
      }
    }
  }
}

static constexpr uint32_t duplicateSample(uint16_t value) {
  return (uint32_t)value | ((uint32_t)value << 16);
}

static_assert(duplicateSample(BUS_LAT | BUS_OE) == 0x000c000c,
              "Latch and blanking must survive halfword swapping");
static_assert(duplicateSample(BUS_CLK) == 0x00400040,
              "Clock phases must survive halfword swapping");

static void buildDmaWaveform() {
  for (int i = 0; i < DMA_WORDS; ++i) {
    const uint16_t low = dma[i];
    // Every plane's own SHIFT_CYCLES span starts at that plane's planeStart[p],
    // so "is this cycle inside some plane's shift window" is just: its offset
    // from the start of whichever plane it falls in must be < SHIFT_CYCLES.
    // Since planes are laid out back to back, (i % ADDR_BLOCK_CYCLES) modulo
    // each plane's own block length gives that offset; simplest to recompute
    // it the same way buildControl did, rather than re-deriving which plane.
    const int cycleInAddr = i % ADDR_BLOCK_CYCLES;
    bool inShift = false;
    for (uint8_t p = 0; p < TOTAL_PLANES; ++p) {
      if (cycleInAddr >= planeStart[p] && cycleInAddr < planeStart[p] + SHIFT_CYCLES) {
        inShift = true;
        break;
      }
    }
    const uint16_t high = low | (inShift ? BUS_CLK : 0);
    // Identical halfwords make I2S sample-pair ordering irrelevant.
    dmaWaveform[2 * i] = duplicateSample(low);
    dmaWaveform[2 * i + 1] = duplicateSample(high);
  }
}

static bool validateDmaWaveform() {
  uint16_t previous = BUS_OE;
  for (uint8_t a = 0; a < ADDR_COUNT; ++a) {
    for (uint8_t p = 0; p < TOTAL_PLANES; ++p) {
      const int displayCycles = planeWeight(p);
      const int blockCycles = SHIFT_CYCLES + LATCH_CYCLES + displayCycles;
      unsigned shifted = 0;
      unsigned latches = 0;
      for (int localCycle = 0; localCycle < blockCycles; ++localCycle) {
        const int globalCycle = a * ADDR_BLOCK_CYCLES + planeStart[p] + localCycle;
        for (int s = 0; s < SAMPLES_PER_CYCLE; ++s) {
          const int index = globalCycle * SAMPLES_PER_CYCLE + s;
          const uint32_t pair = dmaWaveform[index / 2];
          if ((uint16_t)pair != (uint16_t)(pair >> 16)) return false;
          const uint16_t value = (uint16_t)pair;
          if ((value & BUS_CLK) && !(previous & BUS_CLK)) {
            if (!(value & BUS_OE) || (value & BUS_LAT) || shifted >= SHIFT_CYCLES) return false;
            const uint16_t expected = *cell(a, p, shifted++) & (BUS_D1 | BUS_D2);
            if ((value & (BUS_D1 | BUS_D2)) != expected) return false;
            if ((previous ^ value) & (BUS_D1 | BUS_D2)) return false;
          }
          if (value & BUS_LAT) {
            if ((value & BUS_CLK) || !(value & BUS_OE)) return false;
            if (!(previous & BUS_LAT)) {
              if (previous & BUS_CLK) return false;
              if (shifted != SHIFT_CYCLES) return false;
              ++latches;
            }
          }
          if (!(value & BUS_OE) || (value & BUS_LAT)) {
            const unsigned address = ((value & BUS_A0) ? 1 : 0) | ((value & BUS_A1) ? 2 : 0);
            if (address != a || latches != 1 || (value & BUS_CLK)) return false;
          }
          previous = value;
        }
      }
      if (shifted != SHIFT_CYCLES || latches != 1) return false;
    }
  }
  return true;
}

// ------------------------------------------------------------ frame buffer

// Set one bit of the shifted stream, for one plane. Byte i of the 48-byte
// stream goes out MSB first, so stream byte i bit b occupies cycle
// i * 8 + (7 - b), identically within every plane's own shift window.
static inline void setStreamBit(uint8_t addr, uint8_t plane, uint8_t bank,
                                uint8_t byteIdx, uint8_t bitInByte, bool on) {
  uint16_t mask = (bank == BANK_TOP) ? BUS_D1 : BUS_D2;
  uint16_t *c = cell(addr, plane, byteIdx * 8 + (7 - bitInByte));
  if (on) {
    *c |= mask;
  } else {
    *c &= (uint16_t)~mask;
  }
}

// Write one 16-bit chip word: group 0-7, colour COL_B / COL_G / COL_R. The high
// byte is sent first, matching how the bit-banged driver packed p[0] and p[1].
//
// This is the diagnostic/raw primitive used by the geometry test patterns
// below: it sets the SAME bit identically across every plane, including the
// floor plane, which is exactly "full brightness" or "off" and so reproduces
// their pre-BCM behaviour unchanged.
static void setWord(uint8_t addr, uint8_t bank, uint8_t group, uint8_t colour,
                    uint16_t bits) {
  for (uint8_t b = 0; b < 16; ++b) {
    uint8_t byteIdx = group * 6 + colour * 2 + ((b >= 8) ? 0 : 1);
    bool on = (bits >> b) & 0x01;
    for (uint8_t p = 0; p < TOTAL_PLANES; ++p) {
      setStreamBit(addr, p, bank, byteIdx, b & 7, on);
    }
  }
}

static void clearAll() {
  for (uint8_t a = 0; a < ADDR_COUNT; ++a) {
    for (uint8_t p = 0; p < TOTAL_PLANES; ++p) {
      for (int c = 0; c < SHIFT_CYCLES; ++c) {
        *cell(a, p, c) &= (uint16_t)~(BUS_D1 | BUS_D2);
      }
    }
  }
}

// Shared geometry math for both setPixel (below) and setPixelBrightness.
// Every step of this mapping is confirmed on hardware:
//
//   bank      = y / 8           D1 drives rows 0-7, D2 drives rows 8-15
//   rowInBank = y % 8
//   address   = rowInBank % 4   each address serves rowInBank and rowInBank + 4
//   byte      low byte -> the higher row, high byte -> the lower row
//   group     = x / 8           group 0 is leftmost
//   bit       = 7 - (x % 8)     higher bits sit further left
struct PixelSlot {
  uint8_t addr, bank, group, halfIdx, bitInByte;
};

static inline bool pixelSlot(uint8_t x, uint8_t y, PixelSlot *out) {
  if (x >= PANEL_WIDTH || y >= PANEL_HEIGHT) {
    return false;
  }
  uint8_t bank = y / 8;
  uint8_t rowInBank = y % 8;
  uint8_t addr = rowInBank % ADDR_ROW_STRIDE;
  bool higherRow = rowInBank >= ADDR_ROW_STRIDE;

  uint8_t group = x / 8;
  uint8_t bit = 7 - (x % 8);
  if (!higherRow) {
    bit += 8;
  }

  out->addr = addr;
  out->bank = bank;
  out->group = group;
  out->halfIdx = (bit >= 8) ? 0 : 1;  // 0 = high byte, sent first
  out->bitInByte = bit & 7;
  return true;
}

// Set or clear one LED at full brightness (every plane identical), for the
// diagnostic test patterns that only ever need solid on/off.
static void setPixel(uint8_t x, uint8_t y, bool r, bool g, bool b) {
  PixelSlot s;
  if (!pixelSlot(x, y, &s)) {
    return;
  }
  const bool on[3] = {b, g, r};  // COL_B, COL_G, COL_R
  for (uint8_t c = 0; c < 3; ++c) {
    for (uint8_t p = 0; p < TOTAL_PLANES; ++p) {
      setStreamBit(s.addr, p, s.bank, s.group * 6 + c * 2 + s.halfIdx, s.bitInByte, on[c]);
    }
  }
}

// ------------------------------------------------------------ gamma correction
//
// Human brightness perception is roughly a power law, not linear, so a LINEAR
// duty cycle looks badly uneven: the very first non-zero BCM level already
// reads as surprisingly bright (confirmed on hardware -- "big jump from off to
// the lowest setting"), while steps at the high end become hard to tell apart.
//
// A naive power-law gamma (first tried here at exponent 2.8, the common
// LED-strip default) makes this WORSE with only BCM_BITS=4 output levels: it
// is tuned for 8-bit-plus output, where compressing the low end still leaves
// many representable codes there. Confirmed on hardware and by direct
// calculation: at 2.8, fully 37% of the 0-255 input range maps to output level
// 0 -- not dim, literally off -- which is what showed up as "almost half the
// screen is now black". The CIE 1931 perceptual lightness formula below has a
// LINEAR segment near black rather than a curve that keeps compressing all the
// way to zero slope, so it loses less of the input range to the bottom code
// in the first place (calculated: 30% instead of 37% -- better, but, on its
// own, still a real dead zone).
//
// What actually closes that dead zone is the dedicated floor plane defined
// above (FLOOR_CYCLES / TOTAL_PLANES): any value gamma still rounds down to
// output level 0 lights that plane instead of nothing, which is a genuinely
// dimmer step than the main ladder's level 1, not the same brightness forced
// up to meet it (confirmed on hardware as the difference between "a bit dim"
// and "jumps straight to level 1 with no step in between").
//
// Full-strength CIE1931 then turned out to overcorrect: with only ~17 output
// levels total, the curve's low-end compression crams so many input values
// onto the same few dim codes that a visible, unchanging plateau appears
// before the ramp starts climbing (confirmed on hardware as "the lower end is
// scaling very slowly"; calculated at full strength, the widest single-level
// plateau on a 64-wide ramp is 19 pixels -- nearly a third of it). Rather than
// pick one fixed compromise, gammaStrength blends linearly between the raw
// input (0.0) and the full CIE1931 curve (1.0), and is adjustable live with
// '[' / ']' rather than needing another edit-rebuild-reflash cycle to tune by
// eye. 0.5 is the new default (calculated widest plateau: 7 pixels).
//
// Toggle the correction on/off entirely with 'g', independent of strength.
static uint8_t gammaLUT[256];
static bool gammaEnabled = true;
static float gammaStrength = 0.5f;

static void buildGammaLUT() {
  for (int i = 0; i < 256; ++i) {
    // Treat i/255 as perceptual lightness L* on the standard 0-100 scale, and
    // convert to relative luminance Y (0-1) with the CIE 1931 formula.
    float Lstar = (i / 255.0f) * 100.0f;
    float Y;
    if (Lstar <= 8.0f) {
      Y = Lstar / 903.3f;
    } else {
      float t = (Lstar + 16.0f) / 116.0f;
      Y = t * t * t;
    }
    float corrected = Y * 255.0f;
    float blended = i + (corrected - i) * gammaStrength;
    if (blended < 0.0f) blended = 0.0f;
    if (blended > 255.0f) blended = 255.0f;
    gammaLUT[i] = (uint8_t)(blended + 0.5f);
  }
}

static inline uint8_t applyGamma(uint8_t value) {
  return gammaEnabled ? gammaLUT[value] : value;
}

// Real per-pixel dimming. r/g/b are 0-255; only the top BCM_BITS bits of each
// are used for the main ladder, since that is all the hardware has bit planes
// for -- e.g. at BCM_BITS=4, values 0x00-0x0F and 0x10-0x1F both render
// identically on the main ladder. Gamma correction (when enabled) is applied
// here, before that reduction, so it reshapes which 0-255 input maps to which
// of the BCM_BITS output levels rather than just scaling an already-quantised
// value.
//
// Values gamma pushes below the main ladder's smallest level light ONLY the
// dedicated floor plane instead -- a genuinely finer, dimmer rung between true
// off and the main ladder's level 1, rather than a jump straight to it. Every
// level from 1 upward is completely unaffected: the floor plane is off
// whenever the main ladder has anything lit.
static void setPixelBrightness(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
  PixelSlot s;
  if (!pixelSlot(x, y, &s)) {
    return;
  }
  const uint8_t in[3] = {b, g, r};  // COL_B, COL_G, COL_R
  uint8_t mainLevel[3];
  bool useFloor[3];
  for (uint8_t c = 0; c < 3; ++c) {
    uint8_t level = applyGamma(in[c]) >> (8 - BCM_BITS);
    useFloor[c] = (in[c] != 0 && level == 0);
    mainLevel[c] = level;
  }
  for (uint8_t c = 0; c < 3; ++c) {
    for (uint8_t p = 0; p < BCM_BITS; ++p) {
      bool on = (mainLevel[c] >> p) & 0x01;
      setStreamBit(s.addr, p, s.bank, s.group * 6 + c * 2 + s.halfIdx, s.bitInByte, on);
    }
    setStreamBit(s.addr, BCM_BITS, s.bank, s.group * 6 + c * 2 + s.halfIdx, s.bitInByte,
                useFloor[c]);
  }
}

static void fillAddr(uint8_t addr, uint8_t bank, bool b, bool g, bool r) {
  for (uint8_t grp = 0; grp < GROUPS_PER_STREAM; ++grp) {
    setWord(addr, bank, grp, COL_B, b ? 0xFFFF : 0x0000);
    setWord(addr, bank, grp, COL_G, g ? 0xFFFF : 0x0000);
    setWord(addr, bank, grp, COL_R, r ? 0xFFFF : 0x0000);
  }
}

// ------------------------------------------------------------- test patterns

enum TestMode {
  TEST_CYCLE_RULER, // raw DMA cycles, to measure the cycle -> column ratio
  TEST_GEOMETRY,    // border, diagonal and corner markers
  TEST_BRIGHTNESS_RAMP, // real BCM dimming: a left-to-right brightness gradient
  TEST_BREATHE,     // real BCM dimming: one block fading continuously over time
  TEST_ALL_ON,      // everything white
  TEST_ADDR_ID,     // one distinct colour per address and bank
  TEST_GROUP_MARK,  // bit 0 of all 8 groups at once
  TEST_LOW_BYTE,    // bits 0-7 of all 8 groups at once
  TEST_WORD_WALK,   // one 16-bit chip word at a time
  TEST_BIT_WALK,    // one single LED at a time
  TEST_MODE_COUNT
};

static TestMode mode = TEST_GEOMETRY;
static uint32_t clockHz = I2S_CLOCK_DEFAULT_HZ;
// Survives a soft reset, so a reboot loop is obvious in the log.
RTC_DATA_ATTR static uint32_t bootCount = 0;
static uint32_t modeStarted = 0;
static const uint32_t MODE_MS = 6000;

// Raw cycle ruler. Drives D1 and D2 high for the first (step + 1) blocks of 48
// DMA cycles on address 0, ignoring the pixel mapping entirely.
//
// 48 cycles is exactly one group, which covers 8 columns across all three
// colours. So at step 0 the leftmost 8 columns of rows 0, 4, 8 and 12 should be
// white; at step 1, 16 columns; and so on to the full width at step 7.
//
// Each block now generates exactly 48 rising edges in the encoded waveform.
// Lit on every plane, so it is full brightness like before BCM existed.
static void patternCycleRuler(uint32_t step) {
  int groups = (int)(step % GROUPS_PER_STREAM) + 1;
  int upto = groups * 48;
  if (upto > SHIFT_CYCLES) {
    upto = SHIFT_CYCLES;
  }
  for (uint8_t p = 0; p < TOTAL_PLANES; ++p) {
    for (int c = 0; c < upto; ++c) {
      *cell(0, p, c) |= (uint16_t)(BUS_D1 | BUS_D2);
    }
  }
}

// A deliberately asymmetric picture. If setPixel and the DMA stream are both
// right this reads correctly; any mirroring, row swap, group reversal or 16-bit
// word swap is obvious at a glance.
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

// Horizontal brightness ramp: proves intermediate BCM levels are actually
// distinguishable (not just banded into on/off) and that the panel does not
// flicker or ghost at partial brightness. Left = dim, right = full, in four
// row-bands of red / green / blue / white so every channel is exercised.
static void patternBrightnessRamp() {
  for (uint8_t x = 0; x < PANEL_WIDTH; ++x) {
    uint8_t level = (uint8_t)((uint16_t)x * 255 / (PANEL_WIDTH - 1));
    for (uint8_t y = 0; y < PANEL_HEIGHT; ++y) {
      uint8_t band = y / 4;  // four 4-row bands
      uint8_t r = (band == 0 || band == 3) ? level : 0;
      uint8_t g = (band == 1 || band == 3) ? level : 0;
      uint8_t bch = (band == 2 || band == 3) ? level : 0;
      setPixelBrightness(x, y, r, g, bch);
    }
  }
}

// A filled block whose brightness breathes continuously, to show the dimming
// is really live PWM varying in real time, not a fixed static level baked in
// at draw time. Triangle wave, 0 -> 255 -> 0 over 3 seconds.
static void patternBreathe() {
  uint32_t t = millis() % 3000;
  uint32_t half = (t < 1500) ? t : (3000 - t);
  uint8_t level = (uint8_t)(half * 255 / 1500);
  for (uint8_t y = 4; y < 12; ++y) {
    for (uint8_t x = 16; x < 48; ++x) {
      setPixelBrightness(x, y, level, level, level);
    }
  }
}

// Every LED of every colour, both banks, all four addresses.
static void patternAllOn() {
  for (uint8_t addr = 0; addr < ADDR_COUNT; ++addr) {
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

static void patternGroupMark() {
  for (uint8_t grp = 0; grp < GROUPS_PER_STREAM; ++grp) {
    setWord(0, BANK_TOP, grp, COL_R, 0x0001);
  }
}

static void patternLowByte() {
  for (uint8_t grp = 0; grp < GROUPS_PER_STREAM; ++grp) {
    setWord(0, BANK_TOP, grp, COL_R, 0x00FF);
  }
}

static void patternWordWalk(uint32_t step) {
  uint8_t idx = step % (GROUPS_PER_STREAM * 3);  // 24 words per stream
  setWord(0, BANK_TOP, idx / 3, idx % 3, 0xFFFF);
}

static void patternBitWalk(uint32_t step) {
  setWord(0, BANK_TOP, 0, COL_R, 1u << (step % 16));
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
    unsigned groups = (unsigned)(step % GROUPS_PER_STREAM) + 1;
    Serial.printf("  %u cycle blocks -> expect %u columns lit\n", groups,
                  groups * 8);
  } else if (mode == TEST_WORD_WALK) {
    uint8_t idx = step % (GROUPS_PER_STREAM * 3);
    Serial.printf("  word %2u  (group %u, %c)\n", idx, idx / 3, "BGR"[idx % 3]);
  } else if (mode == TEST_BIT_WALK) {
    Serial.printf("  bit %2u\n", (unsigned)(step % 16));
  }
}

static void drawCurrent(uint32_t step) {
  clearAll();
  switch (mode) {
    case TEST_CYCLE_RULER:     patternCycleRuler(step);    break;
    case TEST_GEOMETRY:        patternGeometry();          break;
    case TEST_BRIGHTNESS_RAMP: patternBrightnessRamp();    break;
    case TEST_BREATHE:         patternBreathe();           break;
    case TEST_ALL_ON:          patternAllOn();             break;
    case TEST_ADDR_ID:         patternAddrId();            break;
    case TEST_GROUP_MARK:      patternGroupMark();         break;
    case TEST_LOW_BYTE:        patternLowByte();           break;
    case TEST_WORD_WALK:       patternWordWalk(step);      break;
    case TEST_BIT_WALK:        patternBitWalk(step);       break;
    default: break;
  }
  buildDmaWaveform();
}

// ------------------------------------------------------------- transport
//
// Both transports share the logical pixel data; DMA also encodes the panel clock.

static bool useDma = true;
static bool i2sRunning = false;

// Hand the pins back from the I2S matrix to ordinary GPIO output.
static void releasePinsToGpio() {
  const int8_t pins[BUS_WIDTH] = {PIN_D1, PIN_D2, PIN_LAT, PIN_OE,
                                      PIN_A0, PIN_A1, PIN_CLK};
  for (uint8_t i = 0; i < BUS_WIDTH; ++i) {
    gpio_matrix_out(pins[i], SIG_GPIO_OUT_IDX, false, false);
    pinMode(pins[i], OUTPUT);
  }
  digitalWrite(PIN_OE, HIGH);  // blank while switching, so nothing flashes
  digitalWrite(PIN_LAT, LOW);
  digitalWrite(PIN_CLK, LOW);
}

// One full pass over the buffer, driven entirely by the CPU.
//
// This deliberately mirrors the original bit-banged driver in
// reference/bitbang_reference.cpp, which is known to work on this panel: it uses
// digitalWrite for its ~100ns of data setup time, latches with no clock running,
// and holds the display with a delay rather than clocking through it. The delay
// is now repeated once per BCM plane, scaled by that plane's binary weight --
// this is a coarse approximation (not cycle-accurate like the DMA path) since
// bit-bang is a diagnostic fallback, not a performance target.
//
// Only the source of the data differs -- the bits come from the shift cycles of
// the DMA buffer. So if a picture is right here, the buffer layout and pixel
// mapping are both good and any fault lies in the I2S peripheral instead.
static void bitbangRefresh() {
  for (uint8_t a = 0; a < ADDR_COUNT; ++a) {
    for (uint8_t p = 0; p < TOTAL_PLANES; ++p) {
      digitalWrite(PIN_OE, HIGH);

      for (int c = 0; c < SHIFT_CYCLES; ++c) {
        uint16_t v = *cell(a, p, c);
        digitalWrite(PIN_D1, (v & BUS_D1) ? HIGH : LOW);
        digitalWrite(PIN_D2, (v & BUS_D2) ? HIGH : LOW);
        digitalWrite(PIN_CLK, HIGH);
        digitalWrite(PIN_CLK, LOW);
      }

      digitalWrite(PIN_A0, a & 0x01);
      digitalWrite(PIN_A1, (a >> 1) & 0x01);

      digitalWrite(PIN_LAT, HIGH);
      delayMicroseconds(1);
      digitalWrite(PIN_LAT, LOW);

      digitalWrite(PIN_OE, LOW);
      // planeWeight(), not a power-of-two shift: the floor plane (index
      // BCM_BITS) is deliberately smaller than the main ladder's plane 0.
      delayMicroseconds(planeWeight(p));
      digitalWrite(PIN_OE, HIGH);
    }
  }
}

// WS stays internal; CLK is encoded alongside LAT and the data bits.
static bool startI2S() {
  if (i2sRunning) {
    i2sParallelStop();
    i2sRunning = false;
  }
  const int8_t busPins[BUS_WIDTH] = {PIN_D1, PIN_D2, PIN_LAT,
                                     PIN_OE, PIN_A0, PIN_A1, PIN_CLK};
  if (!i2sParallelBegin(busPins, BUS_WIDTH, -1, clockHz * SAMPLES_PER_CYCLE, dmaWaveform,
                        WAVEFORM_SAMPLES * sizeof(uint16_t), false)) {
    return false;
  }
  i2sRunning = true;
  uint32_t hz = i2sParallelActualHz();
  Serial.printf("DMA samples %u Hz, panel shift clock %u Hz, refresh %u Hz (encoded CLK)\n",
                (unsigned)hz, (unsigned)(hz / SAMPLES_PER_CYCLE),
                (unsigned)(hz / WAVEFORM_SAMPLES));
  return true;
}

// ------------------------------------------------------------------- sketch

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("LED Tile v2 driver - I2S parallel DMA [encoded CLK / BCM dimming]");
  // If this number climbs on its own, the sketch is crash-looping rather than
  // sitting still, and the DMA is simply replaying its last buffer.
  Serial.printf("boot #%u\n", (unsigned)++bootCount);
  Serial.printf("BCM: %u bits (%u levels/channel), base %u cycles, "
                "+1 floor plane at %u cycles\n",
                (unsigned)BCM_BITS, (unsigned)(1u << BCM_BITS),
                (unsigned)BCM_BASE_CYCLES, (unsigned)FLOOR_CYCLES);
  Serial.printf("gamma: %s (CIE1931 blend %.0f%%, floored to the floor plane)\n",
                gammaEnabled ? "ON" : "OFF", (double)(gammaStrength * 100.0f));

  computePlaneLayout();
  buildGammaLUT();

  dma = (uint16_t *)heap_caps_malloc(DMA_WORDS * sizeof(uint16_t),
                                     MALLOC_CAP_DMA);
  dmaWaveform = (uint32_t *)heap_caps_malloc(WAVEFORM_SAMPLES * sizeof(uint16_t),
                                            MALLOC_CAP_DMA);
  if (!dma || !dmaWaveform) {
    Serial.println("FATAL: could not allocate DMA buffer");
    while (true) {
      delay(1000);
    }
  }
  Serial.printf("DMA waveform: %u samples (%u bytes)\n", (unsigned)WAVEFORM_SAMPLES,
                (unsigned)(WAVEFORM_SAMPLES * sizeof(uint16_t)));

  buildControl();
  drawCurrent(0);
  if (!validateDmaWaveform()) {
    releasePinsToGpio();
    Serial.println("FATAL: invalid DMA clock/latch waveform");
    while (true) delay(1000);
  }
  Serial.println("waveform check: OK (384 rising clocks and one clock-low latch per plane)");

  if (!startI2S()) {
    Serial.println("FATAL: I2S parallel setup failed");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("keys: n/./, = mode and step, a = auto,");
  Serial.println("      + / - = clock, r = redraw");
  Serial.println("      d = toggle DMA / bit-bang transport");
  Serial.println("      g = toggle gamma correction, [ / ] = gamma strength");
  Serial.println("      c = cycle ruler (starts at one block)");
  Serial.println();
  {
    I2SParallelStatus st;
    i2sParallelGetStatus(&st);
    Serial.printf("descriptors: %u at %08x\n", (unsigned)st.descCount,
                  (unsigned)st.descBase);
  }
  Serial.println("HELD at GEOMETRY. Press 'd' to compare DMA against bit-bang,");
  Serial.println("'a' to cycle patterns, 'n' to step modes.");

  modeStarted = millis();
  Serial.printf("mode: %s\n", modeName(mode));
}

void loop() {
  static uint32_t step = 0;
  static uint32_t lastStep = 0;
  // Boot held, so CYCLE_RULER sits at one block and can actually be counted.
  static bool manual = true;
  bool redraw = false;

  // Serial control, so a pattern can be held still and read off carefully.
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'n' || c == 'c') {
      mode = (c == 'c') ? TEST_CYCLE_RULER
                        : (TestMode)((mode + 1) % TEST_MODE_COUNT);
      step = 0;
      manual = true;
      redraw = true;
      Serial.printf("mode: %s\n", modeName(mode));
      reportStep(step);
    } else if (c == '.') {
      ++step;
      manual = true;
      redraw = true;
      reportStep(step);
    } else if (c == ',') {
      if (step) --step;
      manual = true;
      redraw = true;
      reportStep(step);
    } else if (c == 'a') {
      manual = false;
      modeStarted = millis();
      Serial.println("auto-advance resumed");
    } else if (c == '+') {
      clockHz = (clockHz < 4000000UL) ? clockHz * 2 : clockHz;
      if (useDma) startI2S();
    } else if (c == '-') {
      clockHz = (clockHz > 250000UL) ? clockHz / 2 : clockHz;
      if (useDma) startI2S();
    } else if (c == 'd') {
      useDma = !useDma;
      if (useDma) {
        startI2S();
        Serial.println("transport: I2S DMA");
      } else {
        if (i2sRunning) {
          i2sParallelStop();
          i2sRunning = false;
        }
        releasePinsToGpio();
        Serial.println("transport: bit-bang (same buffer, CPU driven)");
      }
    } else if (c == 'i') {
      Serial.println("CLK is DMA-encoded; polarity is fixed to rising-edge shifting.");
    } else if (c == 'g') {
      gammaEnabled = !gammaEnabled;
      redraw = true;
      Serial.printf("gamma: %s (CIE1931 blend %.0f%%, floored to the floor plane)\n",
                    gammaEnabled ? "ON" : "OFF", (double)(gammaStrength * 100.0f));
    } else if (c == '[' || c == ']') {
      gammaStrength += (c == ']') ? 0.1f : -0.1f;
      if (gammaStrength < 0.0f) gammaStrength = 0.0f;
      if (gammaStrength > 1.0f) gammaStrength = 1.0f;
      buildGammaLUT();
      redraw = true;
      Serial.printf("gamma strength: %.0f%% (0%%=linear, 100%%=full CIE1931)\n",
                    (double)(gammaStrength * 100.0f));
    } else if (c == 'r') {
      redraw = true;
    } else if (c == '?') {
      Serial.println("n/./, = mode and step, a = auto, + - = clock, g = gamma, "
                     "[ ] = gamma strength, r = redraw, c = cycle ruler, d = transport");
    }
  }

  // Heartbeat. If millis() keeps climbing the CPU is alive and any frozen image
  // is a DMA problem; if it restarts from zero, the sketch is rebooting.
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 2000) {
    lastBeat = millis();
    if (useDma) {
      const uint32_t hz = i2sParallelActualHz();
      const uint32_t timeoutUs = hz ? (2000000ULL * WAVEFORM_SAMPLES / hz) + 1000 : 1000;
      const bool frameCompleted = i2sParallelWaitForFrame(timeoutUs);
      I2SParallelStatus st;
      i2sParallelGetStatus(&st);
      Serial.printf(
          "alive t=%us mode=%d frame=%s clk_en=%u tx=%u park=%u err=%u "
          "dscr=%08x eof=%08x int=%08x st=%08x\n",
          (unsigned)(millis() / 1000), (int)mode,
          frameCompleted ? "OK" : "TIMEOUT", (unsigned)st.clockEnabled,
          (unsigned)st.txStarted, (unsigned)st.dmaParked,
          (unsigned)st.descriptorError, (unsigned)st.linkDscr,
          (unsigned)st.eofDesc, (unsigned)st.intRaw, (unsigned)st.state);
    } else {
      Serial.printf("alive t=%us mode=%d (bit-bang)\n",
                    (unsigned)(millis() / 1000), (int)mode);
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
    // The walking patterns advance slowly enough to follow by eye.
    if (millis() - lastStep > 700) {
      lastStep = millis();
      ++step;
      redraw = true;
      reportStep(step);
    }
  }

  // BREATHE animates continuously from millis(), independent of the step/mode
  // timers above, so force a redraw every ~30ms while it is on screen -- that
  // is what proves the dimming is live PWM rather than a fixed static level.
  static uint32_t lastBreatheDraw = 0;
  if (mode == TEST_BREATHE && millis() - lastBreatheDraw > 30) {
    lastBreatheDraw = millis();
    redraw = true;
  }

  // The DMA chain plays continuously, so drawing only has to happen when the
  // picture actually changes.
  if (redraw) {
    drawCurrent(step);
  }

  if (useDma) {
    delay(5);  // DMA plays on its own; nothing to do here
  } else {
    bitbangRefresh();
  }
}
