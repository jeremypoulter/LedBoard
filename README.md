# LED Tile v2 Driver

This is an ESP32 driver for the LED Tile v2, which is a 64 x 16 LED matrix display based on the MBI5034 LED controller chips.

## Features
- 64 x 16 LED matrix display
- PWM dimming
- 24-bit color depth
- DMA driven

## LED Tile v2

### Description

These tiles have a 1-bit colour depth and so displaying more than 8 colours requires the use of temporal dithering (PWM).

There are 48 x Macroblock MBI5034 LED controller chips on the PCB, which are 16-bit shift register constant current sinks.

The controller chips are labelled R,G & B 1-16. So each chip does 16 LEDs of the same colour. They each have a current set resistor. 

There are two address line inputs and the LED chips seem to be soldered in to banks of 4. 

There is an HC138 decoder on the address lines, and only the first four outputs seem to be wired. 

There are also 8 dual FET chips, which seem to control the 4 banks.

### Coding

The serial data sent to `D1` & `D2` needs to be sent in 3 x 16 bit B,G,R frames, **8 times** per stream (i.e. one stream per data line is 384 bits, or 48 bytes), then this is latched, and then the output is enabled.

The arithmetic: 48 x MBI5034 at 16 bits each gives 768 outputs, and the panel has 64 x 16 x 3 = 3072 LEDs. 3072 / 768 = 4, which is exactly the number of address states. Those 768 outputs are split across the two banks, so each data line carries 384 bits per address, and 384 / 3 colours = 128 pixels = two rows of 64.

```
  BBBBBBBB BBBBBBBB GGGGGGGG GGGGGGGG RRRRRRRR RRRRRRR | BBBBBBBB BBBBBBBB GGGGGGGG GGGGGGGG RRRRRRRR RRRRRRR | BBBBBBBB BBBBBBBB GGGGGGGG GGGGGGGG RRRRRRRR RRRRRRR ... etc .. x 384 bits
```

There are two 'banks'; the top 8 rows and bottom 8 rows. There are loaded with data using two different data in lines, D1 & D2. The data lines are clocked and latched with the same shared clock and latch signals.

In each bank, there are 8 rows. A single stream of 384 bits loads two rows of 2 x 64 LEDs. The address lines A0 & A1 select which row is being loaded from the serial data.

To display a full tile, of all 1024 RGB LEDs, you need to :

1. Set `OE=HIGH` and `LAT=LOW`
2. For each address `A0` and `A1` = `0b00`, `0b01`, `0b10`, `0b11` :
    1. Clock 384 bits of data on to `D1` and `D2`, by toggling the `CLK` pin from `LOW` to `HIGH`
    2. Toggling `LAT` pin from `HIGH` to `LOW` to latch the 384 bits in to the driver chips.
    3. Toggle the `OE` pin `LOW` to turn the LEDs on for that row.
3. Clock 4 x 384 bits of data to light every row of LEDs in sequence.

Repeat this very quickly as fast as you can ! :-)

The LEDs are either on or off, there is no brightness control of individual LEDs possible. To mix colours, you need to implement a PWM or BCM waveform with each refresh cycle of the panel.

### Pinout

The tile presents **three 2x4 2.54 mm headers**: the top one is all 5V, the bottom one is
all GND, and the middle one carries the eight logic signals.

```
   Middle (signal) header, 2x4          Top header  : 8 x 5V  (4 top half, 4 bottom half)
   +---------------------+              Bottom hdr  : 8 x GND
   | 1  D1  |  2  LAT    |
   | 3  A1  |  4  NC     |              Verify pin 1 against the silkscreen before
   | 5  D2  |  6  OE     |              connecting. There is NO polarity protection:
   | 7  A0  |  8  CLK    |              reversing 5V and GND will destroy the board.
   +---------------------+
```

`A0` and `A1` are the address lines which select a pair of rows. `D1` and `D2` are the
serial data lines for the top and bottom banks, accepting interleaved 16-bit BGR data.

### ESP32 pin mapping

Logic is **3.3V direct drive** - the panel inputs are happy with the ESP32's levels, so no
level shifter is needed. Power is a separate matter: the tile needs **5V at ~8A** from its
own supply, and only the ground is shared with the ESP32.

| ESP32 GPIO | Signal | Direction | Notes                                      |
|------------|--------|-----------|--------------------------------------------|
| 26         | `D1`   | out       | Serial data, top bank (rows 0-7)            |
| 18         | `D2`   | out       | Serial data, bottom bank (rows 8-15)        |
| 25         | `CLK`  | out       | Shared shift clock, rising edge             |
| 19         | `LAT`  | out       | Shared latch, HIGH then LOW                 |
| 21         | `OE`   | out       | Output enable, **active LOW**               |
| 22         | `A0`   | out       | Row-pair select, bit 0                      |
| 23         | `A1`   | out       | Row-pair select, bit 1                      |
| GND        | `GND`  | -         | Must be common with the 5V supply ground    |

### Wiring

```
   ESP32-WROVER-KIT                                LED Tile v2
   (3.3V logic)                                    (2x4 signal header)

     GPIO26  >--------------------------------->   D1    pin 1
     GPIO18  >--------------------------------->   D2    pin 5
     GPIO25  >--------------------------------->   CLK   pin 8
     GPIO19  >--------------------------------->   LAT   pin 2
     GPIO21  >--------------------------------->   OE    pin 6
     GPIO22  >--------------------------------->   A0    pin 7
     GPIO23  >--------------------------------->   A1    pin 4
                                                   NC    pin 3  (leave open)

        GND  o------------------+------------->   GND header (all 8 pins)
                                |
                                |
   5V PSU  ( 5V / 8A+ )  -------+
        |
        +------------------------------------->   5V header (all 8 pins)

   Grounds MUST be common. Do not power the tile from the ESP32's 5V pin -
   a fully lit panel draws ~7-8A, far beyond what USB or the dev board can supply.
```

### Pins to avoid on this board

The pin map above is not arbitrary. On an **ESP-WROVER-KIT** these GPIOs are unavailable
or hazardous:

| GPIO      | Why it is unusable                                                          |
|-----------|------------------------------------------------------------------------------|
| 6-11      | Wired to the SPI flash inside the module. Using them will crash the chip.     |
| 16, 17    | PSRAM chip-select and clock, bonded **inside** the WROVER module. Not free.   |
| 34-39     | **Input only.** These pins have no output driver and cannot drive a signal.   |
| 1, 3      | UART0, used by the 115200 baud serial monitor.                               |
| 0, 2, 5, 12, 15 | Strapping pins sampled at reset. GPIO12 held high stops the board booting. |

Additionally, the WROVER-KIT's onboard ILI9341 LCD occupies **GPIO 5, 18, 19, 21, 22, 23
and 25**. The mapping above deliberately reuses six of those, so the **LCD module must be
unplugged**. GPIO25 is the LCD's MISO line: if the display is left connected it will
actively drive that pin whenever its chip-select (GPIO22) goes low, causing bus contention
with the panel clock.

If you would rather keep the LCD attached, this alternative map avoids every onboard
peripheral, using only pins that are free on both WROOM-32 and WROVER modules:

| Signal | Alternative GPIO |
|--------|------------------|
| `D1`   | 27               |
| `D2`   | 32               |
| `CLK`  | 26               |
| `LAT`  | 33               |
| `OE`   | 14               |
| `A0`   | 13               |
| `A1`   | 4                |

On the WROVER-KIT, GPIO13 and GPIO14 are shared with the JTAG and microSD jumpers, and
GPIO4 with the blue RGB LED, so remove those jumpers if you use this map.

Either map remains valid if the driver later moves to I2S parallel DMA: the ESP32 routes
I2S signals through the GPIO matrix, so any output-capable pin can carry any signal.

### Confirmed row mapping

Verified on hardware with the `ADDR_ID` test pattern. Each address state lights two
rows per bank, but they are **four rows apart**, not adjacent:

| Address | Top bank (`D1`) | Bottom bank (`D2`) |
|---------|-----------------|--------------------|
| `0b00`  | rows 0 and 4    | rows 8 and 12      |
| `0b01`  | rows 1 and 5    | rows 9 and 13      |
| `0b10`  | rows 2 and 6    | rows 10 and 14     |
| `0b11`  | rows 3 and 7    | rows 11 and 15     |

```
bank      = row / 8        (0 -> D1, 1 -> D2)
rowInBank = row % 8
address   = rowInBank % 4
half      = rowInBank / 4  (which 64-pixel half of the 128-position stream)
```

The same test also confirms that the colour order really is **B, G, R**, and that `D1`
drives the top eight rows while `D2` drives the bottom eight.

### Confirmed shift order

Verified with the `BIT_WALK` test pattern. As the bit index rises the lit LED sweeps
cleanly **right to left** across the panel, in one unbroken run. A wrong byte order
would instead show two reversed runs of eight, so the panel is **MSB first**, and
`SHIFT_MSB_FIRST` is correctly set to 1.

Bit 0 therefore lands at the **right-hand edge** of the panel, with higher bits running
leftwards. Any framebuffer will need to mirror the x axis when packing pixels.

### Confirmed pixel mapping

The final piece came from `LOW_BYTE`, which lit row 4 end to end and left row 0 dark.
The two rows an address serves are split by **byte**, not by bit parity:

- **Low byte** of each 16-bit word (bits 0-7) feeds the **higher** row (`rowInBank + 4`)
- **High byte** (bits 8-15) feeds the **lower** row (`rowInBank`)

The complete coordinate transform, every step verified against hardware:

```
bank      = y / 8           D1 drives rows 0-7, D2 drives rows 8-15
rowInBank = y % 8
address   = rowInBank % 4   each address serves rowInBank and rowInBank + 4
byte      = (rowInBank >= 4) ? low : high
group     = x / 8           group 0 is leftmost
bit       = 7 - (x % 8)     higher bits sit further left
```

This is implemented as `setPixel(x, y, r, g, b)` in [src/main.cpp](src/main.cpp), and the
`GEOMETRY` test pattern draws a border, a diagonal and corner markers to prove it.

### I2S parallel DMA, with an encoded clock

The driver no longer bit-bangs by default. All seven signals -- `D1`, `D2`, `LAT`,
`OE`, `A0`, `A1` and now **`CLK` itself** -- ride as plain data bits inside the I2S
DMA stream, rather than `CLK` coming from the I2S word-select line.

That last point was a real bring-up lesson, not a stylistic choice: an earlier
version used WS as the hardware-generated pixel clock, and it stalled after exactly
one DMA descriptor (confirmed on hardware via `out_link_dscr` freezing and a latched
`TX_REMPTY`). Generating `CLK` in software by oversampling -- each logical shift
cycle expands to four raw I2S samples encoding "low, low, high, high" -- removed the
dependency on that WS/BCK timing path entirely and has been stable since. The
tradeoff is a 4x sample-rate cost (`SAMPLES_PER_CYCLE` in
[src/main.cpp](src/main.cpp)), which the ESP32's DMA throughput comfortably absorbs
at these panel sizes.

A startup self-check (`validateDmaWaveform()`) walks the entire encoded waveform in
software before I2S is ever started, confirming exactly 384 rising clock edges and
one clean latch pulse per bit-plane per address. If a future change to the layout
introduces an off-by-one, this catches it as a boot-time `FATAL:` message instead of
a mysteriously wrong picture on the panel.

Because the ESP32 routes I2S through the GPIO matrix, **the wiring does not change**.
The previous bit-banged driver is kept verbatim at
[reference/bitbang_reference.cpp](reference/bitbang_reference.cpp) as a known-good
fallback; it is also reachable live at runtime with the `d` serial key, replaying the
exact same buffer through plain `digitalWrite` instead of DMA -- invaluable for
telling "the buffer is wrong" apart from "the I2S peripheral is misbehaving".

### BCM dimming

The panel is strictly 1 bit per channel per LED -- no analogue brightness control
exists at the hardware level, as the "Coding" section above established. Dimming is
therefore **Binary Code Modulation**: each address is split into `BCM_BITS` complete
shift+latch+display sub-passes ("bit planes"), each displayed for a duration
proportional to its binary weight (1, 2, 4, 8... cycles). A pixel's brightness is
encoded by which planes its bit is set in.

```
one address, BCM_BITS = 4:

  plane 0  shift 384 bits -> latch -> display  2 cycles   (weight 1)
  plane 1  shift 384 bits -> latch -> display  4 cycles   (weight 2)
  plane 2  shift 384 bits -> latch -> display  8 cycles   (weight 4)
  plane 3  shift 384 bits -> latch -> display 16 cycles   (weight 8)
```

Address lines only actually move at the first plane of each address (mirroring the
original bit-bang driver, which touches `A0`/`A1` once per address); later planes of
the same address hold the address steady, since no real transition is happening.

`BCM_BITS` defaults to **4** (16 levels per channel, 4096 colours), chosen as a
first, conservative step rather than jumping straight to full 8-bit depth, given how
many real hardware surprises this board has produced through bring-up already. The
cost of raising it is real and compounds per bit, since the fixed 390-cycle
shift+latch overhead repeats for every plane, not just the display window.

**`BCM_BASE_CYCLES`** (the shortest plane's display length, in shift-clock cycles)
is just as important as `BCM_BITS` and easy to get badly wrong -- an earlier value of
2 gave technically-correct relative dimming levels, but only a **1.9% duty cycle
even at full white** (30 display cycles against 1560 cycles of repeated shift
overhead), which read as "barely lit" rather than dim. The current value of 32
targets roughly a **quarter of the time lit at full brightness** -- dimmer than the
pre-BCM design's ~50% duty, but in the same order of magnitude rather than two
orders of magnitude off:

| `BCM_BITS` | `BASE` | Levels/channel | Max duty | DMA words | Buffer size | Refresh @ 2 MHz |
|------------|--------|-----------------|----------|-----------|--------------|------------------|
| -- (pre-BCM baseline) | -- | 2  | ~49.6%  | 3080  | ~54 KB  | ~649 Hz |
| 2          | 32     | 4               | ~11.0%   | 3504      | ~62 KB       | ~571 Hz          |
| 3          | 32     | 8               | ~16.1%   | 5576      | ~98 KB       | ~359 Hz          |
| 4 (current)| 32     | 16              | ~23.5%   | 8160      | ~143 KB      | ~245 Hz          |
| 5          | 32     | 32              | ~33.7%   | 11768     | ~207 KB      | ~170 Hz          |

(Refresh and duty-cycle figures are computed from the I2S clock divider in
[src/i2s_parallel.cpp](src/i2s_parallel.cpp) and the layout macros in
[src/main.cpp](src/main.cpp), not measured -- at the default clock the divider
reproduces the hardware-confirmed 649 Hz of the pre-BCM, single-plane design
exactly, which is what makes the derived numbers above trustworthy.) `BCM_BITS` 5
is the point where the ~184KB `dmaWaveform` buffer starts risking not finding a
single contiguous block in the ESP32's internal DMA-capable RAM; both buffer
allocations already fail safely with a `FATAL: could not allocate DMA buffer`
halt-and-print rather than silent corruption. Raising `BCM_BASE_CYCLES` instead of
`BCM_BITS` trades the same memory for brightness rather than for more levels --
useful if 16 levels is enough but the panel still looks dim.

Two new test patterns exercise real dimming rather than solid on/off:

- **`BRIGHTNESS_RAMP`** -- a static left-to-right gradient in four row-bands (red,
  green, blue, white), to confirm intermediate levels are actually distinguishable
  and that partial brightness does not flicker or ghost.
- **`BREATHE`** -- a filled block whose brightness fades continuously on a 3-second
  triangle wave, proving the dimming is live PWM rather than a fixed level baked in
  at draw time.

`setPixel(x, y, bool r, g, b)` is unchanged and still drives every plane identically
(full brightness or off), so every earlier diagnostic pattern keeps working exactly
as before. The new `setPixelBrightness(x, y, uint8_t r, g, b)` is the real API,
taking 0-255 per channel and using its top `BCM_BITS` bits.

### Gamma correction

Confirmed on hardware: linear duty cycle looks badly uneven to the eye -- a big
jump in brightness from off to the lowest non-zero BCM level, then little
perceptible change between the higher levels. This is expected: human brightness
perception is close to a power law, not linear, so it amplifies differences at the
low end and compresses them at the high end. A linear PWM duty cycle rides straight
into that curve unmodified.

A first attempt used a standard power-law gamma table (exponent 2.8, the common
LED-strip default) and made things **worse**, confirmed on hardware: "almost half
the screen is now black". The exponent is tuned for 8-bit-plus output, where
compressing the dim end still leaves plenty of representable codes there; with only
`BCM_BITS`=4 (16 levels), the same curve pushes **37%** of the 0-255 input range
below the bottom representable code, so a third of the image rendered as literally
off rather than dim. Two independent fixes, both needed:

1. **The CIE 1931 perceptual lightness formula** instead of a raw power law. It has
   a linear segment near black rather than a curve whose slope keeps falling all
   the way to zero, so it loses less of the input range to the bottom code in the
   first place (calculated: 30% instead of 37% -- better, but not sufficient alone).
2. **A hard floor**: any input that is not exactly zero is guaranteed to round to at
   least BCM level 1, never level 0. This is what actually closes the dead zone,
   independent of the curve's shape.

Both live in `buildGammaLUT()` and `setPixelBrightness()` in
[src/main.cpp](src/main.cpp). Toggle the whole thing live with the `g` serial key to
compare against uncorrected linear output -- both `BRIGHTNESS_RAMP` and `BREATHE` go
through `setPixelBrightness()`, so either demonstrates the difference immediately.
Gamma is on by default.

Confirmed on hardware again, one layer deeper: even with both fixes above, there was
still a visible jump from off to the dimmest representable brightness -- because the
floor was forcing those values up to the SAME duty cycle as the main ladder's level
1 (1.57%), not to something genuinely dimmer. The fix is a **dedicated extra bit
plane** (`FLOOR_CYCLES` in [src/main.cpp](src/main.cpp)), smaller than the main
ladder's own smallest level and used only for values gamma would otherwise crush to
zero. Every level from 1 upward is untouched -- the floor plane is simply off
whenever the main ladder has anything lit. Calculated result, at the current
settings:

```
off      0.000%
floor    0.328%   <- new, genuinely dim step
level 1  1.313%
level 2  2.625%
...
level 15 19.688%   (max brightness)
```

This costs one more full shift+latch pass per address (`TOTAL_PLANES = BCM_BITS +
1`), which is not free: DMA words rise from 8160 to 9752, buffer size from ~143KB to
~172KB, refresh from ~245Hz to ~205Hz, and max brightness drops slightly from 23.5%
to 19.7% duty (more of every address's time is now spent on shift overhead rather
than display). All still comfortably within the safe range established earlier in
this document.

### Gamma strength

Confirmed on hardware, one more layer deeper: full-strength CIE1931 overcorrects.
With only ~17 output levels total, the curve's low-end compression crams so many
input values onto the same few dim codes that a visible, unchanging plateau appears
before the ramp starts climbing -- reported as "the lower end is scaling very
slowly". Calculated at full strength, the widest single-level plateau on the 64-wide
`BRIGHTNESS_RAMP` is **19 pixels** -- nearly a third of it stuck on one level.

Rather than pick one fixed replacement value, `gammaStrength` blends linearly
between raw linear input (0%) and the full CIE1931 curve (100%), live-adjustable
with `[` / `]` so it can be dialed in by eye without another edit-rebuild-reflash
cycle. The default dropped from 100% to **50%**, which calculates out to a 7-pixel
widest plateau -- a large reduction, and the rest of the ramp's level widths become
far more even too (mostly 2-6 pixels, rather than one huge outlier next to several
tiny ones). `g` still toggles the correction on/off entirely, independent of
whatever strength `[`/`]` has set.

### The limit of gamma correction, and more BCM levels

Confirmed on hardware: once the low end was fixed, the **high end** still didn't
look like it was changing. This is not something gamma correction can fix, at any
strength or curve shape -- it only decides which of the *existing* output levels an
input maps to; it cannot invent new levels where none exist. At `BCM_BITS`=4 (16
levels), the physical brightness step between adjacent high levels (e.g. 14 and 15)
is only **~7%** -- a real hardware resolution limit, not a curve-shaping problem.

The fix is more levels: `BCM_BITS` raised **4 -> 5** (32 levels), halving that
top-end step to ~3.3%. Raising `BCM_BITS` alone at the existing `BCM_BASE_CYCLES`
would have pushed the DMA buffer to ~235KB, a real risk of not finding one
contiguous block of DMA-capable RAM, so `BCM_BASE_CYCLES` was halved **32 -> 16**
(and `FLOOR_CYCLES` **8 -> 4** alongside it, keeping the same ratio) to compensate:

| | BITS=4 BASE=32 (previous) | BITS=5 BASE=32 (naive bump) | BITS=5 BASE=16 (current) |
|---|---|---|---|
| DMA words | 9752 | 13360 | 11360 |
| Buffer size | ~171 KB | ~235 KB | ~200 KB |
| Refresh | ~205 Hz | ~150 Hz | ~176 Hz |
| Max brightness (duty) | 19.7% | 29.7% | 17.5% |
| Top-level step | 7.1% | 3.3% | 3.3% |

The rescaled version gets the same top-end improvement as the naive bump for a
buffer size much closer to the ~171KB already confirmed working, at the cost of a
small further drop in max brightness (19.7% -> 17.5%). All still comfortably above
the flicker threshold. `gammaStrength` likely wants re-tuning by eye at the new
level count -- the plateau-width calculation above was done for 16 levels, not 32.

### Remaining work

1. **Raise `BCM_BITS`** once `BRIGHTNESS_RAMP` and `BREATHE` are confirmed clean on
   hardware -- see the memory/refresh table above before picking a value.
2. **Double buffering**, so drawing never tears against the running DMA chain.

## References

- <https://wiki.london.hackspace.org.uk/view/LED_tiles_V2>
- <https://led.limehouselabs.org/docs/tiles/led-tiles-v2/>