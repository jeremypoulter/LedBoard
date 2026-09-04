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

### I2S parallel DMA

The driver no longer bit-bangs. All six control signals ride on the I2S bus alongside
the data, and the pixel clock comes from the I2S word-select line, so latch and
blanking are timed by the DMA stream rather than by CPU writes.

The DMA buffer holds one 16-bit word per clock cycle. Each address gets a block of:

| Cycles      | LAT  | OE        | What happens                       |
|-------------|------|-----------|------------------------------------|
| 0 - 383     | low  | high      | 384 data bits shifted in, blanked  |
| 384         | high | high      | latch pulse                        |
| 385         | low  | high      | latch released                     |
| 386 - 769   | low  | **low**   | display window, row lit            |

At a 10 MHz pixel clock that is 3080 cycles per frame, giving roughly a **3.2 kHz
refresh** with a 50% duty cycle -- far above the flicker threshold, and with ample
headroom for the bit planes that BCM will need. The bus bit assignments live in
[src/main.cpp](src/main.cpp) and the peripheral setup in
[src/i2s_parallel.cpp](src/i2s_parallel.cpp).

Because the ESP32 routes I2S through the GPIO matrix, **the wiring does not change**.

The previous bit-banged driver is kept verbatim at
[reference/bitbang_reference.cpp](reference/bitbang_reference.cpp) as a known-good
fallback, since this project is not under version control.

### Remaining work

1. **BCM / temporal dithering** for more than 8 colours. The DMA layout is already
   shaped for it: each bit plane becomes another block per address, with the display
   window length carrying the plane's binary weight.
2. **Double buffering**, so drawing never tears against the running DMA chain.

## References

- <https://wiki.london.hackspace.org.uk/view/LED_tiles_V2>
- <https://led.limehouselabs.org/docs/tiles/led-tiles-v2/>