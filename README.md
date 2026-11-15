# chipguy_JC8012P4A1C_display

A self-contained **display + touch** driver for the 10.1 inch **Guition JC8012P4A1C** —
an **ESP32-P4** board with an **800×1280 JD9365** MIPI-DSI panel and a
**GSL3680** capacitive touch panel. It drives the display and the touch
controller so you don't have to think about either: the hardware is fixed and
fully handled, leaving you to write an **LVGL 9** app and nothing else.

Two more ways to use the screen come with it: a **text terminal** that prints and
scrolls like a computer's console, with no LVGL at all, and a **raster band**, which
lets your own code draw part of the screen in step with the panel, at its full
52 frames a second.

The bundled examples are the heart of the library. Each is a complete, working
**starting point** — the display, touch, and LVGL are already wired up and
running before your code gets control. You pick the one that matches how you
want to build your UI, copy it, and start replacing its placeholder UI with your
own.

This library is one of five for the board:

| Library | What it does |
|---|---|
| **chipguy_JC8012P4A1C_display** (this one) | The screen and touch at the panel's full 800×1280, with LVGL |
| [chipguy_JC8012P4A1C_480display](https://github.com/chipguyhere/JC8012P4A1C_480display) | The same screen drawn 480 pixels wide and scaled up — for UIs written for 480×480 boards |
| [chipguy_JC8012P4A1C_board](https://github.com/chipguyhere/JC8012P4A1C_board) | The shared I²C bus, the pin map, and what else is on the board — speaker, microphone, clock, LED, SD slot, camera, expansion connectors |
| [chipguy_JC8012P4A1C_power](https://github.com/chipguyhere/JC8012P4A1C_power) | The battery and sleep |
| [chipguy_ESP32-P4-C6-firmware-installer](https://github.com/chipguyhere/ESP32-P4-C6-firmware-installer/blob/main/README.md) | Run once per board: installs the touch firmware and updates the Wi-Fi chip |

You need the **board** library installed too — touch talks through it.

---

## Installation

This library is distributed by downloading the code as a ZIP from GitHub —
there is no Arduino Library Manager entry. To install:

1. On the GitHub project page, click **Code ▸ Download ZIP**.
2. Unzip it, and move the resulting folder into your Arduino **libraries**
   directory:
   - macOS / Linux: `~/Documents/Arduino/libraries/`
   - Windows: `Documents\Arduino\libraries\`
3. Do the same for **chipguy_JC8012P4A1C_board**.
4. Restart Arduino IDE so the examples will appear in the File menu.
5. **Once per board:** run the **[firmware installer](https://chipguyhere.github.io/esp-launchpad/minimal-launchpad/?flashConfigURL=https://raw.githubusercontent.com/chipguyhere/ESP32-P4-C6-firmware-installer/main/launchpad.toml)**: one click in Chrome or
   Edge, no IDE needed. Touch needs firmware that this library doesn't include; the
   installer stores it on the board, where it stays through later uploads, and brings
   the Wi-Fi chip's firmware up to date too. Until then the screen works but touch
   doesn't, and the serial log says why. [How and why](https://github.com/chipguyhere/ESP32-P4-C6-firmware-installer/blob/main/README.md).

(Alternatively, in the IDE: **Sketch ▸ Include Library ▸ Add .ZIP Library…**
and select the downloaded ZIP.)

### Dependency: LVGL 9

The examples need **LVGL 9**. Install it via the Arduino **Library Manager**
(search "lvgl"). Each example sketch folder ships its own `lv_conf.h`; LVGL
reads its configuration from that file, so keep it next to the `.ino`.

This library was developed against LVGL 9.3 (the current version of LVGL that is
compatible with SquareLine) and is very likely compatible with later 9.x
versions.

### Arduino IDE board settings

This is an **ESP32-P4** board. In **Tools ▸ Board ▸ esp32**, select
**"ESP32P4 Dev Module"**, and set:

- **PSRAM: Enabled** — **required.** The driver keeps its draw buffers and
  framebuffers in PSRAM; the sketch will not compile with PSRAM disabled.
- **Flash Size: 16 MegaBytes (128 megabits)**
- **Partition Scheme: Custom** — uses the `partitions.csv` bundled with each
  example.
- **USB CDC On Boot: Enabled.**

The examples print to **`Serial0`**, which on this board comes out of the USB-C port
wired to the board's USB-serial chip — the same port you upload through. The board has
three USB-C ports; [which is which](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README-board.md#the-three-usb-c-ports)
is in the board library.

> **On the partition scheme:** each example ships a `partitions.csv` giving **two
> 7.875 MB app slots** (so OTA works) plus a 128 KB LittleFS area and a coredump
> region. This matters — the IDE's default scheme allows only 1.2 MB for the
> sketch, and **LvglCalculator does not fit in that** (it needs about 1.33 MB).
> Select **Custom** so the bundled file is used.

Developed against ESP32 Arduino Core 3.x (the first cores with ESP32-P4
support).

---

## The examples — pick your starting point

Open them from **File ▸ Examples ▸ chipguy_JC8012P4A1C_display**. They all come
up running on the hardware immediately, so you can confirm your board works
before you change a line.

To turn one into your own project, **save it under a new name** (File ▸ Save As)
so your work lives outside the library, then start editing. Each example folder
is self-contained — it carries its own `lv_setup.hpp` and `lv_conf.h` — so a
copied sketch is fully standalone.

### LvglClaudeCodeStub — write the UI yourself (by hand or with an AI)

Use this when you want to build the interface **in code**: directly, or by
handing the sketch to an AI coding assistant such as Claude Code.

The UI lives in a tiny `ui.h` / `ui.cpp` pair in the sketch folder. All the
hardware/LVGL setup happens in the sketch before `ui_init()` is called, so
`ui_init()` is a blank canvas: it receives a live LVGL screen and you create
widgets on it. The demo it ships with (a label that follows your finger, plus a
color-cycling rectangle) is just there to prove the display and touch work —
**delete it** and build your own UI on `lv_screen_active()`.

Because everything except the UI is already wired up, this scaffold is ideal to
hand to **Claude Code**: point it at the sketch folder and describe the app you
want — it can focus entirely on `ui.cpp` without touching display/touch/LVGL
plumbing.

### LvglBouncingBalls — a resolution-independent demo

1–20 balls bounce under gravity with two sliders for count and speed. The whole
UI is laid out from the live display resolution at runtime, so it fills the
screen in any orientation. A good first flash to confirm the panel, touch, and
rotation all work.

### LvglCalculator — a working four-function calculator

A dark-themed touch calculator whose keypad is laid out proportionally from the
live resolution. Defaults to portrait.

### Lvgl93SquarelineLauncher — design the UI visually in SquareLine Studio

Use this when you'd rather **design the interface visually** in
[SquareLine Studio](https://squareline.io/) and run the result on the board.

The sketch's `src/` folder holds a small **placeholder** UI in the exact file
shape a SquareLine export produces (`ui.c`, `ui_Screen1.c`, …), and the sketch
calls the `ui_init()` SquareLine generates. To start your app:

1. In SquareLine Studio, create a project sized to your orientation
   (**800×1280** for portrait, **1280×800** for landscape), **16-bit** color,
   with **no rotation or offset** — this library handles rotation, so keep the
   project unrotated.
2. Lay out your screens visually.
3. **Export ▸ UI files** (the "Arduino TFT_eSPI profile" works, among others).
4. **Replace the whole `src/` folder** with your export. Build and upload.

Treat `src/` as disposable: you overwrite it wholesale each time you re-export.
Your own application logic lives in the **sketch** — after calling `ui_init()`,
attach event handlers from the `.ino` to the UI objects SquareLine exposes (e.g.
`ui_Button1`). That way re-exporting the visuals never touches your code.

---

### RasterBouncingBalls — draw part of the screen yourself, every frame

Up to 200 see-through balls bounce at the panel's full 52 frames a second, drawn by the
sketch's own code a few rows at a time just before the panel shows them, with no
framebuffer. Only the control strip at the bottom is LVGL. It's the worked example of
the raster band: [README-raster.md](README-raster.md).

### RasterLandscapeBouncingBalls — the same, sideways

The balls again, with the screen turned to landscape and the LVGL controls along the
bottom. Turned this way, the panel draws the picture in thin vertical slices, so every
slice holds some balls and a piece of the controls. The sketch's code draws the balls into
each slice, and the controls are copied into it from memory by the chip's DMA engine
(a part that copies memory without the processor). LVGL is kept to a calm 20 updates a
second so it never gets in the balls' way. See
[README-raster.md](README-raster.md#part-of-every-chunk-from-memory-the-prefetch).

### The terminal examples — text instead of LVGL

Six examples, from **TerminalHello** to a *Matrix*-style digital rain, show the text
terminal below.

## The text terminal

Instead of LVGL, a sketch can use the screen as a **text console**: `print()` and
`printf()` write onto a grid of letters, 80 across and 64 down, and the screen scrolls
up as it fills. The display hardware does the scrolling, so it's fast and smooth. It
turns sideways, has a bigger-text setting, and can keep a picture at the top and bottom
while the text scrolls between them. Tapping the screen tells you which letter cell you
touched.

```cpp
#include <chipguy_JC8012P4A1C_terminal.h>

JC8012P4A1C_Board            board;
chipguy_JC8012P4A1C_display  display(board);
chipguy_JC8012P4A1C_terminal term;

void setup() {
    term.begin(display);
    term.println("Hello, JC8012!");
}
```

Everything about it is in **[README-terminal.md](README-terminal.md)**.

## What the examples give you: the `lv_setup` API

Every example wires the hardware to LVGL through one small header,
**`lv_setup.hpp`** (plus **`lv_conf.h`** for LVGL's build config). That header,
and the global objects it defines, is the entire surface you interact with — and
once `begin()` returns, you are just using plain LVGL.

```cpp
#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"

void setup() {
    Serial0.begin(115200);

    lv_setup.begin();           // initialize display + touch + LVGL (default: portrait)
    // or: lv_setup.begin(1);      landscape

    // ...build your UI on lv_screen_active() here (or call ui_init())...
}

void loop() {
    lv_timer_handler();         // let LVGL run
    delay(5);
}
```

What the header provides:

- **`lv_setup.begin(uint16_t rotation = 0)`** — call once in `setup()`. Brings up
  the JD9365 panel, the GSL3680 touch, and LVGL; creates the LVGL display and a
  pointer (touch) input device and connects them. After this returns, the active
  LVGL screen exists and you build your UI with ordinary LVGL calls. The examples
  call it as `lv_setup.begin()` and take the default (portrait).
- **`display`** — a global display object. Handy methods: `display.width()`,
  `display.height()` (for the current rotation), and
  `display.setBacklight(0..100)` to dim/brighten the backlight.
- **`lv_setup.displayDriver()`** — the same `display` object, if you'd rather
  reach it through `lv_setup`.
- **`lv_setup.onTouch`** — optional `void (*)()` hook, called on every touch poll while
  a finger is down. Handy for resetting a screensaver timer.
- **`board`** — the board object from
  [chipguy_JC8012P4A1C_board](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README.md): the LED
  (`board.setLed()`), the BOOT button, and **`board.Wire`**, the I²C bus that touch
  shares with the sound chip, the clock and the I²C connector. Use `board.Wire` for
  anything I²C — never `Wire.begin()`, which would take the bus away from touch.

You don't render or flush anything yourself — LVGL does, through the callbacks
`lv_setup` installed. Your job is to create LVGL widgets and call
`lv_timer_handler()` in `loop()`.

### Portable across chipguy board libraries

Because every hardware detail lives in just two files — **`lv_setup.hpp`** and
**`lv_conf.h`** — your application code is hardware-independent. It only ever
talks to LVGL. To move an app to a **different display board**, drop in the
`lv_setup.hpp` and `lv_conf.h` from that board's chipguy library, and the rest
of your sketch is unchanged. (Different boards have different resolutions, so
alignment-based layouts travel best; fixed pixel coordinates may need
adjusting.)

### Rotation

Pass `0`–`3` (or the equivalent degrees) to `lv_setup.begin()`. Display and
touch rotate together.

| rotation | degrees | orientation | resolution |
|---|---|---|---|
| 0 | 0 | portrait (native) | 800×1280 |
| 1 | 90 | landscape | 1280×800 |
| 2 | 180 | portrait flipped | 800×1280 |
| 3 | 270 | landscape flipped | 1280×800 |

So `0`/`2` are portrait and `1`/`3` are landscape, and `90`/`180`/`270` are
accepted as aliases for `1`/`2`/`3` (e.g. `lv_setup.begin(90)` is landscape).
`90` turns the picture clockwise, as in most Arduino display libraries: a board
mounted turned a quarter-turn clockwise wants `270`.
For the SquareLine example, size your SquareLine project to match the
orientation you choose.

Rotation is **free**: a hardware block in the ESP32-P4 turns each finished frame as it
copies it to the screen, so no processor time is spent on it, and the next frame is
drawn while the current one is shown. How that works, and what it costs in memory, is
in [README-engineering.md](README-engineering.md#rotation-the-ppa-pipeline).

## The rest of the board

### Sound

The board has a speaker connector with an amplifier, and a microphone, both on a sound
chip. This library doesn't play sound, but the board library's
**[README-sound.md](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README-sound.md)** describes the parts
and where to start, including a ready-made output class for MP3 playback.

### TF / microSD card slot

The board has a microSD (TF) slot on the ESP32-P4's dedicated 4-bit SD interface. This
library doesn't drive it and doesn't include a filesystem — but **it shares nothing
with the display**, so `SD_MMC` works alongside this library in either order. One
board-specific wrinkle: the card is powered from one of the P4's internal regulators,
which has to be named in the mount call or the card never powers up. No card-detect
switch. Pinout and copy-paste starting points are in the board library's
**[README-sdcard.md](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README-sdcard.md)**.

### Everything else

Buttons, the colour LED, the clock, the expansion connectors, the camera connector and
Wi-Fi are in the **[board library](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README.md)**; the
battery is in the **[power library](https://github.com/chipguyhere/JC8012P4A1C_power/blob/main/README.md)**.

---

## Where to go next

| File | For whom |
|---|---|
| [README-terminal.md](README-terminal.md) | Anyone who wants a scrolling text console instead of LVGL — including in larger text. |
| [README-raster.md](README-raster.md) | Developers drawing part of the screen themselves, in step with the panel: the API, the time each chunk has, and the rules. |
| [README-engineering.md](README-engineering.md) | Engineers: the MIPI-DSI bring-up, the panel-reset and retry strategy, the PPA rotation pipeline and its memory budget, the DMA scan list and the 2560-block test, how the terminal and the raster band work inside, the touch driver and its filtering, the backlight circuit. |
| [board library README](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README.md) | What else is on the board, and the I²C bus touch shares. |
| [firmware installer README](https://github.com/chipguyhere/ESP32-P4-C6-firmware-installer/blob/main/README.md) | Installing the touch firmware and the Wi-Fi chip's firmware, and why they are kept separate. |
| [board README-board.md](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README-board.md) | Developers: pin map, free pins, headers, USB ports. |
| [board README-sound.md](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README-sound.md) | Sound out and in. |
| [board README-sdcard.md](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README-sdcard.md) | Mounting the card. |
| [power library README](https://github.com/chipguyhere/JC8012P4A1C_power/blob/main/README.md) | The battery, sleep, and what the power button does. |
| [480display README](https://github.com/chipguyhere/JC8012P4A1C_480display/blob/main/README.md) | Running a 480×480 UI on this panel. |

---

## License

This library is **mixed-license**:

- The original code (display driver, touch driver, `lv_setup`, examples) is **MIT** — see
  [LICENSE](LICENSE).
- The vendored ESP-IDF derivatives — `esp_lcd_jd9365.c/.h` and `esp_lcd_panel_dpi_bb.c/.h`
  (Copyright © Espressif Systems) — are licensed under the
  **Apache License, Version 2.0**; see [LICENSE-Apache-2.0.txt](LICENSE-Apache-2.0.txt).
  Those files carry SPDX headers recording the upstream attribution and the modifications.

Nothing here is GPL. The touch controller's firmware, which is Silead's and GPL-licensed,
is not part of this library: the [firmware installer](https://github.com/chipguyhere/ESP32-P4-C6-firmware-installer/blob/main/README.md) puts it in the board's
flash once, and the touch driver reads it from there at power-up. Sketches built with this
library don't contain it.

The JD9365 initialization values are standard panel register settings. `lv_conf.h` is
based on LVGL's template (LVGL is MIT-licensed). The terminal's fonts are rendered from
Inconsolata and Noto Sans CJK (SIL OFL 1.1), DejaVu Sans Mono (Bitstream Vera license)
and Font Awesome Free (icons CC BY 4.0); their licenses are in
[extras/fontgen/](extras/fontgen/).
