# chipguy_JC8012P4A1C_display

A self-contained **display + touch** driver for the **Guition JC8012P4A1C** —
an **ESP32-P4** board with an **800×1280 JD9365** MIPI-DSI panel and a
**GSL3680** capacitive touch panel. It drives the display and the touch
controller so you don't have to think about either: the hardware is fixed and
fully handled, leaving you to write an **LVGL 9** app and nothing else.

The bundled examples are the heart of the library. Each is a complete, working
**starting point** — the display, touch, and LVGL are already wired up and
running before your code gets control. You pick the one that matches how you
want to build your UI, copy it, and start replacing its placeholder UI with your
own.

> This is for the **JC8012P4A1C** board (ESP32-P4, **800×1280 JD9365**
> MIPI-DSI, **GSL3680** capacitive touch). Boards with different controllers or
> resolutions need a different library.

This library is **LVGL-focused**: rotation is done in hardware by the ESP32-P4's
**PPA (Pixel Processing Accelerator)**, and the same UI code runs unchanged in
any of the four orientations.

---

## Installation

This library is distributed by downloading the code as a ZIP from GitHub —
there is no Arduino Library Manager entry. To install:

1. On the GitHub project page, click **Code ▸ Download ZIP**.
2. Unzip it, and move the resulting folder into your Arduino **libraries**
   directory:
   - macOS / Linux: `~/Documents/Arduino/libraries/`
   - Windows: `Documents\Arduino\libraries\`
3. Restart Arduino IDE so the examples will appear in the File menu.

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

- **PSRAM: OPI PSRAM** — **required.** The driver allocates its draw buffers and
  framebuffers in PSRAM; without it, `begin()` fails to allocate.
- **Flash Size: 16 MegaBytes (128 megabits)**
- **USB CDC On Boot: Enabled** — so the `Serial0` log output is visible.

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
For the SquareLine example, size your SquareLine project to match the
orientation you choose.

#### How rotation works (and why it's free)

LVGL always renders at the **native panel resolution** for the chosen
orientation into a full-screen draw buffer. On each finished frame the ESP32-P4
**PPA** rotates that buffer in hardware straight into one of two DSI
framebuffers, and the panel is flipped to show it. There is **no software
rotation and no per-pixel CPU work** — rotation costs nothing, and double
buffering means the next frame renders while the current one is shown.

This is the same hardware path as the companion `chipguy_10inchP4_480display`
library, but here it renders at the panel's full resolution with **rotation
only — no scaling**.

### Touch

The GSL3680 is a **capacitive** panel, so — unlike resistive boards — there is
**no calibration step** and no calibration example. Touch coordinates are read
in the panel's native frame and mapped to the active rotation automatically by
the driver (`display.mapTouch()`), so taps line up with what LVGL drew in every
orientation.

If on your unit the touch axes come out mirrored or swapped for a given
rotation, adjust the per-rotation mapping in
`chipguy_JC8012P4A1C_display::mapTouch()` (in
`src/chipguy_JC8012P4A1C_display.cpp`), or the base `mirror_x` / `mirror_y` /
`swap_xy` flags in `gsl3680_touch::begin()` (in `src/gsl3680_touch.cpp`).

---

## License

MIT — see [LICENSE](LICENSE).

The JD9365 panel driver, the GSL3680 touch driver, and the DPI panel driver are
based on Espressif's ESP-IDF drivers (Apache-2.0) with local modifications.
`lv_conf.h` is based on LVGL's template (LVGL is MIT-licensed).
