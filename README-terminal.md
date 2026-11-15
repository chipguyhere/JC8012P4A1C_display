# The text terminal

This is one of the two ways to drive the screen: a **text terminal**, like a computer's
console. You `print()` and `printf()`, and the text appears on a grid of character
cells and scrolls up the screen as it fills. The other way is [LVGL](README.md#the-examples--pick-your-starting-point),
which draws buttons, sliders and screens. Pick one per sketch: the terminal owns the
screen while it runs.

The terminal needs no other library and scrolls in hardware: the screen moves without
the text being drawn again. It's made for a **stream** of text — a log, a status feed,
a console. For readings that update in place, a dashboard in LVGL is usually the better
fit.

---

## Getting started

```cpp
#include <chipguy_JC8012P4A1C_terminal.h>

JC8012P4A1C_Board            board;            // the touch chip's I²C bus
chipguy_JC8012P4A1C_display  display(board);
chipguy_JC8012P4A1C_terminal term;

void setup() {
    term.begin(display);              // starts the screen; it clears to black
    term.println("Hello, JC8012!");
    term.printf("%d x %d cells\n", term.columns(), term.rows());
}

void loop() {
}
```

`term.begin()` starts the display itself, so don't call `lv_setup.begin()` or
`display.begin()` as well. The screen is **80 characters wide and exactly 64 lines
tall**. Each cell is 10 × 20 pixels. This panel has larger pixels than a phone's (about
150 per inch), so that is ordinary reading size.

## Turning it, and making the text bigger

`begin()` takes two more settings:

```cpp
term.begin(display, 90);          // landscape: 128 x 40 cells
term.begin(display, 0, 480);      // larger text: 48 x 38 cells
term.begin(display, 90, 480);     // both: 76 x 24 cells
```

- **Rotation** is 0, 90, 180 or 270, the picture turned clockwise — the same as
  `lv_setup.begin()` for LVGL. 0 and 180 are portrait, 90 and 270 landscape.
- **Resolution** is 800 (the screen's own) or 480. At 480 the terminal works on a
  480 × 760 screen and the board's graphics hardware stretches it to fill the panel
  (with a 10-pixel border), so every character is 1.6 times bigger — easier to read
  across a room.

Every combination works, and every one is a whole number of cells across and down.
They differ in what does the scrolling:

| Rotation, resolution | Cells | How it scrolls | Lines per second (measured): smooth, instant |
|---|---|---|---|
| 0 or 180, 800 | 80 × 64 | the display hardware reads the screen from a new place — nothing is copied | 117, 422 |
| 90 or 270, 800 | 128 × 40 | the graphics hardware (PPA) copies and turns the screen each update | 66, 152 |
| 0 or 180, 480 | 48 × 38 | the PPA copies and stretches the screen each update | 102, 304 |
| 90 or 270, 480 | 76 × 24 | the PPA turns and stretches it | 98, 302 |

The numbers are for text pouring in as fast as possible. Smooth scrolling is held to
about 100 lines a second on purpose, so you can still see the text move
(`setSmoothScrollLimit()` changes that). A line printed now and then scrolls in about a
tenth of a second. Landscape at 800 is the slowest: turning the whole 2.5-megabyte
screen takes the PPA about 47 ms, more than two frames —
[the engineering notes](README-engineering.md#the-text-terminal) have the measurements.

## The examples

Open them from **File ▸ Examples ▸ chipguy_JC8012P4A1C_display**. Each has
`TERM_ROTATION` and `TERM_RESOLUTION` near the top to try the other settings.

| Example | What it shows |
|---|---|
| **TerminalHello** | The basics: colors, `printf`, tabs, symbols, box drawing, the cursor styles. Then it counts upward, faster and faster. **Start your project from this one.** |
| **TerminalScrollSpeed** | How fast it goes: bursts of a pretend log, too fast to read, with the speed shown after each. |
| **TerminalTouchButtons** | Buttons made of colored text that you tap; they light the board's LED and dim the screen. |
| **TerminalHeaderFooter** | A picture at the top and bottom of the screen that stays put while text scrolls between them. |
| **TerminalSymbols** | Every LVGL symbol the terminal can print, with its name. |
| **TerminalMatrixRain** | The green "digital rain" from *The Matrix*, in half-width katakana. |

---

## Printing

`print()`, `println()`, `printf()` and `putc()` all work as they do with `Serial`.
Text is **UTF-8**, so you can write `"21.5°C"` or `"±5%"` straight into your sketch.

`printf()` never uses the heap. Width and padding work as usual —
`printf("%5d", n)`, `printf("%-8s", name)`, `printf("%06.2f", v)` — and for strings the
width counts screen cells, so columns line up even with characters that take two cells.
One limit: a single number longer than 47 characters (say `"%.60f"`) is cut short.

### Special characters

| Character | In a string | What it does |
|---|---|---|
| New line | `\n` | Go to the start of the next line, scrolling if needed. |
| Return | `\r` | Go back to the start of this line (handy for a counter that rewrites itself). |
| Tab | `\t` | Print spaces up to the next column that's a multiple of 4. |
| Backspace | `\b` | Move back one cell (doesn't erase). |
| Clear to end of line | `\v`, or `JC8012T_CLEAR_EOL_STR` | Erase from the cursor to the end of the line, in the current background color. |
| Clear screen | `\f` | Same as `clear()`. |

### Lines that fill the screen exactly

When text fills the last cell of a line, the cursor waits just past the right edge
instead of jumping to the next line. It only moves on when you print something more.
So a line exactly as wide as the screen, followed by `\n`, takes one line, not two.
`setImmediateWrap(true)` switches back to jumping at once.

## Colors

```cpp
term.setTextColor(JC8012T_YELLOW);                  // text color
term.setTextColor(JC8012T_BLACK, JC8012T_CYAN);     // text and background
term.setTextColor(jc8012t_rgb565(255, 128, 0));     // any color
```

Ready-made colors: `JC8012T_BLACK`, `WHITE`, `GREY`, `RED`, `GREEN`, `BLUE`, `YELLOW`,
`CYAN`, `MAGENTA`, `ORANGE`. Colors are standard RGB565 numbers, the same as in LVGL.

`clear()` fills the screen with the current **background** color and puts the cursor at
the top left.

## Where the text goes

Columns and lines are counted from 0 at the top left. `setCursor(col, row)` moves
there; `cursorCol()` and `cursorRow()` say where the cursor is; `columns()` and
`rows()` say how big the grid is.

**Row 0 is always the top line you can see** — even when only part of it is showing,
which can happen after scrolling back, or with a frozen strip (below) that isn't a
whole number of lines. `evictPartialLine()` scrolls on just far enough to push a cut-off
top line out of sight. `evictPartialLine(true)` scrolls back instead, pushing a cut-off
bottom line out and bringing blank space in at the top.

### The cursor

`setCursorStyle(JC8012T_CURSOR_BLOCK)` (the default), `JC8012T_CURSOR_UNDERLINE`,
`JC8012T_CURSOR_BAR` (a thin line at the left of the cell) or `JC8012T_CURSOR_OFF`. It
blinks by itself, in the current text color. You can move it onto text printed earlier
and it won't damage it: it remembers what was under it.

## Scrolling

When the screen is full, each new line slides everything up. `setScrollMode()` picks how:

- **`JC8012T_SCROLL_SMOOTH`** (the default) — the text glides up, a few pixels each
  frame, and speeds up when it falls behind, up to the limit set by
  `setSmoothScrollLimit()` (100 lines a second; 0 for no limit).
- **`JC8012T_SCROLL_INSTANT`** — each line jumps into place. Faster when text pours in.

Scrolling never flashes or shows half-drawn lines: each new line is drawn out of sight
first, then scrolled in.

### When print() waits

`print()` normally returns at once. The terminal keeps 16 lines of room below the
screen to draw into, so your sketch only waits when it gets more than 16 lines
ahead of the scrolling. `flush()` waits until everything printed so far is on the
screen and the scrolling has stopped.

### Settings to call before begin()

| Call | Default | What it changes |
|---|---|---|
| `term.setSpareLines(n)` | 16 | The room below the screen, 1 to 64 lines. More lets bursts of text scroll faster in instant mode and in the PPA settings, at 64–100 KB of PSRAM per line. |
| `term.setPpaFramebuffers(n)` | 3 | 2 saves 2 MB of memory in the PPA settings, and they update more slowly. |
| `term.setTask(priority, core)` | 5, either core | The priority and core of the terminal's background task. |

`setScrollMode()` and `setSmoothScrollLimit()` can be changed at any time.

## A header and footer that stay put

`freeze(topPixels, bottomPixels)` takes strips at the top and bottom of the screen out
of the scrolling. Text scrolls between them, and you draw what you like into them with
`drawBitmap()`. `freeze(0, 0)` undoes it. **What's on the screen stays where it is** —
including any text that was in the strips, so draw over all of each strip.
**TerminalHeaderFooter** shows how.

```cpp
term.freeze(40, 40);                                     // 40 px header and footer
term.drawBitmap(0, 0, 800, 40, headerPixels);            // into the header
term.drawBitmap(0, term.height() - 40, 800, 40, footerPixels);
```

`drawBitmap(x, y, w, h, pixels)` draws a picture at a position on the screen, counted in
pixels from the top left. `pixels` is `w × h` RGB565 colors, row by row — the format
most image-to-C converters produce for "RGB565". Parts off the screen are cut off.
It's fine to keep drawing into the frozen strips while text scrolls; a picture drawn
onto the scrolling part scrolls away with the text.

Make each strip a multiple of 20 pixels (one line) and the text area stays a whole
number of lines. At resolution 480 a strip must also be a multiple of 8, and
`freeze()` rounds it up; multiples of **40** suit every setting.

## Touch

The display starts the touch screen itself, so there is nothing to set up:

```cpp
int16_t col, row;
if (term.pollTouch(col, row)) {
    // the finger is on cell (col, row): same numbers as setCursor()
}
```

`pollTouch()` answers in character cells, so "was my button touched?" is a simple
comparison — see **TerminalTouchButtons**. A 20-pixel line is a small target for a
finger, so its buttons are three lines tall. Touches on a frozen header or footer don't
count in cells; ask for pixels instead with `term.pollTouch(x, y, JC8012T_TOUCH_PIXELS)`.
Rotation and resolution are already applied, and at 480 a touch on the border around the
picture is ignored. The touch screen needs no calibration.

## Symbols and fonts

The built-in font has:

- English and the accented letters of Western, Central and Eastern European languages;
- Greek, with the accents of modern Greek;
- symbols common in projects: `° ± µ Ω ℃ ≤ ≥ ≈ √ ∞ ← ↑ → ↓ ✓ ✗ ⚠ ⚡ ★ ♥`, fractions,
  superscripts and subscripts (`CO₂`, `m²`), currency (`€ £ ¥`);
- **box drawing** (`┌─┐ │ └─┘ ═ ║ ╔ ╗`), drawn so lines meet exactly between cells,
  and block shapes (`█ ▌ ▐ ▀ ▄ ░ ▒ ▓`);
- **LVGL's symbols**, under LVGL's own names: `term.print(LV_SYMBOL_WIFI)` shows the
  same Wi-Fi icon an LVGL label would, one cell wide. **TerminalSymbols** shows them all.

The letters are drawn from **Inconsolata**, a free font in the style of Consolas; the
symbols from **DejaVu Sans Mono** and **Font Awesome Free**. A character the font
doesn't have prints as `�`. The font takes about 75 KB of flash.

**Japanese kana** come in a separate font you choose to include, about 36 KB:

```cpp
#include <jc8012t_font_mono10_ja_kana.h>
#include <chipguy_JC8012P4A1C_terminal.h>
// ...
term.begin(display, 0, 800, &jc8012t_font_mono10_ja_kana);
```

Hiragana and katakana take two cells; half-width katakana (ｶﾀｶﾅ) take one, like Latin
letters. Chinese, Japanese kanji and Korean fonts can be built with the font generator in
this library's `extras/fontgen/` folder; they are too large (0.5–4 MB of source each) to
ship here.

## Good to know

- **The terminal owns the screen.** Once it's running, don't use LVGL or draw on the
  display yourself; draw pictures with `term.drawBitmap()`. `display.setBacklight()` is
  fine. The display's `sleepUntil…()` functions have not been tried with the terminal.
- **It uses one background task**, which scrolls, blinks the cursor and runs the
  graphics hardware. Printing from more than one of your own tasks is safe; each
  `print()` or `printf()` comes out whole.
- **Memory:** the screen and its 16 spare lines live in PSRAM, of which the board has
  32 MB: about 5 MB in portrait at 800, and up to about 12 MB with the graphics
  hardware (landscape at 800). Internal RAM: about 10 KB, all reserved when the sketch
  starts.

---

## Where to go next

- Back to the [README](README.md) for LVGL and the rest of the board.
- [README-engineering.md](README-engineering.md#the-text-terminal) — how the terminal
  works inside: the display DMA's four-piece screen, the scroll ring, the graphics
  hardware path, and the measurements.
