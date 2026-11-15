/*
 * Console-style text terminal for the Guition JC8012P4A1C -- the alternative
 * to LVGL.  print(), printf() and putc() onto a grid of 10 x 20 pixel
 * character cells -- exactly 80 x 64 on the 800 x 1280 screen --
 * scrolled in HARDWARE: a screenful of text moves without being redrawn.
 *
 *   #include <chipguy_JC8012P4A1C_terminal.h>
 *   JC8012P4A1C_Board            board;            // the touch chip's I2C bus
 *   chipguy_JC8012P4A1C_display  display(board);
 *   chipguy_JC8012P4A1C_terminal term;
 *   void setup() { term.begin(display); term.println("Hello"); }
 *
 * Two ways to run, chosen by begin()'s rotation and resolution:
 *
 *   800 x 1280, rotation 0 or 180 -- DIRECT.  Text is drawn straight into the
 *     panel's framebuffer, and scrolling only changes where the display DMA
 *     starts reading each frame: a frozen top band, the scrolling window
 *     (from a ring of rows, in two pieces where it wraps), a frozen bottom
 *     band.  No pixel is copied to scroll.  80 columns x 64 lines.
 *
 *   rotation 90 or 270, or resolution 480 -- PPA.  The terminal draws into
 *     an upright buffer of its own, and the ESP32-P4's PPA (pixel-processing
 *     accelerator) rotates it -- and for 480 scales it 1.625x into a centred
 *     780 x 1235 -- into the framebuffer, then the frame is flipped in.
 *     Scrolling is still a change of where the PPA reads, not a copy of the
 *     text.  The PPA runs only when something on screen changed.  1280 x 800
 *     gives 128 columns x 40 lines; 480 x 760 gives 48 x 38 of larger text,
 *     and 760 x 480 gives 76 x 24.  The 10 x 20 cell divides every one of
 *     these screens exactly.  (The panel is about 150 dpi, so 10 x 20 is
 *     about the physical size of the P4-86 panel's 16 x 30.)
 *
 * Either way the scroll ring holds setSpareLines() lines more than the
 * screen, so a new line is cleared and drawn before it scrolls into view:
 * scrolling never flashes.  print() returns at once unless it gets more than
 * that many lines ahead of the scrolling.
 *
 * The terminal owns the screen: start it INSTEAD of lv_setup / LVGL, and let
 * begin() start the display (it needs a taller framebuffer than LVGL does, and
 * no LVGL draw buffers).  display.setBacklight() is fine.  (The display's
 * sleepUntil...() functions have not been tried with the terminal.)
 *
 * Coordinates: columns and rows count character cells from the top-left of
 * the scrolling (unfrozen) area.  Row 0 is always the top line on screen,
 * even when only part of it shows; see evictPartialLine().  drawBitmap() and
 * pollTouch(..., JC8012T_TOUCH_PIXELS) use screen pixels (0..width()-1,
 * 0..height()-1).
 *
 * Control characters:
 *   '\n'  new line (column 0 of the next line, scrolling if needed)
 *   '\r'  column 0 of this line
 *   '\t'  spaces up to the next column that is a multiple of 4
 *   '\b'  cursor one cell left (does not erase)
 *   '\v'  (Ctrl-K) clear to the end of the line; also JC8012T_CLEAR_EOL
 *   '\f'  (Ctrl-L) clear the screen, as clear()
 * Other control characters are ignored.  Text is UTF-8; code points above
 * U+FFFF and characters missing from the font print as U+FFFD.
 *
 * The front end -- printing, printf, the fonts, the API -- comes from the
 * terminal in chipguy_ESP32-CYD-Resistive_display; the drawing and scrolling
 * underneath come from chipguy_P486Panel_display's.  See README-terminal.md.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License -- see LICENSE.  Fonts: see extras/fontgen/.
 */

#pragma once

#ifdef ARDUINO_ESP32P4_DEV

#include <Arduino.h>
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "driver/ppa.h"
#include "chipguy_JC8012P4A1C_display.h"

// ── Tuning ─────────────────────────────────────────────────────────────────
// Set at run time, before begin(): see setSpareLines(), setPpaFramebuffers()
// and setTask() in the class.  (They were once #defines, but a #define in a
// sketch never reaches the library's own .cpp.)
#define JC8012T_SPARE_LINES_DEFAULT 16
#define JC8012T_STACK_BYTES 4096        // the terminal task's stack; see stackHighWater()
#define JC8012T_PRINTF_NUMBUF 48        // printf(): the longest single number, in characters

// Cells are 10 pixels wide (double-width characters 20).  The built-in font
// is 20 tall; a font may be up to JC8012T_MAX_CELL_H tall.
#define JC8012T_CELL_W      10
#define JC8012T_MAX_CELL_H  32
#define JC8012T_MAX_COLS    (JC8012_NATIVE_HEIGHT / JC8012T_CELL_W)

#define JC8012T_CLEAR_EOL     '\v'
#define JC8012T_CLEAR_EOL_STR "\v"

// ── LVGL's symbols ─────────────────────────────────────────────────────────
// The built-in font carries LVGL's symbol set at the code points LVGL uses,
// rendered from Font Awesome Free (icons CC BY 4.0 by Fonticons, Inc.), and
// the names below are LVGL's own (from lv_symbol_def.h, LVGL, MIT license).
// So a line written for the terminal moves to an LVGL label unchanged, and
// back:
//   term.print(LV_SYMBOL_WIFI " connected");
// Each is guarded as LVGL guards its own, so lvgl.h may be included before or
// after this header.  Each prints in one cell.  The TerminalSymbols example
// shows them all with their names.
#ifndef LV_SYMBOL_BULLET
#define LV_SYMBOL_BULLET        "\xE2\x80\xA2"   // U+2022
#endif
#ifndef LV_SYMBOL_AUDIO
#define LV_SYMBOL_AUDIO         "\xEF\x80\x81"   // U+F001
#endif
#ifndef LV_SYMBOL_VIDEO
#define LV_SYMBOL_VIDEO         "\xEF\x80\x88"   // U+F008
#endif
#ifndef LV_SYMBOL_LIST
#define LV_SYMBOL_LIST          "\xEF\x80\x8B"   // U+F00B
#endif
#ifndef LV_SYMBOL_OK
#define LV_SYMBOL_OK            "\xEF\x80\x8C"   // U+F00C
#endif
#ifndef LV_SYMBOL_CLOSE
#define LV_SYMBOL_CLOSE         "\xEF\x80\x8D"   // U+F00D
#endif
#ifndef LV_SYMBOL_POWER
#define LV_SYMBOL_POWER         "\xEF\x80\x91"   // U+F011
#endif
#ifndef LV_SYMBOL_SETTINGS
#define LV_SYMBOL_SETTINGS      "\xEF\x80\x93"   // U+F013
#endif
#ifndef LV_SYMBOL_HOME
#define LV_SYMBOL_HOME          "\xEF\x80\x95"   // U+F015
#endif
#ifndef LV_SYMBOL_DOWNLOAD
#define LV_SYMBOL_DOWNLOAD      "\xEF\x80\x99"   // U+F019
#endif
#ifndef LV_SYMBOL_DRIVE
#define LV_SYMBOL_DRIVE         "\xEF\x80\x9C"   // U+F01C
#endif
#ifndef LV_SYMBOL_REFRESH
#define LV_SYMBOL_REFRESH       "\xEF\x80\xA1"   // U+F021
#endif
#ifndef LV_SYMBOL_MUTE
#define LV_SYMBOL_MUTE          "\xEF\x80\xA6"   // U+F026
#endif
#ifndef LV_SYMBOL_VOLUME_MID
#define LV_SYMBOL_VOLUME_MID    "\xEF\x80\xA7"   // U+F027
#endif
#ifndef LV_SYMBOL_VOLUME_MAX
#define LV_SYMBOL_VOLUME_MAX    "\xEF\x80\xA8"   // U+F028
#endif
#ifndef LV_SYMBOL_IMAGE
#define LV_SYMBOL_IMAGE         "\xEF\x80\xBE"   // U+F03E
#endif
#ifndef LV_SYMBOL_TINT
#define LV_SYMBOL_TINT          "\xEF\x81\x83"   // U+F043
#endif
#ifndef LV_SYMBOL_PREV
#define LV_SYMBOL_PREV          "\xEF\x81\x88"   // U+F048
#endif
#ifndef LV_SYMBOL_PLAY
#define LV_SYMBOL_PLAY          "\xEF\x81\x8B"   // U+F04B
#endif
#ifndef LV_SYMBOL_PAUSE
#define LV_SYMBOL_PAUSE         "\xEF\x81\x8C"   // U+F04C
#endif
#ifndef LV_SYMBOL_STOP
#define LV_SYMBOL_STOP          "\xEF\x81\x8D"   // U+F04D
#endif
#ifndef LV_SYMBOL_NEXT
#define LV_SYMBOL_NEXT          "\xEF\x81\x91"   // U+F051
#endif
#ifndef LV_SYMBOL_EJECT
#define LV_SYMBOL_EJECT         "\xEF\x81\x92"   // U+F052
#endif
#ifndef LV_SYMBOL_LEFT
#define LV_SYMBOL_LEFT          "\xEF\x81\x93"   // U+F053
#endif
#ifndef LV_SYMBOL_RIGHT
#define LV_SYMBOL_RIGHT         "\xEF\x81\x94"   // U+F054
#endif
#ifndef LV_SYMBOL_PLUS
#define LV_SYMBOL_PLUS          "\xEF\x81\xA7"   // U+F067
#endif
#ifndef LV_SYMBOL_MINUS
#define LV_SYMBOL_MINUS         "\xEF\x81\xA8"   // U+F068
#endif
#ifndef LV_SYMBOL_EYE_OPEN
#define LV_SYMBOL_EYE_OPEN      "\xEF\x81\xAE"   // U+F06E
#endif
#ifndef LV_SYMBOL_EYE_CLOSE
#define LV_SYMBOL_EYE_CLOSE     "\xEF\x81\xB0"   // U+F070
#endif
#ifndef LV_SYMBOL_WARNING
#define LV_SYMBOL_WARNING       "\xEF\x81\xB1"   // U+F071
#endif
#ifndef LV_SYMBOL_SHUFFLE
#define LV_SYMBOL_SHUFFLE       "\xEF\x81\xB4"   // U+F074
#endif
#ifndef LV_SYMBOL_UP
#define LV_SYMBOL_UP            "\xEF\x81\xB7"   // U+F077
#endif
#ifndef LV_SYMBOL_DOWN
#define LV_SYMBOL_DOWN          "\xEF\x81\xB8"   // U+F078
#endif
#ifndef LV_SYMBOL_LOOP
#define LV_SYMBOL_LOOP          "\xEF\x81\xB9"   // U+F079
#endif
#ifndef LV_SYMBOL_DIRECTORY
#define LV_SYMBOL_DIRECTORY     "\xEF\x81\xBB"   // U+F07B
#endif
#ifndef LV_SYMBOL_UPLOAD
#define LV_SYMBOL_UPLOAD        "\xEF\x82\x93"   // U+F093
#endif
#ifndef LV_SYMBOL_CALL
#define LV_SYMBOL_CALL          "\xEF\x82\x95"   // U+F095
#endif
#ifndef LV_SYMBOL_CUT
#define LV_SYMBOL_CUT           "\xEF\x83\x84"   // U+F0C4
#endif
#ifndef LV_SYMBOL_COPY
#define LV_SYMBOL_COPY          "\xEF\x83\x85"   // U+F0C5
#endif
#ifndef LV_SYMBOL_SAVE
#define LV_SYMBOL_SAVE          "\xEF\x83\x87"   // U+F0C7
#endif
#ifndef LV_SYMBOL_BARS
#define LV_SYMBOL_BARS          "\xEF\x83\x89"   // U+F0C9
#endif
#ifndef LV_SYMBOL_ENVELOPE
#define LV_SYMBOL_ENVELOPE      "\xEF\x83\xA0"   // U+F0E0
#endif
#ifndef LV_SYMBOL_CHARGE
#define LV_SYMBOL_CHARGE        "\xEF\x83\xA7"   // U+F0E7
#endif
#ifndef LV_SYMBOL_PASTE
#define LV_SYMBOL_PASTE         "\xEF\x83\xAA"   // U+F0EA
#endif
#ifndef LV_SYMBOL_BELL
#define LV_SYMBOL_BELL          "\xEF\x83\xB3"   // U+F0F3
#endif
#ifndef LV_SYMBOL_KEYBOARD
#define LV_SYMBOL_KEYBOARD      "\xEF\x84\x9C"   // U+F11C
#endif
#ifndef LV_SYMBOL_GPS
#define LV_SYMBOL_GPS           "\xEF\x84\xA4"   // U+F124
#endif
#ifndef LV_SYMBOL_FILE
#define LV_SYMBOL_FILE          "\xEF\x85\x9B"   // U+F15B
#endif
#ifndef LV_SYMBOL_WIFI
#define LV_SYMBOL_WIFI          "\xEF\x87\xAB"   // U+F1EB
#endif
#ifndef LV_SYMBOL_BATTERY_FULL
#define LV_SYMBOL_BATTERY_FULL  "\xEF\x89\x80"   // U+F240
#endif
#ifndef LV_SYMBOL_BATTERY_3
#define LV_SYMBOL_BATTERY_3     "\xEF\x89\x81"   // U+F241
#endif
#ifndef LV_SYMBOL_BATTERY_2
#define LV_SYMBOL_BATTERY_2     "\xEF\x89\x82"   // U+F242
#endif
#ifndef LV_SYMBOL_BATTERY_1
#define LV_SYMBOL_BATTERY_1     "\xEF\x89\x83"   // U+F243
#endif
#ifndef LV_SYMBOL_BATTERY_EMPTY
#define LV_SYMBOL_BATTERY_EMPTY "\xEF\x89\x84"   // U+F244
#endif
#ifndef LV_SYMBOL_USB
#define LV_SYMBOL_USB           "\xEF\x8A\x87"   // U+F287
#endif
#ifndef LV_SYMBOL_BLUETOOTH
#define LV_SYMBOL_BLUETOOTH     "\xEF\x8A\x93"   // U+F293
#endif
#ifndef LV_SYMBOL_TRASH
#define LV_SYMBOL_TRASH         "\xEF\x8B\xAD"   // U+F2ED
#endif
#ifndef LV_SYMBOL_EDIT
#define LV_SYMBOL_EDIT          "\xEF\x8C\x84"   // U+F304
#endif
#ifndef LV_SYMBOL_BACKSPACE
#define LV_SYMBOL_BACKSPACE     "\xEF\x95\x9A"   // U+F55A
#endif
#ifndef LV_SYMBOL_SD_CARD
#define LV_SYMBOL_SD_CARD       "\xEF\x9F\x82"   // U+F7C2
#endif
#ifndef LV_SYMBOL_NEW_LINE
#define LV_SYMBOL_NEW_LINE      "\xEF\xA2\xA2"   // U+F8A2
#endif

// ── Fonts ──────────────────────────────────────────────────────────────────
// A font is a sorted table of code points with 4-bit greyscale glyphs,
// run-length coded; see extras/fontgen/make_terminal_fonts.py for the format.
// Glyphs are JC8012T_CELL_W wide, or twice that for double-width (CJK)
// characters.  A code point not in a font is looked up in its fallback.
struct jc8012t_font_t {
    uint8_t  cellW, cellH;
    uint16_t count;
    const uint16_t *codepoints;   // sorted, `count` entries
    const uint32_t *offsets;      // count + 1 entries; bit 31 = double width
    const uint8_t  *data;
    const jc8012t_font_t *fallback;
};

// The built-in font: Inconsolata for Latin, with accents and Greek; DejaVu
// Sans Mono for arrows, math, shapes and other symbols; box drawing and block
// elements drawn to meet at cell edges; LVGL's symbols from Font Awesome
// Free.  10 x 20 pixel cells.  About 75 KB of flash.
extern const jc8012t_font_t jc8012t_font_mono10;

// OPT-IN: Japanese kana (hiragana, katakana, half-width katakana), in its own
// header so a sketch pays only if it includes it -- jc8012t_font_mono10_ja_kana.h,
// about 36 KB.  The CYD library's Chinese, Japanese-kanji and Korean fonts use
// the same format; make_terminal_fonts.py in extras/fontgen/ builds them.

// ── Colors (RGB565) ────────────────────────────────────────────────────────
static constexpr uint16_t jc8012t_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
static constexpr uint16_t JC8012T_BLACK   = 0x0000;
static constexpr uint16_t JC8012T_WHITE   = 0xFFFF;
static constexpr uint16_t JC8012T_GREY    = jc8012t_rgb565(128, 128, 128);
static constexpr uint16_t JC8012T_RED     = 0xF800;
static constexpr uint16_t JC8012T_GREEN   = 0x07E0;
static constexpr uint16_t JC8012T_BLUE    = 0x001F;
static constexpr uint16_t JC8012T_YELLOW  = 0xFFE0;
static constexpr uint16_t JC8012T_CYAN    = 0x07FF;
static constexpr uint16_t JC8012T_MAGENTA = 0xF81F;
static constexpr uint16_t JC8012T_ORANGE  = jc8012t_rgb565(255, 165, 0);

enum jc8012t_cursor_t : uint8_t {
    JC8012T_CURSOR_OFF = 0,
    JC8012T_CURSOR_BLOCK,       // the whole cell, solid in the text color
    JC8012T_CURSOR_UNDERLINE,   // a bar along the bottom of the cell
    JC8012T_CURSOR_BAR,         // a thin vertical line at the cell's left edge
};

enum jc8012t_scroll_mode_t : uint8_t {
    JC8012T_SCROLL_SMOOTH = 0,  // slide, at least 4 pixel rows a frame, faster
                              // the further behind it is
    JC8012T_SCROLL_INSTANT,     // jump a whole line at once
};

enum jc8012t_touch_units_t : uint8_t {
    JC8012T_TOUCH_CELLS = 0,    // column / row; touches on frozen areas are ignored
    JC8012T_TOUCH_PIXELS,       // screen pixels, frozen areas included
};

// The formatter behind printf(), separate from the terminal so it can be
// tested against the C library's own printf.
struct jc8012t_fmt_sink {
    void (*bytes)(void *ctx, const char *s, size_t n);
    void (*codepoint)(void *ctx, uint32_t cp);
    bool (*wide)(void *ctx, uint16_t cp);
    void *ctx;
};
size_t jc8012t_term_vformat(const jc8012t_fmt_sink &sink, const char *fmt, va_list ap);

class chipguy_JC8012P4A1C_terminal : public Print {
public:
    chipguy_JC8012P4A1C_terminal() {}

    // Start the display and the terminal.  rotation: 0 / 90 / 180 / 270 (or
    // 1 / 2 / 3), the picture turned clockwise as lv_setup.begin() does: 0 and
    // 180 portrait, 90 and 270 landscape.  resolution: 800 (native: 800 x 1280
    // or 1280 x 800, cells of 10 x 20) or 480 (480 x 760 or 760 x 480, which
    // the PPA scales up 1.625x into a centred 780 x 1235: larger text).  The
    // screen clears to black with white text and the cursor at (0, 0).
    // Returns false if the display or a buffer could not be set up.  Call it
    // INSTEAD of lv_setup.begin() / display.begin().
    bool begin(chipguy_JC8012P4A1C_display &display, uint16_t rotation = 0,
               uint16_t resolution = 800, const jc8012t_font_t *font = &jc8012t_font_mono10);

    // ── Text ──────────────────────────────────────────────────────────────
    size_t write(uint8_t b) override;
    size_t write(const uint8_t *buf, size_t n) override;
    using Print::write;

    int putc(int c) { write((uint8_t)c); return c; }
    void putCodepoint(uint32_t cp);

    // printf without the heap.  Width counts screen cells, so a CJK character
    // counts 2.  A single number longer than JC8012T_PRINTF_NUMBUF is cut short.
    size_t printf(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
    size_t vprintf(const char *fmt, va_list ap);

    // Clear the scrolling area to the current background color and put the
    // cursor at (0, 0).  includeFrozen also clears the frozen header / footer.
    void clear(bool includeFrozen = false);
    void clearToEndOfLine();

    // Block until everything printed so far is on screen and scrolling has
    // stopped.
    void flush() override;

    // ── Cursor and colors ─────────────────────────────────────────────────
    void setCursor(int16_t col, int16_t row);
    int16_t cursorCol() const { return _curCol; }
    int16_t cursorRow() const;

    void setTextColor(uint16_t fg) { _fg = fg; }
    void setTextColor(uint16_t fg, uint16_t bg) { _fg = fg; _bg = bg; }
    uint16_t textColor() const { return _fg; }
    uint16_t backgroundColor() const { return _bg; }

    // Blinking cursor, in the current text color.  It saves the pixels it
    // covers and puts them back, so it never damages old text.
    void setCursorStyle(jc8012t_cursor_t style);

    // false (default): filling the last cell parks the cursor just past the
    // edge until something more is printed, so a full-width line followed by
    // '\n' takes one line.  true: move to the next line at once.
    void setImmediateWrap(bool wrapNow) { _immediateWrap = wrapNow; }

    void setScrollMode(jc8012t_scroll_mode_t mode) { _scrollMode = mode; }

    // The most lines a second smooth scrolling moves (default 100), so that
    // text pouring in is still seen to scroll; 0 for no limit.  Instant
    // scrolling is never limited.
    void setSmoothScrollLimit(uint16_t linesPerSecond) { _smoothLimit = linesPerSecond; }

    // Pixel rows scrolled since begin() (wraps at 2^32): for measuring.
    uint32_t pixelsScrolled() const { return _scrolled; }

    // ── Geometry ──────────────────────────────────────────────────────────
    int16_t columns() const { return _cols; }
    int16_t rows() const;                 // rows on screen, a partial one included
    int16_t width() const { return _W; }  // screen pixels, as rotated
    int16_t height() const { return _H; }
    uint8_t cellWidth() const { return JC8012T_CELL_W; }
    uint8_t cellHeight() const { return _CH; }
    uint16_t rotation() const { return _rot * 90; }
    bool usesPPA() const { return _ppa; }

    // Freeze `topPixels` at the top and `bottomPixels` at the bottom: they stop
    // scrolling, so a header or footer drawn there with drawBitmap() stays put.
    // freeze(0, 0) unfreezes.  At least one text line must remain.  What is on
    // screen stays where it is.  At resolution 480 sizes are rounded up to
    // a multiple of 8 (the 1.625x scale needs whole pixels).  Returns false if
    // they don't fit.
    bool freeze(uint16_t topPixels, uint16_t bottomPixels);
    uint16_t frozenTop() const { return _fixT; }
    uint16_t frozenBottom() const { return _fixB; }

    // After scrolling, or a freeze() that isn't a multiple of the cell height,
    // the top line on screen can be partly cut off.  This scrolls forward just
    // far enough to push it off; backward = true scrolls back instead, pushing
    // the partial bottom line off and bringing blank space in at the top.
    void evictPartialLine(bool backward = false);

    // ── Graphics ──────────────────────────────────────────────────────────
    // Draw a w x h RGB565 image (row by row; 0xF800 is red) at screen position
    // (x, y).  Clipped.  In the frozen areas it stays put; over the scrolling
    // area it lands where (x, y) is on screen now and scrolls with the text.
    // Blocks until drawn, so `pixels` may be on the caller's stack.
    void drawBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pixels);

    // ── Touch ─────────────────────────────────────────────────────────────
    // The display's GSL3680, which display.begin() starts.
    // JC8012T_TOUCH_CELLS (default): x = column, y = row, as setCursor(); false
    // for no touch and for touches on the frozen areas.  JC8012T_TOUCH_PIXELS:
    // screen pixels, rotation and resolution applied.
    bool pollTouch(int16_t &x, int16_t &y, jc8012t_touch_units_t units = JC8012T_TOUCH_CELLS);

    // ── Misc ──────────────────────────────────────────────────────────────
    // ── Before begin() ────────────────────────────────────────────────────
    // Lines the scroll ring holds beyond the screen (default 16, at most 64).
    // print() may run this many lines ahead of the scrolling before it
    // waits, and in the PPA modes the PPA composes one picture while print()
    // fills the next lines.  More spare lines let a burst scroll faster: each
    // screen update moves a quarter of the way to the newest line.  Costs
    // that many lines of PSRAM in each framebuffer (direct mode) or twice in
    // the PPA surface.
    void setSpareLines(uint8_t lines) { _spareLines = lines < 1 ? 1 : lines > 64 ? 64 : lines; }
    // Framebuffers in the PPA modes: 3 (default) lets the next picture start
    // while the last is still waiting for a frame boundary to be shown; 2
    // saves a framebuffer of PSRAM.  Direct mode always uses 2.
    void setPpaFramebuffers(uint8_t n) { _ppaFbs = n >= 3 ? 3 : 2; }
    // The task that scrolls, blinks the cursor and runs the PPA (default:
    // priority 5, either core).
    void setTask(UBaseType_t priority, BaseType_t core = tskNO_AFFINITY) { _taskPrio = priority; _taskCore = core; }

    bool setFont(const jc8012t_font_t *font);   // same cell size only
    uint32_t stackHighWater() const;
    // How long the last PPA update took, in microseconds (0 in direct mode),
    // and how many there have been.
    uint32_t composeMicros() const { return _composeUs; }
    uint32_t composeCount() const { return _composeN; }

private:
    static constexpr uint16_t CELL_KEEP   = 0xFFFF;   // unknown contents
    static constexpr uint16_t CELL_WIDE_R = 0xFFFE;   // right half of the cell to its left
    enum Band : uint8_t { TOP = 0, RING = 1, BOTTOM = 2 };

    // ── Static storage ────────────────────────────────────────────────────
    uint16_t _cells[JC8012T_MAX_COLS];        // the cursor line's characters
    uint16_t _curSave[2 * JC8012T_CELL_W * JC8012T_MAX_CELL_H];   // pixels under the cursor
    uint16_t _glyphBuf[2 * JC8012T_CELL_W * JC8012T_MAX_CELL_H];  // one decoded glyph
    StackType_t  _stack[JC8012T_STACK_BYTES / sizeof(StackType_t)];
    StaticTask_t _tcb;
    StaticSemaphore_t _mutexBuf, _drawBuf, _progBuf;

    // ── Fixed after begin() ───────────────────────────────────────────────
    chipguy_JC8012P4A1C_display *_disp = nullptr;
    TaskHandle_t _task = nullptr;
    SemaphoreHandle_t _mutex = nullptr;     // serializes callers (recursive)
    SemaphoreHandle_t _draw = nullptr;      // storage + shared state, caller vs task
    SemaphoreHandle_t _prog = nullptr;      // task -> waiting caller, every frame
    const jc8012t_font_t *_font = nullptr;
    int16_t _W = 800, _H = 1280, _cols = 80;   // the upright screen
    int16_t _num = 1, _den = 1;             // its scale on the panel (13/8 at 480)
    int16_t _ox = 0, _oy = 0;               // its corner on the panel (480: 10, 22)
    int16_t _rw = 800, _rh = 1280;          // its size on the panel (480: 780 x 1235)
    uint8_t _CH = 20;
    uint8_t _rot = 0;                       // 0..3, clockwise quarter turns
    bool _ppa = false;
    uint16_t *_store = nullptr;             // framebuffer (direct) or surface (PPA)
    int32_t _stride = 800;                  // pixels per storage row
    int32_t _SP = 0;                        // spare ring rows
    ppa_client_handle_t _ppaClient = nullptr;
    uint8_t _front = 0;                     // PPA mode: framebuffer on screen
    uint32_t _retiredAt[3] = {};            // PPA mode: frame count when each left the screen

    // ── Shared state, guarded by _draw ────────────────────────────────────
    // Positions in the scrolling area are "virtual" pixel rows: an ever-
    // increasing coordinate whose value mod _R is the ring row that holds it.
    int16_t _fixT = 0, _fixB = 0, _VSA = 1280;
    int32_t _R = 1280;
    int32_t _vtop = 0;                      // virtual row at the top of the window, as last set
    int32_t _vtopLive = 0;                  // ... as the display is scanning it now (direct
                                            // mode lags _vtop by one frame)
    int32_t _vtopT = 0;                     // ... once scrolling catches up
    int32_t _clearedTo = 0;                 // virtual rows below this are ready to show
    int32_t _grid = 0;                      // text lines start at virtual rows == _grid (mod _CH)
    int32_t _curTop = 0;                    // cursor line
    int16_t _curCol = 0;                    // == _cols: parked past the right edge
    uint16_t _screenBg = JC8012T_BLACK;
    bool _dirty = false;                    // PPA mode: storage changed since the last compose
    volatile bool _composing = false;       // PPA mode: a compose is running (without _draw)
    bool _busy = false;                     // a caller is mid-operation: the task leaves the cursor alone
    jc8012t_cursor_t _cursorStyle = JC8012T_CURSOR_BLOCK;
    bool _blinkOn = true;
    TickType_t _blinkAt = 0;
    bool _csShown = false;                  // cursor pixels on screen; _curSave holds what's under
    int32_t _csTop = 0;
    int16_t _csX = 0, _csY = 0, _csW = 0, _csH = 0;
    uint8_t *_dlo[4] = {}, *_dhi[4] = {};   // dirty byte ranges to write back, per region
    volatile uint32_t _scrolled = 0;
    volatile uint32_t _composeUs = 0, _composeN = 0;

    // ── Set before begin() ────────────────────────────────────────────────
    uint8_t _spareLines = JC8012T_SPARE_LINES_DEFAULT;
    uint8_t _ppaFbs = 3;
    UBaseType_t _taskPrio = 5;
    BaseType_t _taskCore = tskNO_AFFINITY;

    // ── Caller-side only ──────────────────────────────────────────────────
    uint16_t _fg = JC8012T_WHITE, _bg = JC8012T_BLACK;
    bool _immediateWrap = false;
    volatile jc8012t_scroll_mode_t _scrollMode = JC8012T_SCROLL_SMOOTH;
    volatile uint16_t _smoothLimit = 100;   // lines a second; 0 = none
    int64_t _lastLoopUs = 0;                // task: when it last went round
    uint32_t _u8cp = 0;
    uint8_t _u8need = 0;
    uint16_t _palFg = 1, _palBg = 1;
    uint16_t _pal[16];

    // storage
    uint16_t *_rowPtr(Band b, int32_t idx, int copy, bool &rev) const;
    void _span(Band b, int32_t idx, int x, int n, const uint16_t *px, uint16_t color);
    void _readSpan(Band b, int32_t idx, int x, int n, uint16_t *out) const;
    void _screenRow(int y, int32_t vtop, Band &b, int32_t &idx) const;
    void _markDirty(int region, uint8_t *p, size_t bytes);
    void _commit();
    // text
    void _lock();
    void _unlock();
    void _feedByte(uint8_t b);
    void _putCp(uint32_t cp);
    void _putGlyph(uint16_t cp);
    void _newline();
    void _clearEol();
    void _moveCursor(int32_t top, int16_t col);
    void _ensureReady();
    void _waitFrame();
    void _flushLocked();
    void _drawCell(int32_t lineTop, int col, uint16_t cp, uint16_t fg, uint16_t bg, int &w);
    void _palette(uint16_t fg, uint16_t bg);
    void _showCursor();
    void _liftCursor();
    void _clearAll(bool includeFrozen, uint16_t color);
    void _layout(int16_t T, int16_t B);
    int32_t _lineFloor(int32_t v) const;
    bool _isWide(uint16_t cp) const;
    const uint8_t *_glyph(uint16_t cp, uint32_t &len, bool &wide) const;
    // task
    static void _taskEntry(void *arg);
    void _taskLoop();
    void _applyScan();
    struct ComposeSnap { int32_t vtop, R; int16_t fixT, fixB, VSA; };
    void _compose(const ComposeSnap &v);
};

#endif // ARDUINO_ESP32P4_DEV
