/*
 * TerminalHeaderFooter - a fixed header and footer with text scrolling
 * between them, on the text terminal of the Guition JC8012P4A1C.
 *
 * The terminal can leave strips at the top and bottom of the screen out of
 * the scroll ("frozen").  This sketch freezes 40 pixels at the top and 40 at
 * the bottom, draws a pre-rendered header and footer image into them with
 * drawBitmap(), and prints a log in between.  While the text scrolls, it
 * keeps updating an "activity" bar inside the footer -- drawing into a frozen
 * area never disturbs the scrolling text.  At 800 x 1280 (rotation 0 or
 * 180) the display DMA itself reads the screen in pieces -- header, the
 * scrolling window (two pieces where it wraps), footer -- so nothing is
 * copied to scroll.
 *
 * The images are RGB565 arrays in bitmaps.h, 1280 pixels wide (the screen's
 * width in landscape), made by extras/bitmaps/make_header_footer.py.  Make
 * your own the same way: any w x h array of uint16_t RGB565 pixels, row by
 * row, will do.  They are drawn centered, so in portrait they lose 240 pixels
 * at each side, and 400 at resolution 480.
 *
 * Header and footer are each two text lines tall (40 pixels), so the pixels
 * between them hold a whole number of lines in every mode: 60 in portrait,
 * 36 in landscape, 34 and 20 at resolution 480.  (At 480 a frozen height
 * must also be a multiple of 8, for the PPA's 1.625x scale; freeze() rounds
 * up.)  Freeze a height that isn't a multiple of 20 and the top line can end
 * up partly cut off; evictPartialLine() scrolls just enough to push it off,
 * and this sketch shows the call.
 *
 * Try the other ways to run: set TERM_ROTATION (0, 90, 180, 270 -- the
 * picture turned clockwise; 0 and 180 are portrait) and
 * TERM_RESOLUTION (800, or 480 for larger text) below.
 *
 * Arduino IDE Board Settings:
 *   - Board: ESP32P4 Dev Module
 *   - PSRAM: Enabled
 *   - Flash Size: 16 MegaBytes (128 megabits)
 *   - Partition Scheme: Custom (uses the partitions.csv in this sketch folder)
 *   - USB CDC On Boot: Enabled
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 */

#include <chipguy_JC8012P4A1C_terminal.h>
#include "bitmaps.h"

JC8012P4A1C_Board            board;            // the touch chip's I2C bus
chipguy_JC8012P4A1C_display  display(board);
chipguy_JC8012P4A1C_terminal term;

#define TERM_ROTATION   0      // 0, 90, 180 or 270
#define TERM_RESOLUTION 800    // 800, or 480 for larger text

// Fill part of the footer's activity bar: `fill` of BAR_W pixels lit.
static void drawBar(int fill, uint16_t color) {
    uint16_t row[BAR_W];
    for (int x = 0; x < BAR_W; x++) row[x] = x < fill ? color : 0x2104;
    int16_t x0 = (term.width() - FOOTER_W) / 2 + BAR_X;
    int16_t y0 = term.height() - FOOTER_H + BAR_Y;
    for (int y = 0; y < BAR_H; y++) term.drawBitmap(x0, y0 + y, BAR_W, 1, row);
}

void setup() {
    Serial0.begin(115200);
    if (!term.begin(display, TERM_ROTATION, TERM_RESOLUTION)) {
        Serial0.println("Display init failed!");
        while (1) delay(100);
    }
    term.setCursorStyle(JC8012T_CURSOR_UNDERLINE);
    term.setScrollMode(JC8012T_SCROLL_INSTANT);   // full speed throughout

    // Freeze the strips, then draw into them.  Centered: in portrait the
    // 1280-wide images lose 240 pixels at each side (400 at resolution 480).
    term.freeze(HEADER_H, FOOTER_H);
    int16_t x = (term.width() - HEADER_W) / 2;
    term.drawBitmap(x, 0, HEADER_W, HEADER_H, kHeader);
    term.drawBitmap(x, term.height() - FOOTER_H, FOOTER_W, FOOTER_H, kFooter);
    drawBar(0, JC8012T_GREEN);

    term.printf("%d x %d cells\nbetween the\nheader and footer\n", term.columns(), term.rows());
}

void loop() {
    static uint32_t n = 0;

    // A long burst of log lines as fast as the terminal takes them, with the
    // footer bar tracking progress.
    const int kLines = 400;
    term.flush();
    uint32_t t0 = millis(), px0 = term.pixelsScrolled();
    for (int i = 0; i < kLines; i++) {
        term.setTextColor(i % 7 ? JC8012T_WHITE : JC8012T_YELLOW);
        term.printf("line %lu\n", (unsigned long)n++);
        if (i % 10 == 9) drawBar((i + 1) * BAR_W / kLines, JC8012T_GREEN);
    }
    term.flush();
    float lps = (term.pixelsScrolled() - px0) * 1000.0f / (millis() - t0) / term.cellHeight();

    // Scrolling speed.
    term.setTextColor(JC8012T_GREEN);
    term.printf("%.0f lines%sper second\n", lps, term.columns() >= 20 ? " " : "\n");
    delay(2000);

    // The top line is now probably cut in half by the header.
    term.setTextColor(JC8012T_CYAN);
    term.print("top line partial?");
    delay(1500);
    term.evictPartialLine();          // scroll on until it's gone
    term.print("\revicted" JC8012T_CLEAR_EOL_STR);
    delay(1500);
    term.evictPartialLine(true);      // or back: blank space comes in at the top
    term.print("\rscrolled back" JC8012T_CLEAR_EOL_STR);
    delay(1500);
    term.setTextColor(JC8012T_WHITE);
    term.println();
    drawBar(0, JC8012T_GREEN);
}
