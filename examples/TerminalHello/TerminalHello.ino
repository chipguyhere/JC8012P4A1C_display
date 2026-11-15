/*
 * TerminalHello - the text terminal's basics on the Guition JC8012P4A1C.
 *
 * No LVGL: print() and printf() write straight onto a grid of character
 * cells, and the hardware scrolls the text.  This sketch shows colors,
 * printf padding, tabs, clear-to-end-of-line, symbols, box drawing and the
 * four cursor styles, then counts upward forever so you can watch it scroll.
 *
 * Try the other ways to run: set TERM_ROTATION (0, 90, 180, 270 -- the
 * picture turned clockwise; 0 and 180 are portrait) and
 * TERM_RESOLUTION (800, or 480 for larger text) below.  800 at 0 or 180 is
 * scrolled by the display DMA alone; the rest use the PPA.
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

JC8012P4A1C_Board            board;            // the touch chip's I2C bus
chipguy_JC8012P4A1C_display  display(board);
chipguy_JC8012P4A1C_terminal term;

#define TERM_ROTATION   0      // 0, 90, 180 or 270
#define TERM_RESOLUTION 800    // 800, or 480 for larger text

void setup() {
    Serial0.begin(115200);
    if (!term.begin(display, TERM_ROTATION, TERM_RESOLUTION)) {
        Serial0.println("Display init failed!");
        while (1) delay(100);
    }
    Serial0.printf("Terminal: %d x %d, rotation %d, %s mode, %d x %d cells\n",
                   term.width(), term.height(), term.rotation(),
                   term.usesPPA() ? "PPA" : "direct", term.columns(), term.rows());

    term.setTextColor(JC8012T_YELLOW);
    term.println("Hello, JC8012!");
    term.setTextColor(JC8012T_WHITE);
    term.printf("%d x %d cells\n", term.columns(), term.rows());

    // printf pads to a width: numbers, and strings (counted in cells).
    term.printf("[%5d][%-4s]\n", 42, "ab");
    term.printf("[%05.1f%s]\n", 21.5, "°C");      // 021.5°C

    // Tab moves to the next multiple of 4 columns.
    term.print("a\tbc\td\n");

    // Background colors, and '\v' (JC8012T_CLEAR_EOL) to clear the rest of a line.
    term.setTextColor(JC8012T_BLACK, JC8012T_CYAN);
    term.print(" inverse ");
    term.print(JC8012T_CLEAR_EOL_STR "\n");                // the cyan runs to the edge
    term.setTextColor(JC8012T_WHITE, JC8012T_BLACK);

    // Symbols: Unicode, and LVGL's set under the same names.
    term.print("± µ Ω ✓ ⚠ ");
    term.print(LV_SYMBOL_WIFI " " LV_SYMBOL_BATTERY_3 " " LV_SYMBOL_OK "\n");

    // Box drawing meets at the cell edges.
    term.setTextColor(JC8012T_GREEN);
    term.print("┌──┬──┐\n");
    term.print("│OK│  │\n");
    term.print("└──┴──┘\n");
    term.setTextColor(JC8012T_WHITE);

    // The cursor can sit on text printed earlier: here, on the "O" of "OK".
    int16_t row = term.cursorRow();
    term.setCursor(1, row - 2);
    delay(2500);
    term.setCursor(0, row);

    // The four cursor styles, two seconds each.
    term.print("block ");
    term.setCursorStyle(JC8012T_CURSOR_BLOCK);
    delay(2000);
    term.print("\rline  ");
    term.setCursorStyle(JC8012T_CURSOR_UNDERLINE);
    delay(2000);
    term.print("\rbar   ");
    term.setCursorStyle(JC8012T_CURSOR_BAR);
    delay(2000);
    term.print("\roff   ");
    term.setCursorStyle(JC8012T_CURSOR_OFF);
    delay(2000);
    term.setCursorStyle(JC8012T_CURSOR_BLOCK);
    term.println();
}

// Count upward, slowly at first and then with no delay at all: from there on
// the terminal's scrolling is the only thing setting the pace.
void loop() {
    static uint32_t n = 0;
    term.printf("count %lu\n", (unsigned long)n++);
    if (n % 100 == 0)
        Serial0.printf("count %lu, terminal task stack never used: %lu bytes\n",
                      (unsigned long)n, (unsigned long)term.stackHighWater());
    if (n < 200) delay(200 - n);
}
