/*
 * TerminalTouchButtons - touch buttons made of colored text, on the text
 * terminal of the Guition JC8012P4A1C.
 *
 * Each button is a block of character cells three rows tall (a 20-pixel
 * row alone is a small target for a finger): a half block at each end and
 * the label in the middle row.  pollTouch() reports a touch in the same column / row
 * terms as setCursor(), so "was it touched?" is a simple comparison.  Two
 * buttons light the board's RGB LED red or green (through the board
 * library), the third dims the backlight; a pressed button shows lit while
 * your finger is on it.  Touch needs no setup: display.begin() starts the
 * touch chip, and pollTouch() reads it.
 *
 * The buttons are ordinary text, so printing enough to scroll would scroll
 * the buttons away with everything else -- for buttons that stay put while
 * text scrolls, put them in a frozen area (see TerminalHeaderFooter) and test
 * touches with JC8012T_TOUCH_PIXELS.
 *
 * Try the other ways to run: set TERM_ROTATION (0, 90, 180, 270 -- the
 * picture turned clockwise; 0 and 180 are portrait) and
 * TERM_RESOLUTION (800, or 480 for larger text) below.  Touch follows.
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

JC8012P4A1C_Board            board;            // the touch chip's I2C bus, the LED
chipguy_JC8012P4A1C_display  display(board);
chipguy_JC8012P4A1C_terminal term;

#define TERM_ROTATION   0      // 0, 90, 180 or 270
#define TERM_RESOLUTION 800    // 800, or 480 for larger text

enum Action : uint8_t { LED_RED, LED_GREEN, DIM };

struct Button {
    int16_t col, row, width;
    const char *label;
    uint16_t color;
    Action action;
    bool on;
};

static Button buttons[3];

static void drawButton(const Button &b, bool pressed) {
    // Off: the color's label on grey.  On: black on the color.  Pressed:
    // white on the color, so a press shows before you let go.
    uint16_t fill = (pressed || b.on) ? b.color : JC8012T_GREY;
    uint16_t text = pressed ? JC8012T_WHITE : b.on ? JC8012T_BLACK : b.color;
    for (int r = -1; r <= 1; r++) {
        term.setCursor(b.col, b.row + r);
        term.setTextColor(fill, JC8012T_BLACK);
        term.print("\u2590");                // ▐ right half block: the left end, half a cell in
        term.setTextColor(text, fill);
        if (r == 0) term.print(b.label);
        else for (int i = 0; i < b.width - 2; i++) term.print(' ');
        term.setTextColor(fill, JC8012T_BLACK);
        term.print("\u258C");                // ▌ left half block: the right end
    }
    term.setTextColor(JC8012T_WHITE, JC8012T_BLACK);
}

static void status(int16_t row, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    term.setCursor(0, row);
    term.vprintf(fmt, ap);
    term.print(JC8012T_CLEAR_EOL_STR);
    va_end(ap);
}

static int hit(int16_t col, int16_t row) {
    for (int i = 0; i < 3; i++) {
        const Button &b = buttons[i];
        if (row >= b.row - 1 && row <= b.row + 1 && col >= b.col && col < b.col + b.width) return i;
    }
    return -1;
}

void setup() {
    Serial0.begin(115200);
    if (!term.begin(display, TERM_ROTATION, TERM_RESOLUTION)) {
        Serial0.println("Display init failed!");
        while (1) delay(100);
    }
    if (!display.touchAvailable()) Serial0.println("Touch controller did not answer");
    term.setCursorStyle(JC8012T_CURSOR_OFF);

    // Three buttons of ten cells (half block, eight letters, half block)
    // spread across the screen.
    int16_t gap = (term.columns() - 10) / 2;
    buttons[0] = {0,                  3, 10, "  RED   ", JC8012T_RED,   LED_RED,   false};
    buttons[1] = {gap,                3, 10, " GREEN  ", JC8012T_GREEN, LED_GREEN, false};
    buttons[2] = {(int16_t)(2 * gap), 3, 10, "  DIM   ", JC8012T_CYAN,  DIM,       false};
    for (Button &b : buttons) drawButton(b, false);
    term.setTextColor(JC8012T_YELLOW);
    term.setCursor(0, 0);
    term.print("LED and backlight");
    term.setTextColor(JC8012T_WHITE);
    status(6, "Tap a button");
}

void loop() {
    static int pressed = -1;
    int16_t col, row, px, py;
    bool down = term.pollTouch(col, row);
    if (down) {
        term.pollTouch(px, py, JC8012T_TOUCH_PIXELS);
        status(8, "cell %d,%d", col, row);
        status(9, "px %d,%d", px, py);
    }
    int now = down ? hit(col, row) : -1;
    if (now != pressed) {
        if (pressed >= 0 && !down) {
            // Lifted off the button (not slid off it): toggle it.
            Button &b = buttons[pressed];
            b.on = !b.on;
            switch (b.action) {
                case LED_RED:
                case LED_GREEN:
                    board.setLed(buttons[0].on ? 64 : 0, buttons[1].on ? 64 : 0, 0);
                    break;
                case DIM:    display.setBacklight(b.on ? 20 : 100); break;
            }
            status(6, "%s %s", b.label, b.on ? "on" : "off");
        }
        if (pressed >= 0) drawButton(buttons[pressed], false);
        if (now >= 0) drawButton(buttons[now], true);
        pressed = now;
    }
    delay(20);
}
