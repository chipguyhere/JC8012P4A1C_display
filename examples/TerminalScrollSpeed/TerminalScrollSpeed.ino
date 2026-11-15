/*
 * TerminalScrollSpeed - how fast the text terminal prints and scrolls, on
 * the Guition JC8012P4A1C.
 *
 * Plays a pretend boot log: bursts of a few hundred lines as fast as the
 * terminal will take them, then a pause "waiting" for something, with a
 * spinner that rewrites one line in place.  Too fast to read -- that's the
 * point.  The sketch only ever prints.  Screen lines per second for each
 * burst go to the screen and to Serial0.  Bursts alternate between the two
 * scroll modes, JC8012T_SCROLL_SMOOTH and JC8012T_SCROLL_INSTANT (see
 * setScrollMode()); in PPA mode the time each screen update takes is
 * reported too.
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

static const char *const kWords[] = {
    "init", "probe", "ok", "mount", "sync", "link up", "retry", "cache",
    "dma", "irq", "spi", "i2c", "uart", "adc", "rtc", "nvs",
};

// One burst of `lines` log lines; returns pixel rows scrolled per second.
// A line either fits the screen or runs well onto a second line -- never by
// just a character or two, which looks like a mistake.
static float burst(int lines) {
    term.flush();
    uint32_t t0 = millis(), px0 = term.pixelsScrolled();
    for (int i = 0; i < lines; i++) {
        uint32_t ms = millis();
        uint32_t r = esp_random();
        char line[48];
        int n = snprintf(line, sizeof line, "%s %02lX", kWords[r % 16],
                         (unsigned long)((r >> 8) & 0xFF));
        int cells = 11 + n;                       // after the "[ssss.mmm] " stamp
        if ((r & 0x30) == 0) cells += 2;          // " \u2713" is two cells
        int over = cells - term.columns();
        if (over == 1 || over == 2) n += snprintf(line + n, sizeof line - n, " (again)");
        term.setTextColor(JC8012T_GREY);
        term.printf("[%4lu.%03lu] ", (unsigned long)(ms / 1000), (unsigned long)(ms % 1000));
        term.setTextColor((r & 7) == 0 ? JC8012T_YELLOW : JC8012T_WHITE);
        term.printf("%s%s\n", line, (r & 0x30) == 0 ? " \u2713" : "");
    }
    term.flush();
    return (term.pixelsScrolled() - px0) * 1000.0f / (millis() - t0);
}

// Pretend to wait on something for `ms`, spinning a cursor in place.
static void pretendWait(const char *what, uint32_t ms) {
    static const char *const spin = "|/-\\";
    term.setTextColor(JC8012T_CYAN);
    uint32_t t0 = millis();
    for (int k = 0; millis() - t0 < ms; k++) {
        term.printf("\r%s %c" JC8012T_CLEAR_EOL_STR, what, spin[k & 3]);
        delay(120);
    }
    term.printf("\r%s done" JC8012T_CLEAR_EOL_STR "\n", what);
}

void setup() {
    Serial0.begin(115200);
    if (!term.begin(display, TERM_ROTATION, TERM_RESOLUTION)) {
        Serial0.println("Display init failed!");
        while (1) delay(100);
    }
    term.setCursorStyle(JC8012T_CURSOR_OFF);
    Serial0.printf("TerminalScrollSpeed: %d x %d, rotation %d, %s mode, %d x %d cells\n",
                   term.width(), term.height(), term.rotation(),
                   term.usesPPA() ? "PPA" : "direct", term.columns(), term.rows());
}

void loop() {
    static const char *const kWaits[] = {"dhcp", "sensors", "flash", "ntp"};
    static int round = 0;
    // Alternate the scroll modes, to compare.
    term.setScrollMode(round & 1 ? JC8012T_SCROLL_INSTANT : JC8012T_SCROLL_SMOOTH);
    float pps = burst(200 + 100 * (round % 3));
    float lps = pps / term.cellHeight();
    term.setTextColor(JC8012T_GREEN);
    // The speed, on one line where it fits.
    term.printf("%s scrolling:\n", round & 1 ? "instant" : "smooth");
    term.printf("%.0f lines%sper second\n", lps, term.columns() >= 20 ? " " : "\n");
    Serial0.printf("%s burst: %.0f px/s = %.1f screen lines/s, stack never used %lu bytes",
                   round & 1 ? "instant" : "smooth", pps, lps, (unsigned long)term.stackHighWater());
    if (term.usesPPA()) Serial0.printf(", PPA update %lu us", (unsigned long)term.composeMicros());
    Serial0.println();
    pretendWait(kWaits[round++ % 4], 1500);
}
