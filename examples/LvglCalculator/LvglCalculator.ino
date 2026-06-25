/*******************************************************************************
 * JC8012P4A1C LVGL Calculator
 *
 * For the Guition JC8012P4A1C ESP32-P4 LCD (800x1280 JD9365 MIPI-DSI panel +
 * GSL3680 capacitive touch).  Display rotation is done by the ESP32-P4 PPA
 * in hardware.
 *
 * A working four-function calculator with a touch keypad, styled after the
 * familiar dark iOS calculator: a black background, dark-gray number keys, a
 * light-gray top function row (backspace / C / %) and an orange operator
 * column (/ x - + =).
 *
 * Resolution independence is the point of this example.  The whole UI is laid
 * out from the live display resolution in ui.cpp — button diameter, gaps,
 * margins and font sizes are all derived from width()/height() at runtime.
 * Nothing is hard-coded, so the identical ui.cpp lays out a proportionate
 * keypad on this 800x1280 panel and on a tiny 240x320 board alike.
 *
 * Orientation: lv_setup.begin(rotation) accepts 0 = portrait (800x1280, the
 * panel's native orientation), 1 = landscape (1280x800), 2 = portrait flipped,
 * 3 = landscape flipped (degrees 90/180/270 work as aliases for 1/2/3).  A
 * calculator wants portrait, so this sketch defaults to 0.
 *
 * Requires the following libraries:
 *   - chipguy_JC8012P4A1C_display  (display + touch driver, this library)
 *   - lvgl 9
 *
 * Arduino IDE Board Settings:
 *   - Board: ESP32P4 Dev Module
 *   - PSRAM: OPI PSRAM   (REQUIRED — the driver allocates its buffers in PSRAM)
 *   - Flash Size: 16 MegaBytes (128 megabits)
 *   - USB CDC On Boot: Enabled
 *
 ******************************************************************************/

// The bundled lv_conf.h is based on LVGL 9.3 (LVGL 9 compatible).
#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"

// The calculator UI lives in ui.h / ui.cpp in this sketch folder.
#include "ui.h"

void setup() {
    // On the ESP32-P4 the USB serial port is Serial0; "USB CDC On Boot" must be
    // enabled in the Tools menu to see this output.
    Serial0.begin(115200);

    // Initialize display, touch, and LVGL.  Portrait (800x1280) by default;
    // pass a rotation to change it (see header).  The UI adapts to whatever
    // resolution the chosen rotation reports.
    lv_setup.begin();
    Serial0.printf("LVGL initialized with %dx%d touchscreen\n",
                   display.width(), display.height());

    // Build the calculator screen.
    ui_init();
}

void loop() {
    // Give loop control to LVGL.
    lv_timer_handler();
    delay(5);
}
