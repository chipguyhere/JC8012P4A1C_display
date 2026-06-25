/*******************************************************************************
 * JC8012P4A1C LVGL Claude Code Stub
 *
 * For the Guition JC8012P4A1C ESP32-P4 LCD (800x1280 JD9365 MIPI-DSI panel +
 * GSL3680 capacitive touch).  Display rotation is done by the ESP32-P4 PPA
 * (Pixel Processing Accelerator) in hardware.
 *
 * A minimal starting point for building an LVGL application from scratch on
 * this hardware.  The UI lives in a tiny ui.h / ui.cpp pair in the sketch
 * folder that draws a simple "Hello, world!" screen.
 *
 * The intent is to hand this to Claude Code (or any developer) as a clean
 * scaffold: the display, touch, and LVGL plumbing is already wired up, so you
 * can focus on writing your own UI in ui.cpp.  Just replace the contents of
 * ui_init() with your own widgets and grow from there.
 *
 * Orientation: lv_setup.begin(rotation) accepts 0 = portrait (800x1280, the
 * panel's native orientation), 1 = landscape (1280x800), 2 = portrait flipped,
 * 3 = landscape flipped (degrees 90/180/270 work as aliases for 1/2/3).  This
 * sketch defaults to portrait; change the argument below to rotate.  Display
 * and touch rotate together.
 *
 * Compatible with LVGL 9; the bundled lv_conf.h is based on LVGL 9.3.  If your
 * installed LVGL differs, the API is generally close enough to adapt.
 *
 * Requires the following libraries:
 *   - chipguy_JC8012P4A1C_display  (display + touch driver, this library)
 *   - lvgl 9
 *
 * Arduino IDE Board Settings:
 *   - Board: ESP32P4 Dev Module
 *   - PSRAM: Enabled   (REQUIRED — the sketch will not compile without it)
 *   - Flash Size: 16 MegaBytes (128 megabits)
 *   - USB CDC On Boot: Enabled
 *
 ******************************************************************************/

// The bundled lv_conf.h is based on LVGL 9.3 (LVGL 9 compatible).
// It's important for the config and library versions to be compatible.
#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"

// The UI for this example lives in ui.h / ui.cpp in this sketch folder.
// It is a hand-written "Hello, world!" screen meant as a starting point.
// Build your own application by editing ui_init() in ui.cpp.
#include "ui.h"

void setup() {
    // On the ESP32-P4 the USB serial port is Serial0; "USB CDC On Boot" must be
    // enabled in the Tools menu to see this output.
    Serial0.begin(115200);

    // Initialize display, touch, and LVGL.  Defaults to portrait; pass a
    // rotation to change it (0/2 = portrait, 1/3 = landscape; see header).
    lv_setup.begin();
    Serial0.printf("LVGL initialized with %dx%d touchscreen\n",
                   display.width(), display.height());

    // Start the application's own setup
    ui_init();
}

void loop() {
    // Give loop control to LVGL objects created by the application
    lv_timer_handler();
    delay(5);
}
