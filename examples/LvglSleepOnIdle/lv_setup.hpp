/*
 * lv_setup.hpp - The bridge between LVGL and your display + touch hardware.
 *
 * THIS is the file that wires LVGL up to a specific board.  Right now it just
 * pulls in the ready-made setup that ships inside the
 * chipguy_JC8012P4A1C_display library, so you don't have to maintain a copy:
 *
 *   - It creates a global `board`       (the shared I2C bus + pin map, from
 *                                        chipguy_JC8012P4A1C_board: board.Wire,
 *                                        board.setLed(), board.bootButtonPressed())
 *   - It creates a global `display`     (the JD9365 MIPI-DSI driver)
 *   - It creates a global `lv_setup`    (an lv_setup_class instance)
 *   - Calling lv_setup.begin(rotation)  initializes display + touch + LVGL
 *
 * --------------------------------------------------------------------------
 *  TARGETING A DIFFERENT BOARD
 *  Point the #include below at a different library's setup header and the same
 *  sketch (ui.cpp / ui.h, etc.) drives a different board — as long as that
 *  header exposes the same `display` / `lv_setup` shape.
 *
 *  CUSTOMIZING HOW LVGL IS WIRED UP
 *  Because this file lives in your sketch folder, it's the easy place to take
 *  control.  Open the real implementation at:
 *
 *      <Arduino libraries>/chipguy_JC8012P4A1C_display/src/lv_setup_JC8012P4A1C.hpp
 *
 *  copy its entire contents over the #include line below, and edit freely —
 *  draw buffers, color format, rotation handling, the touch read callback, etc.
 *  Nothing else in the sketch needs to change.
 * --------------------------------------------------------------------------
 *
 * Include this file after lv_conf.h and lvgl.h in your sketch.
 *
 *
 * For reference, the setup class this pulls in looks like:
 *
 *   // A global instance named `lv_setup` is created for you.
 *   class lv_setup_class {
 *   public:
 *       // Initialize display, touch, and LVGL.  Call once from setup().
 *       // rotation: 0 = portrait (800x1280, native), 1 = landscape (1280x800),
 *       //           2 = portrait flipped, 3 = landscape flipped.
 *       //           Degrees (0/90/180/270) work as aliases for 0/1/2/3.
 *       void begin(uint16_t rotation = 0);
 *
 *       // The display driver, e.g. for backlight or the touch panel:
 *       //   lv_setup.displayDriver().setBacklight(50);
 *       chipguy_JC8012P4A1C_display &displayDriver();
 *   };
 *
 *   // Also provided as a global: the display driver, handy for resolution:
 *   //   display.width(), display.height()
 *   extern chipguy_JC8012P4A1C_display display;
 *
 * Tip: the GSL3680 is a capacitive touch panel — no calibration is required.
 * Its native coordinates are mapped to the active rotation automatically by the
 * driver's mapTouch(), so a touch always lines up with what LVGL drew.
 */

#pragma once

#include <lv_setup_JC8012P4A1C.hpp>
