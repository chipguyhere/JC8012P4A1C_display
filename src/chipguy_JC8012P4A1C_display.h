/*
 * JD9365 MIPI-DSI display driver for the JC8012P4A1C 8" / 10.1" ESP32-P4 LCD.
 *
 * This is the Guition "JC8012P4A1C" board: an ESP32-P4 driving an 800x1280
 * JD9365 panel over 2-lane MIPI-DSI, with a GSL3680 capacitive touch panel on
 * I2C.  800x1280 is the panel's NATIVE (portrait) geometry.
 *
 * Designed to feed an LVGL display in DIRECT render mode: LVGL renders a full
 * frame into a draw buffer, and the ESP32-P4 PPA (Pixel Processing Accelerator)
 * rotates that buffer in hardware into one of two DSI framebuffers, then flips
 * the panel to show it.  Rotation is therefore free (no CPU cost, no per-pixel
 * software transform) and the same UI code runs in any of the four orientations.
 *
 * Unlike the companion chipguy_10inchP4_480display library, this driver renders
 * at the panel's full native resolution: there is NO scaling, only rotation.
 *
 *   rotation 0  -> portrait  (800x1280, native, PPA 0 deg)
 *   rotation 1  -> landscape (1280x800, PPA 90 deg)
 *   rotation 2  -> portrait flipped  (800x1280, PPA 180 deg)
 *   rotation 3  -> landscape flipped (1280x800, PPA 270 deg)
 *
 * Degrees (0/90/180/270) are accepted as aliases for the indices 0/1/2/3.
 *
 * Board wiring (fixed on the JC8012P4A1C board) — see pins_config.h:
 *   LCD: MIPI-DSI (2 data lanes), RST GPIO27, backlight GPIO23 (active HIGH)
 *   Touch (GSL3680, I2C): SDA GPIO7, SCL GPIO8, RST GPIO22, INT GPIO21
 *
 * Copyright (c) 2025 chipguyhere
 * MIT License
 */

#pragma once

#ifdef ARDUINO_ESP32P4_DEV

#ifndef BOARD_HAS_PSRAM
#error "This library requires PSRAM. Enable PSRAM (OPI) in Arduino IDE Tools menu."
#endif

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "driver/ppa.h"
#include "gsl3680_touch.h"
#include "pins_config.h"

// Native panel geometry (portrait).  width()/height() reflect the active
// rotation; these are the unrotated physical dimensions.
#define JC8012_NATIVE_WIDTH   800
#define JC8012_NATIVE_HEIGHT  1280

// Normalize a rotation argument to an index 0..3.  Accepts either the index
// directly (0..3) or degrees (0/90/180/270) as aliases for 0/1/2/3.
static inline uint8_t jc8012_normalize_rotation(uint16_t rotation) {
    switch (rotation) {
        case 90:  return 1;
        case 180: return 2;
        case 270: return 3;
        default:  return (uint8_t)(rotation & 0x03);
    }
}

class chipguy_JC8012P4A1C_display {
public:
    chipguy_JC8012P4A1C_display();

    // Bring up the MIPI-DSI panel, the GSL3680 touch, two DSI framebuffers,
    // two full-resolution LVGL draw buffers, and the PPA rotation client.
    // rotation: 0 = portrait (800x1280, native), 1 = landscape (1280x800),
    //           2 = portrait flipped, 3 = landscape flipped.  Degrees
    //           (0/90/180/270) are accepted as aliases for 0/1/2/3.
    // Returns false if any allocation or hardware step failed.
    bool begin(uint16_t rotation = 0);

    // Draw buffers LVGL renders into (two, for DIRECT render mode).  Sized to
    // the logical orientation: 800x1280 portrait, 1280x800 landscape.
    uint16_t *getDrawBuffer(uint8_t index);

    // The two native 800x1280 DSI framebuffers the panel scans out of.
    uint16_t *getFramebuffer(uint8_t index);

    // Choose which framebuffer the panel displays (used internally by flip()).
    void setActiveFramebuffer(uint8_t index, bool wait_for_vsync = true);

    // PPA-rotate the given draw buffer into the given framebuffer (using the
    // rotation chosen in begin(), no scaling), then show that framebuffer.
    // Runs asynchronously on a dedicated task; serializes on the previous flip.
    void flip(uint8_t draw_buf_index, uint8_t target_fb_index);

    // Block until the next frame boundary (vsync).
    bool waitVsync(uint32_t timeout_ms = 50);

    uint8_t getNumFramebuffers();

    // Logical dimensions for the current rotation (what LVGL renders at).
    int16_t width()  const { return _draw_w; }
    int16_t height() const { return _draw_h; }
    size_t  framebufferSize() const { return (size_t)_draw_w * _draw_h * sizeof(uint16_t); }

    // Native (unrotated) panel dimensions.
    int16_t nativeWidth()  const { return JC8012_NATIVE_WIDTH; }
    int16_t nativeHeight() const { return JC8012_NATIVE_HEIGHT; }

    // Current rotation (0..3).
    uint8_t rotation() const { return _rotation; }

    // Backlight brightness, 0..100 (active HIGH on this board).
    void setBacklight(int percentage);
    void setBacklight(bool on) { setBacklight(on ? 100 : 0); }

    // Touch driver access (the GSL3680 reports in NATIVE panel coordinates).
    gsl3680_touch& getTouchDriver() { return _touch; }

    // Map a raw NATIVE touch point (as returned by getTouchDriver().getTouch())
    // to LOGICAL coordinates for the active rotation, so a touch lines up with
    // what LVGL drew.  Returns false (and leaves lx/ly untouched) if the point
    // falls outside the logical area.
    bool mapTouch(uint16_t rawX, uint16_t rawY, int32_t &lx, int32_t &ly) const;

    // Debug counters from the DPI panel driver.
    uint32_t getFrameCount();
    uint32_t getUnderrunCount();

private:
    gsl3680_touch _touch;
    esp_lcd_panel_handle_t _panel_handle = nullptr;
    esp_lcd_panel_io_handle_t _io_handle = nullptr;
    ppa_client_handle_t _ppa_client = nullptr;
    uint16_t *_draw_buffers[2] = {nullptr, nullptr};
    uint8_t  _rotation = 0;
    uint16_t _draw_w = JC8012_NATIVE_WIDTH;
    uint16_t _draw_h = JC8012_NATIVE_HEIGHT;

    bool initHardware();
    bool allocateDrawBuffers();

    // Async PPA task: rotates a draw buffer into a framebuffer and flips.
    static void ppaTaskFunc(void *arg);
    TaskHandle_t _ppa_task_handle = nullptr;
    SemaphoreHandle_t _ppa_done_sem = nullptr;
    struct {
        uint8_t draw_buf_index;
        uint8_t target_fb_index;
    } _ppa_work;
};

#endif // ARDUINO_ESP32P4_DEV
