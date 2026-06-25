/*
 * lv_setup.hpp - Connects LVGL to the JC8012P4A1C ESP32-P4 display and touch.
 * Include this file after lv_conf.h and lvgl.h in your sketch.
 *
 * The JD9365 panel is driven in LVGL DIRECT render mode: LVGL renders a full
 * frame into one of two full-resolution draw buffers, then the ESP32-P4 PPA
 * (Pixel Processing Accelerator) rotates that buffer in hardware into a DSI
 * framebuffer and flips the panel to it.  Rotation is therefore free, and the
 * GSL3680 capacitive touch needs no calibration — its native coordinates are
 * mapped to the active rotation by the driver's mapTouch().
 */

#pragma once

#include <chipguy_JC8012P4A1C_display.h>
#include "esp_cache.h"

static chipguy_JC8012P4A1C_display display;

class lv_setup_class {
public:
    // Initializes display, touch input, and LVGL.
    // rotation: 0 = portrait (800x1280, native), 1 = landscape (1280x800),
    //           2 = portrait flipped, 3 = landscape flipped.  Degrees
    //           (0/90/180/270) are accepted as aliases for 0/1/2/3.
    void begin(uint16_t rotation = 0) {
        if (!display.begin(rotation)) {
            Serial0.println("Display init failed!");
            while (1) delay(100);
        }
        Serial0.printf("Display initialized (%dx%d)\n", display.width(), display.height());

        _drawBuf0 = (uint8_t *)display.getDrawBuffer(0);
        _drawBuf1 = (uint8_t *)display.getDrawBuffer(1);
        _drawBufSize = display.framebufferSize();

        lv_init();

        static lv_display_t *disp = lv_display_create(display.width(), display.height());
        lv_display_set_flush_cb(disp, _disp_flush);
        // Force RGB565 so the PPA's color mode (RGB565) always matches what LVGL
        // renders, independent of how lv_conf.h's color depth happened to resolve.
        lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
        lv_display_set_buffers(disp, _drawBuf0, _drawBuf1, _drawBufSize,
                               LV_DISPLAY_RENDER_MODE_DIRECT);
        lv_display_set_user_data(disp, this);

        static lv_indev_t *indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(indev, _touchpad_read);
        lv_indev_set_user_data(indev, this);

        lv_tick_set_cb(_tick_get);
    }

    // Access the underlying display driver (backlight, touch, etc.).
    chipguy_JC8012P4A1C_display &displayDriver() { return display; }

private:
    uint8_t *_drawBuf0 = nullptr;
    uint8_t *_drawBuf1 = nullptr;
    size_t   _drawBufSize = 0;
    uint8_t  _target_fb_index = 0;

    static uint32_t _tick_get(void) { return millis(); }

    static void _disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap) {
        auto *self = (lv_setup_class *)lv_display_get_user_data(disp);
        // In DIRECT mode LVGL may call flush several times per refresh (one per
        // dirty area).  Only on the last one is the draw buffer complete, so we
        // PPA-rotate it into a framebuffer and flip the panel then.
        if (lv_display_flush_is_last(disp)) {
            uint8_t draw_buf_index = (pixelmap == self->_drawBuf0) ? 0 : 1;
            display.flip(draw_buf_index, self->_target_fb_index);
            self->_target_fb_index ^= 1;
        }
        lv_display_flush_ready(disp);
    }

    static void _touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
        uint16_t rawX, rawY;
        if (!display.getTouchDriver().getTouch(&rawX, &rawY)) {
            data->state = LV_INDEV_STATE_RELEASED;
            return;
        }
        int32_t lx, ly;
        if (!display.mapTouch(rawX, rawY, lx, ly)) {
            data->state = LV_INDEV_STATE_RELEASED;
            return;
        }
        data->point.x = lx;
        data->point.y = ly;
        data->state = LV_INDEV_STATE_PRESSED;
    }
};

static lv_setup_class lv_setup;
