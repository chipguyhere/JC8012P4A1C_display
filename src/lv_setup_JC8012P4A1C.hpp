/*
 * lv_setup_JC8012P4A1C.hpp - Connects LVGL to the JC8012P4A1C ESP32-P4 display
 * and touch.
 *
 * This is the *actual* LVGL-to-driver glue for this board.  It ships inside the
 * library (src/) so every example can share one copy.  Each example's sketch
 * folder contains a small lv_setup.hpp that simply #includes this file; that
 * thin shim is the file you edit if you want to take this code over and
 * customize it (see the comments in any example's lv_setup.hpp).
 *
 * Include this file (or, normally, the example's lv_setup.hpp) after lvgl.h.
 *
 * The JD9365 panel is driven in LVGL DIRECT render mode: LVGL renders a full
 * frame into one of two full-resolution draw buffers, then the ESP32-P4 PPA
 * (Pixel Processing Accelerator) rotates that buffer in hardware into a DSI
 * framebuffer and flips the panel to it.  Rotation is therefore free, and the
 * GSL3680 capacitive touch needs no calibration — its native coordinates are
 * mapped to the active rotation by the driver's mapTouch().
 */

#pragma once

#ifdef CHIPGUY_LV_SETUP_INCLUDED
#error "Another lv_setup_*.hpp is already included - it also defines `display` and `lv_setup`."
#endif
#define CHIPGUY_LV_SETUP_INCLUDED

#include <JC8012P4A1C_Board.h>                 // shared I2C bus + pin map
#include <chipguy_JC8012P4A1C_display.h>
#include "esp_cache.h"

// The board owns the shared I2C bus (GPIO7/8); the touch controller is on it,
// and so are the ES8311 codec, the RX8025T RTC and the expansion headers.  Use
// board.Wire for any of those — never ::Wire.begin() on other pins.
static JC8012P4A1C_Board board;
static chipguy_JC8012P4A1C_display display(board);

class lv_setup_class {
public:
    // Optional callback invoked on every touch press (e.g. activity timeout).
    // It fires on each poll while a finger is held, so it repeats during a press.
    void (*onTouch)() = nullptr;

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

    // Low-power sleep: panel off, chip in light sleep, back where it left off.
    // The finger that wakes the board is swallowed — LVGL sees no press until
    // it lifts — so the waking swipe can't hit a button.  Measured currents
    // and what each wakes on are in the display driver's header; in short:
    //   sleepUntilSwipe()  ~16 mA (3 s checks) — swipe to wake
    //   sleepUntilTap()    ~45 mA              — any touch wakes
    //   sleepUntilButton() ~10 mA              — BOOT only
    // Hold the Wi-Fi chip off first (board.setRadioPower(false)) unless the
    // sketch uses Wi-Fi: it costs ~22 mA asleep or awake.
    chipguy_JC8012P4A1C_display::WakeReason sleepUntilSwipe(uint32_t checkMs = 3000) {
        auto why = display.sleepUntilSwipe(checkMs); _afterWake(); return why;
    }
    chipguy_JC8012P4A1C_display::WakeReason sleepUntilTap() {
        auto why = display.sleepUntilTap(); _afterWake(); return why;
    }
    chipguy_JC8012P4A1C_display::WakeReason sleepUntilButton() {
        auto why = display.sleepUntilButton(); _afterWake(); return why;
    }

private:
    uint8_t *_drawBuf0 = nullptr;
    uint8_t *_drawBuf1 = nullptr;
    size_t   _drawBufSize = 0;
    uint8_t  _target_fb_index = 0;
    bool     _swallowUntilRelease = false;

    void _afterWake() {
        _swallowUntilRelease = true;
        lv_obj_invalidate(lv_screen_active());   // redraw anything changed while asleep
    }

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
        auto *self = (lv_setup_class *)lv_indev_get_user_data(indev);
        uint16_t rawX, rawY;
        if (!display.getTouchDriver().getTouch(&rawX, &rawY)) {
            self->_swallowUntilRelease = false;   // the waking finger has lifted
            data->state = LV_INDEV_STATE_RELEASED;
            return;
        }
        if (self->_swallowUntilRelease) {         // still the finger that woke us
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
        if (self->onTouch) self->onTouch();
    }
};

static lv_setup_class lv_setup;
