/*
 * lv_setup.hpp - LVGL on a strip along the bottom of the landscape screen,
 * composited into the display's raster band by DMA.
 *
 * This is the customized form of the library's lv_setup_JC8012P4A1C.hpp (see
 * the comments in any other example's lv_setup.hpp): it has the same globals
 * -- `board`, `display`, `lv_setup` -- but LVGL is only a strip, and it never
 * reaches the panel's own framebuffer.
 *
 * The screen is landscape, the library's rotation 3: 1280 x 800, the panel
 * scanning the picture's columns right to left, each one top to bottom.  The
 * whole panel is a raster band (see RasterLandscapeBouncingBalls.ino), so
 * every panel row is made in a bounce buffer just before it is scanned.  In
 * this orientation the strip's rows are the last `stripRows` pixels of EVERY
 * panel row, so the strip has to go into every chunk.  It gets there like this:
 *
 *   1. LVGL renders the strip, 1280 x stripRows, into one of two draw buffers
 *      in PSRAM (DIRECT mode, so each draw buffer always holds the whole strip).
 *   2. The PPA turns it, 90 degrees, into one of two STRIP FRAMEBUFFERS, also
 *      in PSRAM: stripRows x 1280, laid out as the panel reads it, so the
 *      strip's part of 16 panel rows is one 16 x stripRows x 2-byte block
 *      (5 KB at 160 rows).
 *   3. The display's raster prefetch copies that block, by the AXI GDMA, into
 *      the last stripRows columns of the chunk's bounce buffer, one chunk
 *      ahead (rasterSrc() below tells it where from).  The balls are drawn
 *      into the rest of the row.
 *
 * Flipping between the two strip framebuffers never tears: rasterSrc()
 * changes source only where a panel frame begins, and the flush waits until
 * the raster has moved off the strip framebuffer it is about to overwrite.
 *
 * The PPA turns only the columns LVGL changed -- in this refresh and in the
 * one before, which the other strip framebuffer missed -- so a spinner costs
 * a sliver of PSRAM traffic, not the whole 400 KB strip twice over.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 */

#pragma once

#ifdef CHIPGUY_LV_SETUP_INCLUDED
#error "Another lv_setup_*.hpp is already included - it also defines `display` and `lv_setup`."
#endif
#define CHIPGUY_LV_SETUP_INCLUDED

#include <JC8012P4A1C_Board.h>                 // shared I2C bus + pin map
#include <chipguy_JC8012P4A1C_display.h>
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "driver/ppa.h"

// The board owns the shared I2C bus (GPIO7/8); the touch controller is on it.
static JC8012P4A1C_Board board;
static chipguy_JC8012P4A1C_display display(board);

class lv_strip_setup_class {
public:
    // Optional callback invoked on every touch press.
    void (*onTouch)() = nullptr;

    // Brings up the display in landscape (rotation 3, 1280 x 800), touch, and
    // LVGL as a 1280 x stripRows display: the bottom stripRows rows of the
    // screen.  stripRows: a multiple of 32, 32..768.  rasterCore: the core the
    // raster task will run on; the panel's interrupts are taken there, away from
    // LVGL's long cache write-backs.  Call display.setScanBlocks() and
    // display.setRasterBuffers() before this.  refreshMs: how often LVGL may
    // redraw and flip the strip -- LVGL here is the controls, on casual
    // deadlines, not the animation.
    void begin(int stripRows, int rasterCore = 0, uint32_t refreshMs = 50) {
        _rows = stripRows;
        display.setInterruptCore(rasterCore);
        // LVGL's buffers are the strip's own, below; the panel's framebuffer
        // is never read once the raster band covers the whole panel, so one
        // is enough (it saves 2 MB of PSRAM and half the DMA lists).
        display.setDrawBuffers(false);
        display.setFramebufferCount(1);
        if (_rows % 32 || _rows < 32 || _rows > 768 || !display.begin(3)) {
            Serial0.println("Display init failed!");
            while (1) delay(100);
        }

        _bytes = (size_t)W * _rows * 2;       // a multiple of 128 (W * 2 is)
        for (int i = 0; i < 2; i++) {
            _draw[i]  = (uint16_t *)heap_caps_aligned_calloc(128, 1, _bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            _strip[i] = (uint16_t *)heap_caps_aligned_calloc(128, 1, _bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!_draw[i] || !_strip[i]) {
                Serial0.println("No PSRAM for the LVGL strip");
                while (1) delay(100);
            }
            // calloc cleared them through the cache; the PPA and the GDMA read memory
            esp_cache_msync(_draw[i], _bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            esp_cache_msync(_strip[i], _bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        }
        ppa_client_config_t pc = {};
        pc.oper_type = PPA_OPERATION_SRM;
        if (ppa_register_client(&pc, &_ppa) != ESP_OK) {
            Serial0.println("PPA client registration failed");
            while (1) delay(100);
        }
        Serial0.printf("Display initialized (%dx%d), LVGL strip %dx%d at the bottom\n",
                       display.width(), display.height(), W, _rows);

        lv_init();
        static lv_display_t *disp = lv_display_create(W, _rows);
        lv_display_set_flush_cb(disp, _disp_flush);
        lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
        lv_display_set_buffers(disp, _draw[0], _draw[1], _bytes, LV_DISPLAY_RENDER_MODE_DIRECT);
        lv_display_set_user_data(disp, this);
        lv_timer_set_period(lv_display_get_refr_timer(disp), refreshMs);

        static lv_indev_t *indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(indev, _touchpad_read);
        lv_indev_set_user_data(indev, this);

        lv_tick_set_cb(_tick_get);
    }

    chipguy_JC8012P4A1C_display &displayDriver() { return display; }

    int stripRows() const { return _rows; }
    // The first panel column the strip takes (800 - stripRows): the prefetch's x0.
    int panelColumn() const { return JC8012_NATIVE_WIDTH - _rows; }
    // The strip's first row on the landscape screen; the balls get the rows above.
    int screenRow() const { return JC8012_NATIVE_WIDTH - _rows; }
    // Strips turned and handed to the raster so far (LVGL's frames).
    uint32_t flips() const { return _flips; }
    // Total time the flush spent in the PPA, and waiting for the raster to
    // move off the strip framebuffer it was about to write.
    uint32_t ppaUs() const { return _ppaUs; }
    uint32_t flipWaitUs() const { return _waitUs; }

    // The raster prefetch's source: display.setRasterPrefetch(lv_setup.panelColumn(),
    // 800, lv_strip_setup_class::rasterSrc, &lv_setup).  Called from the raster task
    // for each chunk, in order, as its copy starts: rows `y`.. of the strip
    // framebuffer for panel frame `frame`.  The strip framebuffer is chosen
    // once a frame, where `frame` changes.
    static IRAM_ATTR const void *rasterSrc(int y, int rows, uint32_t frame, void *ctx) {
        (void)rows;
        lv_strip_setup_class *self = (lv_strip_setup_class *)ctx;
        if (frame != self->_srcFrame) {
            self->_srcFrame = frame;
            self->_src = self->_want;
            self->_running = true;
        }
        return self->_strip[self->_src] + (size_t)y * self->_rows;
    }

private:
    static constexpr int W = JC8012_NATIVE_HEIGHT;   // 1280: the strip's width on screen
    int _rows = 160;
    size_t _bytes = 0;
    uint16_t *_draw[2] = {};
    uint16_t *_strip[2] = {};
    ppa_client_handle_t _ppa = nullptr;
    volatile int _want = 0;              // the strip framebuffer written last (whole)
    volatile int _src = 0;               // the one the raster reads this panel frame
    volatile uint32_t _srcFrame = 0xFFFFFFFFu;
    volatile bool _running = false;      // the raster has asked for a source
    uint32_t _flips = 0, _ppaUs = 0, _waitUs = 0;
    bool _pressing = false;
    // Columns (strip x) changed in this refresh so far, and in the refresh
    // before (which the target strip framebuffer missed).  Both start whole,
    // so each strip framebuffer is filled once.
    int _dx1 = 0, _dx2 = W - 1;
    int _px1 = 0, _px2 = W - 1;

    static uint32_t _tick_get(void) { return millis(); }

    static void _disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap) {
        auto *self = (lv_strip_setup_class *)lv_display_get_user_data(disp);
        // The rows LVGL changed, written back from the cache for the PPA.  A
        // row is 2560 bytes, a whole number of cache lines.  8 rows at a time:
        // the write-back holds this core's interrupts off while it runs, and
        // the whole strip in one go (400 KB) held them off long enough to
        // delay the balls' helper task here past its chunk's deadline.
        for (int y = area->y1; y <= area->y2; y += 8) {
            int n = area->y2 + 1 - y < 8 ? area->y2 + 1 - y : 8;
            esp_cache_msync(pixelmap + (size_t)y * W * 2, (size_t)n * W * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        }
        if (area->x1 < self->_dx1) self->_dx1 = area->x1;
        if (area->x2 > self->_dx2) self->_dx2 = area->x2;
        // In DIRECT mode the draw buffer is whole after the last flush of a
        // refresh: turn it into the strip framebuffer the raster isn't using.
        if (lv_display_flush_is_last(disp)) {
            const int target = self->_want ^ 1;
            uint32_t t0 = micros();
            // Until the raster has taken the last flip (it does at the next
            // panel frame), it may still be reading `target`.
            while (self->_running && self->_src == target && micros() - t0 < 100000) vTaskDelay(1);
            uint32_t t1 = micros();
            // The columns to bring the target up to date: this refresh's and
            // the last one's.
            int x1 = self->_dx1 < self->_px1 ? self->_dx1 : self->_px1;
            int x2 = self->_dx2 > self->_px2 ? self->_dx2 : self->_px2;
            // Out: the panel rows W-1-x2 .. W-1-x1, a contiguous piece of the
            // strip framebuffer, given to the PPA as its whole output picture
            // so its driver's cache sync covers only that.  Rounded out to an
            // even number of rows from an even row: rows * 2 bytes x 2 is a
            // whole number of 128-byte cache lines (W - 1 is odd, so that is
            // x1 even, x2 odd).
            x1 &= ~1;
            x2 |= 1;
            const int r0 = W - 1 - x2, nr = x2 - x1 + 1;
            self->_px1 = self->_dx1;
            self->_px2 = self->_dx2;
            self->_dx1 = W;
            self->_dx2 = -1;
            ppa_srm_oper_config_t op = {};
            op.in.buffer = pixelmap;
            op.in.pic_w = W;
            op.in.pic_h = self->_rows;
            op.in.block_w = x2 - x1 + 1;
            op.in.block_h = self->_rows;
            op.in.block_offset_x = x1;
            op.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
            // Turned: stripRows across, a panel row each, as the panel reads
            // them; strip column x is panel row 1279 - x.
            op.out.buffer = self->_strip[target] + (size_t)r0 * self->_rows;
            op.out.buffer_size = (uint32_t)nr * self->_rows * 2;
            op.out.pic_w = self->_rows;
            op.out.pic_h = nr;
            op.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
            // Counter-clockwise, as the library's rotation 3 (panel row 1279 - x
            // is the picture's column x)
            op.rotation_angle = PPA_SRM_ROTATION_ANGLE_90;
            op.scale_x = 1.0f;
            op.scale_y = 1.0f;
            op.mode = PPA_TRANS_MODE_BLOCKING;
            if (ppa_do_scale_rotate_mirror(self->_ppa, &op) == ESP_OK) {
                self->_want = target;
                self->_flips++;
            }
            uint32_t t2 = micros();
            self->_waitUs += t1 - t0;
            self->_ppaUs += t2 - t1;
        }
        lv_display_flush_ready(disp);
    }

    static void _touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
        auto *self = (lv_strip_setup_class *)lv_indev_get_user_data(indev);
        uint16_t rawX, rawY;
        int32_t lx, ly;
        if (!display.getTouchDriver().getTouch(&rawX, &rawY) || !display.mapTouch(rawX, rawY, lx, ly)) {
            self->_pressing = false;
            data->state = LV_INDEV_STATE_RELEASED;
            return;
        }
        ly -= self->screenRow();                   // into the strip's own rows
        if (ly < 0) {
            // Above the strip, over the balls: not LVGL's -- unless a finger
            // that went down in the strip has strayed up while dragging a slider.
            if (!self->_pressing) {
                data->state = LV_INDEV_STATE_RELEASED;
                return;
            }
            ly = 0;
        }
        self->_pressing = true;
        data->point.x = lx;
        data->point.y = ly;
        data->state = LV_INDEV_STATE_PRESSED;
        if (self->onTouch) self->onTouch();
    }
};

static lv_strip_setup_class lv_setup;
