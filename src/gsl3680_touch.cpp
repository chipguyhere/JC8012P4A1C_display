#ifdef ARDUINO_ESP32P4_DEV

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_touch.h"
#include "gsl3680_fw.h"
#include "gsl3680_touch.h"

#define CONFIG_LCD_HRES 800
#define CONFIG_LCD_VRES 1280

static const char *TAG = "example";

esp_lcd_touch_handle_t tp;

uint16_t touch_strength[1];
uint8_t touch_cnt = 0;

gsl3680_touch::gsl3680_touch(ThreadsafeWire &wire, int8_t rst_pin, int8_t int_pin)
    : _wire(wire), _rst(rst_pin), _int(int_pin)
{
}

esp_err_t gsl3680_touch::_busRead(void *ctx, uint8_t reg, uint8_t *data, uint8_t len)
{
    auto *self = (gsl3680_touch *)ctx;
    // readRegister() holds the bus across the register write, the repeated
    // start and the drain of Wire's RX buffer, so no other task can interleave.
    return self->_wire.readRegister(JC8012P4A1C_Board::GSL3680_ADDR, reg, data, len)
        ? ESP_OK : ESP_FAIL;
}

esp_err_t gsl3680_touch::_busWrite(void *ctx, uint8_t reg, const uint8_t *data, uint8_t len)
{
    auto *self = (gsl3680_touch *)ctx;
    return self->_wire.writeRegister(JC8012P4A1C_Board::GSL3680_ADDR, reg, data, len)
        ? ESP_OK : ESP_FAIL;
}

// Bring up the GSL3680 controller on the board's shared I2C bus.  Returns false
// instead of aborting: a watchdog reboot leaves the touch controller powered and
// can catch its I2C bus mid-transaction, and the caller is better placed than
// we are to decide whether that is worth a reset-and-retry or whether the
// display should just come up without touch.
bool gsl3680_touch::begin()
{
    esp_err_t ret;

    // Cleared up front so a partial init can't leave a stale handle behind for
    // getTouch()/set_rotation() to dereference.
    tp = nullptr;

    // The bus itself is the board library's (started by board.begin()); this
    // driver only ever talks to address 0x40 through it.
    const gsl3680_bus_t bus = { _busRead, _busWrite, this };

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = CONFIG_LCD_HRES,
        .y_max = CONFIG_LCD_VRES,
        .rst_gpio_num = (gpio_num_t)_rst,
        .int_gpio_num = (gpio_num_t)_int,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 1,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller gsl3680");
    ret = gsl3680_touch_init_local(&bus, &tp_cfg, &tp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gsl3680_touch_init_local failed: %s", esp_err_to_name(ret));
        tp = nullptr;
        return false;
    }

    return true;
}


/* Spurious-contact filter -------------------------------------------------
 *
 * The GSL3680 on these panels intermittently reports a phantom contact: it
 * parks at a fixed spot near one end of an axis, holds perfectly still there
 * for ~150ms, then vanishes.  Measured on a 10.1 inch panel, 65 of 709 traced
 * touch samples during ordinary dragging carried one.
 *
 * With the point-0 clobber in esp_lcd_touch_gsl3680_get_xy() fixed, a phantom
 * normally lands in slot 1 and is ignored here, since this call only ever
 * reads the first contact.  Nothing guarantees it cannot take slot 0 though,
 * so distance is checked as well: a contact that teleports did not get there
 * by finger.
 *
 * Thresholds come from measurement, not taste.  Real finger travel between two
 * reads 16ms apart: median 7 raw units, 99th percentile 180, maximum 187.  The
 * phantom jumps measured 445 to 545.  350 sits in that gap with room either
 * side, and both panels this driver serves are 800x1280, so one raw-coordinate
 * threshold covers every orientation and every lv_setup glue on top.
 */
static const uint32_t GSL3680_MAX_STEP_RAW = 350;

/* How many reads a suspect contact may be held before the touch is dropped
 * instead.  Six reads is ~100ms - longer than any plausible real outlier,
 * shorter than the phantom's dwell.  Dropping is the deliberate choice for the
 * failure case: the worst a phantom can then do is end a drag early, where
 * before it would drag the control to whichever extreme the phantom sat at. */
static const uint8_t GSL3680_MAX_HOLD = 6;

void gsl3680_touch::_resetFilter()
{
    _histCount = 0;
    _histPos   = 0;
    _haveTouch = false;
    _holdCount = 0;
    _missCount = 0;
}

uint16_t gsl3680_touch::_median(const uint16_t *hist) const
{
    // Insertion sort of a tiny fixed array, then the middle element.  A median
    // is used rather than an average because the noise here is impulsive: an
    // average blends a spike into the result, while a median discards it
    // outright as long as fewer than half the window is bad.
    uint16_t buf[MEDIAN_N];
    for (uint8_t i = 0; i < _histCount; i++) buf[i] = hist[i];
    for (uint8_t i = 1; i < _histCount; i++) {
        uint16_t v = buf[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && buf[j] > v) { buf[j + 1] = buf[j]; j--; }
        buf[j + 1] = v;
    }
    return buf[_histCount / 2];
}

bool gsl3680_touch::getTouch(uint16_t *x, uint16_t *y, uint8_t *fingersOut)
{
    // esp_lcd_touch_read_data() asserts on a NULL handle, so bail out if begin()
    // failed and the caller chose to run with touch disabled.
    if (tp == nullptr || _suspended) {
        if (fingersOut) *fingersOut = 0;
        _resetFilter();
        return false;
    }

    // A read that FAILS is not the same as a read that says "no finger", and
    // conflating the two is what makes touch fall apart in an electrically
    // noisy installation.  esp_lcd_touch_read_data() now reports I2C failures
    // instead of feeding garbage to the tracker, so honour that here.
    uint16_t sx = 0, sy = 0;
    bool have_sample = false;

    touch_cnt = 0;
    if (esp_lcd_touch_read_data(tp) == ESP_OK) {
        have_sample = esp_lcd_touch_get_coordinates(tp, x, y, touch_strength, &touch_cnt, 1);
        sx = *x;
        sy = *y;
    }
    if (fingersOut) *fingersOut = have_sample ? touch_cnt : 0;

    // ---- 1. dropouts do not end a press ---------------------------------
    if (!have_sample) {
        if (_haveTouch && _missCount < RELEASE_DEBOUNCE) {
            _missCount++;
            *x = _outX;
            *y = _outY;
            return true;
        }
        _resetFilter();
        return false;
    }
    _missCount = 0;

    // ---- 2. a contact that teleports did not get there by finger ---------
    // The median below absorbs a spike or two, but not a phantom that parks
    // somewhere and holds - those were measured lasting nine reads, long
    // enough to carry a median window with them.  Distance catches those.
    if (_haveTouch) {
        int32_t dx = (int32_t)sx - (int32_t)_lastRawX; if (dx < 0) dx = -dx;
        int32_t dy = (int32_t)sy - (int32_t)_lastRawY; if (dy < 0) dy = -dy;
        if ((uint32_t)dx > GSL3680_MAX_STEP_RAW || (uint32_t)dy > GSL3680_MAX_STEP_RAW) {
            if (_holdCount < GSL3680_MAX_HOLD) {
                _holdCount++;
                *x = _outX;
                *y = _outY;
                return true;
            }
            _resetFilter();
            return false;
        }
    }
    _holdCount = 0;
    _lastRawX = sx;
    _lastRawY = sy;

    // ---- 3. sliding median ----------------------------------------------
    if (!_haveTouch) {
        // First sample of a press: seed the whole window with it rather than
        // ramping up from one entry, so the reported point starts where the
        // finger actually landed instead of drifting into place.
        for (uint8_t i = 0; i < MEDIAN_N; i++) { _histX[i] = sx; _histY[i] = sy; }
        _histCount = MEDIAN_N;
        _histPos   = 0;
    } else {
        _histX[_histPos] = sx;
        _histY[_histPos] = sy;
        _histPos = (uint8_t)((_histPos + 1) % MEDIAN_N);
        if (_histCount < MEDIAN_N) _histCount++;
    }

    _outX = _median(_histX);
    _outY = _median(_histY);
    _haveTouch = true;

    *x = _outX;
    *y = _outY;
    return true;
}

void gsl3680_touch::suspend()
{
    if (_rst < 0) return;
    gpio_set_level((gpio_num_t)_rst, 0);
    _suspended = true;
    _resetFilter();
}

bool gsl3680_touch::resume(bool verify)
{
    if (tp == nullptr) return false;
    _suspended = false;
    _resetFilter();
    return gsl3680_touch_restart_local(tp, verify) == ESP_OK;
}

void gsl3680_touch::set_rotation(uint8_t r){
if (tp == nullptr) return;
switch(r){
    case 0:
        esp_lcd_touch_set_swap_xy(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        esp_lcd_touch_set_mirror_y(tp, false);
        break;
    case 1:
        esp_lcd_touch_set_swap_xy(tp, false);
        esp_lcd_touch_set_mirror_x(tp, true);
        esp_lcd_touch_set_mirror_y(tp, true);
        break;
    case 2:
        esp_lcd_touch_set_swap_xy(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        esp_lcd_touch_set_mirror_y(tp, false);
        break;
    case 3:
        esp_lcd_touch_set_swap_xy(tp, false);
        esp_lcd_touch_set_mirror_x(tp, true);
        esp_lcd_touch_set_mirror_y(tp, true);
        break;
    }

}

#endif // ARDUINO_ESP32P4_DEV
