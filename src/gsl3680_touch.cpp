/*
 * gsl3680_touch - GSL3680 capacitive touch on the Guition JC8012P4A1C.
 * See gsl3680_touch.h for the overview and README-engineering.md ("Touch")
 * for the measurements behind the numbers here.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 */
#ifdef ARDUINO_ESP32P4_DEV

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "nvs.h"
#include "mbedtls/sha256.h"
#include "miniz.h"              // ROM inflate (tinfl), esp_rom component
#include "gsl3680_touch.h"

static const char *TAG = "gsl3680";

/* The firmware image --------------------------------------------------------
 *
 * The controller's RAM is reached in 128-byte pages: write the page number to
 * register 0xF0, then the page's 32 little-endian words to registers 0x00,
 * 0x04 ... 0x7C.  The image is just that, page after page:
 *
 *     uint32 page number, then uint32 word[32]          (132 bytes a page)
 *
 * all little-endian, 139 pages, 18348 bytes.  NVS holds it zlib-compressed
 * (~9.9 KB; the P4's default NVS partition is 20 KB, too small for it raw),
 * and it is inflated with the copy of miniz in the P4's ROM.  Only an image
 * whose SHA-256 matches IMAGE_SHA256 is uploaded: the hash names exactly the
 * firmware this driver was measured with, so a corrupt or different blob is
 * refused rather than half-working.
 */
static const size_t   PAGE_WORDS  = 32;
static const size_t   PAGE_BYTES  = 4 + PAGE_WORDS * 4;
static const size_t   IMAGE_PAGES = 139;
static const size_t   IMAGE_BYTES = IMAGE_PAGES * PAGE_BYTES;   // 18348
static const uint8_t  IMAGE_SHA256[32] = {
    0x65, 0x08, 0x76, 0xe3, 0xe1, 0xb0, 0x73, 0x93, 0xab, 0x5e, 0xd4, 0x9d, 0x56, 0xbd, 0x9f, 0x6b,
    0x1d, 0x62, 0x76, 0x59, 0x96, 0x5c, 0x78, 0xfe, 0x25, 0xcc, 0xba, 0x45, 0x6a, 0x59, 0x40, 0x06,
};
static const size_t   BLOB_MAX = 32 * 1024;     // sanity bound on what NVS hands back

/* Registers --------------------------------------------------------------- */
static const uint8_t REG_TOUCH   = 0x80;   // count, frame counter, then 5 x 4-byte contacts
static const uint8_t REG_STATUS  = 0xB0;   // reads 5A 5A 5A 5A once the firmware runs
static const uint8_t REG_CLEAR   = 0xBC;
static const uint8_t REG_CTRL    = 0xE0;   // 0x88 = halt the core, 0x00 = run
static const uint8_t REG_E4      = 0xE4;
static const uint8_t REG_88      = 0x88;
static const uint8_t REG_PAGE    = 0xF0;

/* Report format and coordinate mapping -------------------------------------
 *
 * REG_TOUCH: byte 0 = contacts, bytes 1-3 = a frame counter that advances once
 * per scan, then one 4-byte record per contact:
 *     bytes 0-1   "Y": 12 bits of position; the top bits are a flag (seen:
 *                 0x4xxx, briefly at touch-down and lift) - such a record is
 *                 not a usable position
 *     bytes 2-3   "X": 12 bits of position, top 4 bits a finger ID (always 0
 *                 with this firmware, which leaves tracking to the host)
 *
 * The controller's axes are the panel's swapped, in sensor units.  Its config
 * describes a 14 x 26 electrode grid on the 800 x 1280 panel, and at 64 units
 * per electrode that makes
 *     native x = Y * 800 / 896          native y = 1280 - X * 1280 / 1664
 * which is what a fit to 26 measured targets on the bench found (x: 0.879 to
 * 0.890 per unit, y: -0.771 to -0.778; r > 0.9997).  Contacts reach every edge.
 */
static const uint32_t SENSOR_Y_RANGE = 14 * 64;   // 896  -> native x
static const uint32_t SENSOR_X_RANGE = 26 * 64;   // 1664 -> native y
static const int      NATIVE_W = 800, NATIVE_H = 1280;
static const uint8_t  MAX_CONTACTS = 5;

/* Spurious-contact filter (setFilter(true); off by default) ---------------
 *
 * The GSL3680 on these panels intermittently reports a phantom contact: it
 * parks at a fixed spot near one end of an axis, holds perfectly still there
 * for ~150ms, then vanishes.  Measured on a 10.1 inch panel, 65 of 709 traced
 * touch samples during ordinary dragging carried one.
 *
 * getTouch() follows ONE finger: the contact nearest the previous sample (the
 * controller's slot order is not stable - two fingers swapped slots 24 times
 * in 1546 measured frames - so "slot 0" is not a finger).  A phantom elsewhere
 * is therefore normally just not chosen.  Distance is checked as well: a
 * contact that teleports did not get there by finger.
 *
 * Thresholds come from measurement, not taste.  Real finger travel between two
 * reads 16ms apart: median 7 px, 99th percentile 180, maximum 187.  The
 * phantom jumps measured 445 to 545.  350 sits in that gap with room either
 * side, in native pixels, so it covers every orientation and every lv_setup
 * glue on top.
 */
static const uint32_t GSL3680_MAX_STEP = 350;

/* How long a suspect contact may be held before the touch is dropped
 * instead.  100ms (six reads at 16ms) - longer than any plausible real outlier,
 * shorter than the phantom's dwell.  Dropping is the deliberate choice for the
 * failure case: the worst a phantom can then do is end a drag early, where
 * before it would drag the control to whichever extreme the phantom sat at. */
static const int64_t GSL3680_MAX_HOLD_US = 100 * 1000;

/* A read that repeats the previous scan's frame counter is answered from the
 * previous result.  The controller scans about every 20 ms (measured), so a
 * counter that hasn't moved for this long means it stopped scanning, and the
 * read counts as a dropout instead - a stall can't hold a press forever. */
static const int64_t GSL3680_STALL_US = 150 * 1000;

// ---------------------------------------------------------------------------

gsl3680_touch::gsl3680_touch(ThreadsafeWire &wire, int8_t rst_pin, int8_t int_pin)
    : _wire(wire), _rst(rst_pin), _int(int_pin)
{
}

const char *gsl3680_touch::statusText(Status s)
{
    switch (s) {
        case NOT_STARTED:  return "not started";
        case OK:           return "OK";
        case NO_FIRMWARE:  return "touch firmware not installed in NVS";
        case BAD_FIRMWARE: return "touch firmware in NVS is damaged or not the expected version";
        case NO_MEMORY:    return "out of memory";
        case NO_RESPONSE:  return "controller not responding on I2C";
        case NOT_RUNNING:  return "controller did not start its firmware";
    }
    return "?";
}

bool gsl3680_touch::_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    // readRegister() holds the bus across the register write, the repeated
    // start and the drain of Wire's RX buffer, so no other task can interleave.
    return _wire.readRegister(JC8012P4A1C_Board::GSL3680_ADDR, reg, data, len);
}

bool gsl3680_touch::_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    return _wire.writeRegister(JC8012P4A1C_Board::GSL3680_ADDR, reg, data, len);
}

// Reset pulse, then hold the core and clear its state, ready for an upload or
// a restart.
void gsl3680_touch::_resetPulse()
{
    if (_rst >= 0) {
        gpio_set_level((gpio_num_t)_rst, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level((gpio_num_t)_rst, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    _write8(REG_CTRL, 0x88);
    vTaskDelay(pdMS_TO_TICKS(10));
    _write8(REG_E4, 0x04);
    vTaskDelay(pdMS_TO_TICKS(10));
    const uint8_t zero[4] = {0, 0, 0, 0};
    _write(REG_CLEAR, zero, 4);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Clear the core's registers (once, before the first upload after power-up).
void gsl3680_touch::_halt()
{
    _write8(REG_CTRL, 0x88);
    vTaskDelay(pdMS_TO_TICKS(20));
    _write8(REG_88, 0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
    _write8(REG_E4, 0x04);
    vTaskDelay(pdMS_TO_TICKS(5));
    _write8(REG_CTRL, 0x00);
    vTaskDelay(pdMS_TO_TICKS(20));
}

void gsl3680_touch::_start()
{
    _write8(REG_CTRL, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
}

bool gsl3680_touch::_running()
{
    vTaskDelay(pdMS_TO_TICKS(30));
    uint8_t b[4] = {0};
    if (!_read(REG_STATUS, b, 4)) return false;
    return b[0] == 0x5A && b[1] == 0x5A && b[2] == 0x5A && b[3] == 0x5A;
}

// Read, inflate and hash-check the firmware image from NVS.  Returns a
// heap buffer of IMAGE_BYTES (caller frees) or nullptr with _status set.
uint8_t *gsl3680_touch::_loadImage(size_t *len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        // A namespace nobody has written yet reads as NOT_FOUND.
        if (err != ESP_ERR_NVS_NOT_FOUND) ESP_LOGE(TAG, "nvs_open: %s", esp_err_to_name(err));
        _status = NO_FIRMWARE;
        return nullptr;
    }
    size_t blobLen = 0;
    err = nvs_get_blob(h, NVS_KEY, nullptr, &blobLen);
    if (err != ESP_OK || blobLen == 0 || blobLen > BLOB_MAX) {
        nvs_close(h);
        _status = (err == ESP_ERR_NVS_NOT_FOUND) ? NO_FIRMWARE : BAD_FIRMWARE;
        return nullptr;
    }

    uint8_t *blob = (uint8_t *)malloc(blobLen);
    uint8_t *img  = (uint8_t *)heap_caps_malloc(IMAGE_BYTES, MALLOC_CAP_8BIT);
    // The inflater's state is ~11 KB: too big for a task stack.
    tinfl_decompressor *inf = (tinfl_decompressor *)malloc(sizeof(tinfl_decompressor));
    if (!blob || !img || !inf) {
        free(blob); free(img); free(inf);
        nvs_close(h);
        _status = NO_MEMORY;
        return nullptr;
    }
    err = nvs_get_blob(h, NVS_KEY, blob, &blobLen);
    nvs_close(h);

    bool good = false;
    if (err == ESP_OK) {
        tinfl_init(inf);
        size_t inLen = blobLen, outLen = IMAGE_BYTES;
        tinfl_status st = tinfl_decompress(inf, blob, &inLen, img, img, &outLen,
            TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
        if (st == TINFL_STATUS_DONE && outLen == IMAGE_BYTES) {
            uint8_t sha[32];
            mbedtls_sha256(img, IMAGE_BYTES, sha, 0);
            good = memcmp(sha, IMAGE_SHA256, 32) == 0;
        }
    }
    free(blob);
    free(inf);
    if (!good) {
        free(img);
        _status = BAD_FIRMWARE;
        return nullptr;
    }
    *len = IMAGE_BYTES;
    return img;
}

bool gsl3680_touch::_upload(const uint8_t *img, size_t len)
{
    uint32_t errors = 0;
    for (size_t off = 0; off + PAGE_BYTES <= len; off += PAGE_BYTES) {
        const uint8_t *p = img + off;
        if (!_write8(REG_PAGE, p[0])) errors++;            // page numbers fit a byte
        for (size_t w = 0; w < PAGE_WORDS; w++) {
            if (!_write((uint8_t)(w * 4), p + 4 + w * 4, 4)) errors++;
        }
    }
    if (errors) ESP_LOGE(TAG, "firmware upload: %lu I2C write errors", (unsigned long)errors);
    return errors == 0;
}

// Bring up the controller.  Returns false instead of aborting: the caller is
// better placed than we are to decide whether a failure is worth a retry (see
// retryMightHelp()) or whether the display should just come up without touch.
bool gsl3680_touch::begin()
{
    _resetFilter();
    _suspended = false;
    _lastFrame = 0xFFFFFFFF;

    size_t imgLen = 0;
    uint8_t *img = _loadImage(&imgLen);
    if (!img) {
        if (_status == NO_FIRMWARE || _status == BAD_FIRMWARE) {
            ESP_LOGE(TAG, "%s. Touch is disabled. Install it once with the "
                          "InstallFirmware example of the "
                          "chipguy_ESP32-P4-C6-firmware-installer library.",
                     statusText(_status));
        } else {
            ESP_LOGE(TAG, "%s", statusText(_status));
        }
        return false;
    }

    // Power-up reset.  INT is held low through it, as the vendor's bring-up
    // does, then released to an input.
    if (_rst >= 0) {
        gpio_config_t rc = {};
        rc.mode = GPIO_MODE_OUTPUT;
        rc.pin_bit_mask = BIT64(_rst);
        gpio_config(&rc);
    }
    if (_int >= 0) {
        gpio_config_t ic = {};
        ic.mode = GPIO_MODE_OUTPUT;
        ic.pull_up_en = GPIO_PULLUP_ENABLE;
        ic.pin_bit_mask = BIT64(_int);
        gpio_config(&ic);
        gpio_set_level((gpio_num_t)_int, 0);
    }
    if (_rst >= 0) {
        gpio_set_level((gpio_num_t)_rst, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)_rst, 1);
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    if (_int >= 0) {
        gpio_config_t ic = {};
        ic.mode = GPIO_MODE_INPUT;
        ic.pin_bit_mask = BIT64(_int);
        gpio_config(&ic);
    }

    uint8_t probe[4];
    if (!_read(REG_PAGE, probe, 4)) {
        free(img);
        _status = NO_RESPONSE;
        ESP_LOGE(TAG, "%s", statusText(_status));
        return false;
    }

    uint32_t t0 = (uint32_t)(esp_log_timestamp());
    _halt();
    _resetPulse();
    bool wrote = _upload(img, imgLen);
    free(img);
    _start();
    _resetPulse();
    _start();

    if (!wrote) {
        _status = NO_RESPONSE;
    } else if (!_running()) {
        _status = NOT_RUNNING;
    } else {
        _status = OK;
    }
    if (_status == OK) {
        ESP_LOGI(TAG, "firmware uploaded and running (%lu ms)",
                 (unsigned long)(esp_log_timestamp() - t0));
        return true;
    }
    ESP_LOGE(TAG, "%s", statusText(_status));
    return false;
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
    if (_status != OK) return false;
    _suspended = false;
    _resetFilter();
    _lastFrame = 0xFFFFFFFF;
    _resetPulse();
    _start();
    return verify ? _running() : true;
}

void gsl3680_touch::_resetFilter()
{
    _histCount = 0;
    _histPos   = 0;
    _haveTouch = false;
    _holdUs = 0;
    _lastResult = false;
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
    if (fingersOut) *fingersOut = 0;
    if (_status != OK || _suspended) {
        _resetFilter();
        return false;
    }

    // A read that FAILS is not the same as a read that says "no finger", and
    // conflating the two is what makes touch fall apart in an electrically
    // noisy installation.  A failed read is a dropout, ridden out below.
    const int64_t now = esp_timer_get_time();
    uint8_t d[4 + 4 * MAX_CONTACTS];
    bool readOk = _read(REG_TOUCH, d, sizeof(d));

    if (readOk) {
        uint32_t frame = (uint32_t)d[1] | ((uint32_t)d[2] << 8) | ((uint32_t)d[3] << 16);
        if (frame == _lastFrame) {
            if (now - _frameUs < GSL3680_STALL_US) {
                // The same scan as last time: no new information.
                if (_lastResult) { *x = _outX; *y = _outY; }
                if (fingersOut) *fingersOut = _lastCount;
                return _lastResult;
            }
            readOk = false;                          // stalled: treat as a dropout
        } else {
            _lastFrame = frame;
            _frameUs = now;
        }
    }

    // Decode the usable contacts into native coordinates.
    uint16_t cx[MAX_CONTACTS], cy[MAX_CONTACTS];
    uint8_t n = 0;
    if (readOk) {
        uint8_t count = d[0] > MAX_CONTACTS ? MAX_CONTACTS : d[0];
        for (uint8_t i = 0; i < count; i++) {
            const uint8_t *p = d + 4 + 4 * i;
            uint16_t sy = (uint16_t)(p[0] | (p[1] << 8));
            uint16_t sx = (uint16_t)(p[2] | ((p[3] & 0x0F) << 8));
            if (sy & 0xF000) continue;                   // flagged: not a position
            int32_t nx = ((int32_t)sy * NATIVE_W + (int32_t)SENSOR_Y_RANGE / 2) / (int32_t)SENSOR_Y_RANGE;
            int32_t ny = NATIVE_H - ((int32_t)sx * NATIVE_H + (int32_t)SENSOR_X_RANGE / 2) / (int32_t)SENSOR_X_RANGE;
            if (nx < 0) nx = 0; else if (nx >= NATIVE_W) nx = NATIVE_W - 1;
            if (ny < 0) ny = 0; else if (ny >= NATIVE_H) ny = NATIVE_H - 1;
            cx[n] = (uint16_t)nx;
            cy[n] = (uint16_t)ny;
            n++;
        }
        _lastCount = count;
        if (fingersOut) *fingersOut = count;
    }

    // ---- 1. dropouts do not end a press ---------------------------------
    if (n == 0) {
        if (_haveTouch && now - _goodUs < (int64_t)RELEASE_MS * 1000) {
            *x = _outX;
            *y = _outY;
            _lastResult = true;
            return true;
        }
        _resetFilter();
        return false;
    }
    _goodUs = now;

    // ---- 2. follow the same finger --------------------------------------
    uint8_t pick = 0;
    if (_haveTouch) {
        uint32_t best = UINT32_MAX;
        for (uint8_t i = 0; i < n; i++) {
            int32_t dx = (int32_t)cx[i] - _lastRawX, dy = (int32_t)cy[i] - _lastRawY;
            uint32_t dd = (uint32_t)(dx * dx + dy * dy);
            if (dd < best) { best = dd; pick = i; }
        }
    }
    uint16_t sx = cx[pick], sy = cy[pick];

    if (!_filter) {
        _lastRawX = _outX = sx;
        _lastRawY = _outY = sy;
        _haveTouch = true;
        _lastResult = true;
        *x = sx;
        *y = sy;
        return true;
    }

    // ---- 3. a contact that teleports did not get there by finger ---------
    // The median below absorbs a spike or two, but not a phantom that parks
    // somewhere and holds - those were measured lasting nine reads, long
    // enough to carry a median window with them.  Distance catches those.
    if (_haveTouch) {
        int32_t dx = (int32_t)sx - (int32_t)_lastRawX; if (dx < 0) dx = -dx;
        int32_t dy = (int32_t)sy - (int32_t)_lastRawY; if (dy < 0) dy = -dy;
        if ((uint32_t)dx > GSL3680_MAX_STEP || (uint32_t)dy > GSL3680_MAX_STEP) {
            if (!_holdUs) _holdUs = now;
            if (now - _holdUs < GSL3680_MAX_HOLD_US) {
                *x = _outX;
                *y = _outY;
                _lastResult = true;
                return true;
            }
            _resetFilter();
            return false;
        }
    }
    _holdUs = 0;
    _lastRawX = sx;
    _lastRawY = sy;

    // ---- 4. sliding median ----------------------------------------------
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
    _lastResult = true;

    *x = _outX;
    *y = _outY;
    return true;
}

#endif // ARDUINO_ESP32P4_DEV
