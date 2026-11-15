/*
 * gsl3680_touch - GSL3680 capacitive touch on the Guition JC8012P4A1C.
 *
 * The GSL3680 has no firmware of its own: it runs from RAM, and the host
 * uploads ~18 KB of firmware every time it powers up.  This library does not
 * contain that firmware.  It reads it from NVS, where a one-time installer
 * sketch put it (the InstallFirmware example of the
 * chipguy_ESP32-P4-C6-firmware-installer library), checks it against a
 * SHA-256 compiled in here, and uploads it.  Without it the display works and
 * touch reports nothing; status() says why.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 */
#ifdef ARDUINO_ESP32P4_DEV

#ifndef CHIPGUY_GSL3680_TOUCH_H
#define CHIPGUY_GSL3680_TOUCH_H
#include <stdint.h>
#include "esp_err.h"

// The shared I2C bus (GPIO7/8) belongs to the board library: the GSL3680 sits on
// it alongside the ES8311 codec, the RX8025T RTC, the camera and the headers.
#include <JC8012P4A1C_Board.h>

class gsl3680_touch
{
public:
    // Why begin() succeeded or failed.
    enum Status : uint8_t {
        NOT_STARTED,      // begin() not called yet
        OK,
        NO_FIRMWARE,      // nothing in NVS: run the installer sketch
        BAD_FIRMWARE,     // NVS holds something, but not the expected image
        NO_MEMORY,
        NO_RESPONSE,      // the controller did not answer on I2C
        NOT_RUNNING,      // firmware uploaded, but the controller didn't start it
    };

    // Where the installer puts the firmware: NVS namespace and key.  The value
    // is the zlib-compressed image described in gsl3680_touch.cpp.
    static constexpr const char *NVS_NAMESPACE = "jc8012";
    static constexpr const char *NVS_KEY       = "gsl3680fw";

    // Takes the board's shared, mutex-protected bus.  The bus must already be
    // started (board.begin()) by the time begin() is called.
    gsl3680_touch(ThreadsafeWire &wire, int8_t rst_pin = -1, int8_t int_pin = -1);

    // Loads the firmware from NVS, uploads it and starts the controller (~1 s).
    // Returns false if touch is unavailable; status() says why, and the object
    // stays safe to use either way: getTouch() then reports no touches.
    bool begin();
    Status status() const { return _status; }
    static const char *statusText(Status s);

    // True for the failures a retry or a reboot could fix (the controller not
    // answering), false for those it can't (firmware missing from NVS).
    bool retryMightHelp() const { return _status == NO_RESPONSE || _status == NOT_RUNNING; }

    // Reports the tracked contact in native panel coordinates (800x1280,
    // portrait).  Spurious contacts are filtered out - see getTouch().
    // fingersOut, when supplied, receives how many contacts the controller
    // reported for this read (up to 5), which is worth logging when touch
    // misbehaves.
    bool getTouch(uint16_t *x, uint16_t *y, uint8_t *fingersOut = nullptr);

    // Kept for source compatibility; rotation is the display's mapTouch()'s job.
    void set_rotation(uint8_t) {}

    // Low-power control.  suspend() holds the controller in reset - it stops
    // scanning (measured: ~25-40 mA saved) and reports no touches.  resume()
    // pulses reset and restarts it WITHOUT re-uploading the firmware, then
    // checks that the firmware is still running; it returns false if it was
    // lost, in which case call begin() again for a full (~1 s) re-init.
    // resume() takes ~110 ms, or ~70 ms with verify=false (skips the check;
    // the firmware has been measured to survive a held reset of at least
    // 2 minutes).
    //
    // After resume(), the controller runs ~10 fast calibration scans (~90 ms)
    // and a finger is first reported ~174 ms later - and a finger that was
    // ALREADY resting on the screen at resume() is absorbed into that
    // calibration and never reported (measured).  Wake-by-touch built on this
    // must therefore look for a MOVING finger.
    void suspend();
    bool resume(bool verify = true);
    bool isSuspended() const { return _suspended; }

    // Steadiness filter: the jump rejection and the sliding median in
    // getTouch().  OFF by default, which reports each sample as read - the
    // normal behavior of a touch driver.  Turn it on for a UI where a phantom
    // jump would do real harm (a volume slider dragged to its extreme) and
    // fast swipes don't matter; the cost is ~50 ms of lag and a contact that
    // moves >350 px between reads being held, then dropped.  Following one
    // finger, skipping flagged records and repeated scans, and the release
    // debounce (RELEASE_MS) apply either way: they add no lag.
    void setFilter(bool on) { _filter = on; _resetFilter(); }
    bool filter() const { return _filter; }

    // Length of the filter's sliding median.  This controller latches one
    // coordinate per scan, so the window spans 7 successive reads and the
    // output trails the finger by 3 of them - roughly 50 ms at a 16 ms poll.
    static const uint8_t MEDIAN_N = 7;

    // How long failed or empty reads are ridden out before a release is
    // believed.  Electrical noise makes reads drop out; a finger does not
    // flicker.  65 ms is well under the gap between two deliberate taps, so a
    // real lift is still seen as a lift.  Measured in time, not reads: a
    // caller that polls slowly (LVGL behind a heavy frame) must not stretch a
    // tap into a drag that swallows the next one.
    static const uint32_t RELEASE_MS = 65;

private:
    ThreadsafeWire &_wire;
    int8_t _rst, _int;
    bool   _suspended = false;
    Status _status = NOT_STARTED;

    bool _read(uint8_t reg, uint8_t *data, uint8_t len);
    bool _write(uint8_t reg, const uint8_t *data, uint8_t len);
    bool _write8(uint8_t reg, uint8_t v) { return _write(reg, &v, 1); }
    void _resetPulse();
    void _halt();
    void _start();
    bool _running();
    uint8_t *_loadImage(size_t *len);
    bool _upload(const uint8_t *img, size_t len);

    // Last report's frame counter: a read that returns the same scan again
    // carries no new information and is not fed to the filter twice.
    uint32_t _lastFrame = 0xFFFFFFFF;
    uint8_t  _lastCount = 0;
    bool     _lastResult = false;
    bool     _filter = false;

    // Filter state, in native panel coordinates.
    uint16_t _histX[MEDIAN_N] = {0};   // sliding window of accepted samples
    uint16_t _histY[MEDIAN_N] = {0};
    uint8_t  _histCount = 0;           // valid entries
    uint8_t  _histPos = 0;             // ring write position
    uint16_t _outX = 0, _outY = 0;     // last value handed to the caller
    uint16_t _lastRawX = 0, _lastRawY = 0;  // last accepted sample: jump test and tracking
    bool     _haveTouch = false;
    int64_t  _goodUs = 0;              // when a contact was last read
    int64_t  _holdUs = 0;              // when the current jump rejection began (0 = none)
    int64_t  _frameUs = 0;             // when the frame counter last advanced

    void     _resetFilter();
    uint16_t _median(const uint16_t *hist) const;
};

#endif

#endif // ARDUINO_ESP32P4_DEV
