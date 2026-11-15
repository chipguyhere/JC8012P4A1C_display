#ifdef ARDUINO_ESP32P4_DEV

#ifndef _GT911_TOUCH_H
#define _GT911_TOUCH_H
#include <stdio.h>
#include "esp_err.h"

// The shared I2C bus (GPIO7/8) belongs to the board library: the GSL3680 sits on
// it alongside the ES8311 codec, the RX8025T RTC, the camera and the headers.
#include <JC8012P4A1C_Board.h>

class gsl3680_touch
{
public:
    // Takes the board's shared, mutex-protected bus.  The bus must already be
    // started (board.begin()) by the time begin() is called.
    gsl3680_touch(ThreadsafeWire &wire, int8_t rst_pin = -1, int8_t int_pin = -1);

    // Returns false if the controller failed to come up.  The object stays
    // safe to use either way: getTouch() then reports no touches and
    // set_rotation() is a no-op.
    bool begin();

    // Reports the first tracked contact in raw panel coordinates (800x1280).
    // Spurious contacts are filtered out - see the note in getTouch().
    // fingersOut, when supplied, receives how many contacts the controller
    // claimed for this read, which is worth logging when touch misbehaves.
    bool getTouch(uint16_t *x, uint16_t *y, uint8_t *fingersOut = nullptr);
    void set_rotation(uint8_t r);

    // Low-power control.  suspend() holds the controller in reset — it stops
    // scanning (measured: ~25-40 mA saved) and reports no touches.  resume()
    // pulses reset and restarts it WITHOUT re-downloading the firmware (the
    // vendor init does the same after its download), then checks the firmware
    // word; it returns false if the firmware was lost, in which case call
    // begin() again for a full (~1 s) re-init.  resume() takes ~110 ms, or
    // ~70 ms with verify=false (skips the check; the firmware has been measured
    // to survive a held reset of at least 2 minutes).
    //
    // After resume(), the controller runs ~10 fast calibration scans (~90 ms)
    // and a finger is first reported ~174 ms later — and a finger that was
    // ALREADY resting on the screen at resume() is absorbed into that
    // calibration and never reported (measured).  Wake-by-touch built on this
    // must therefore look for a MOVING finger.
    void suspend();
    bool resume(bool verify = true);
    bool isSuspended() const { return _suspended; }

    // Length of the sliding median window getTouch() smooths through.  7 is
    // what proved effective on the resistive CYD panel, but note the two are
    // not the same trade: there the 7 samples are independent ADC conversions
    // taken inside a single read and cost nothing, whereas this controller
    // latches one processed coordinate per read, so the window spans 7
    // successive reads and the output trails the finger by 3 of them - roughly
    // 50ms at a 16ms read period.  Fine for a wall control panel, and the
    // number to lower first if anything ever needs to feel snappier.
    static const uint8_t MEDIAN_N = 7;

    // Consecutive failed or empty reads tolerated before a release is believed.
    // Electrical noise makes reads drop out; a finger does not flicker.  Four
    // reads is ~65ms, well under the gap between two deliberate taps, so a real
    // lift is still seen as a lift.
    static const uint8_t RELEASE_DEBOUNCE = 4;

private:
    ThreadsafeWire &_wire;
    int8_t _rst, _int;
    bool _suspended = false;

    // gsl3680_bus_t hooks: the vendored C driver calls these for every
    // register access, and they go through the shared bus's mutex.
    static esp_err_t _busRead(void *ctx, uint8_t reg, uint8_t *data, uint8_t len);
    static esp_err_t _busWrite(void *ctx, uint8_t reg, const uint8_t *data, uint8_t len);

    // Filter state, all in raw panel coordinates so it is independent of
    // whatever rotation or scaling the glue above applies.
    uint16_t _histX[MEDIAN_N] = {0};   // sliding window of accepted samples
    uint16_t _histY[MEDIAN_N] = {0};
    uint8_t  _histCount = 0;           // valid entries, ramps up at press start
    uint8_t  _histPos = 0;             // ring write position
    uint16_t _outX = 0, _outY = 0;     // last value handed to the caller
    uint16_t _lastRawX = 0, _lastRawY = 0;  // last accepted raw, for the jump test
    bool     _haveTouch = false;
    uint8_t  _holdCount = 0;           // consecutive jump rejections
    uint8_t  _missCount = 0;           // consecutive dropouts

    void     _resetFilter();
    uint16_t _median(const uint16_t *hist) const;
};

#endif

#endif // ARDUINO_ESP32P4_DEV
