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
 *   rotation 1  -> landscape (1280x800, picture turned 90 deg clockwise)
 *   rotation 2  -> portrait flipped  (800x1280, 180 deg)
 *   rotation 3  -> landscape flipped (1280x800, picture turned 90 deg CCW)
 *
 * Degrees (0/90/180/270) are accepted as aliases for the indices 0/1/2/3.
 *
 * Board wiring (fixed on the JC8012P4A1C board) — the pin map lives in
 * JC8012P4A1C_Board (chipguy_JC8012P4A1C_board); pins_config.h aliases it:
 *   LCD: MIPI-DSI (2 data lanes), RST GPIO27, backlight GPIO23 (active HIGH)
 *   Touch (GSL3680, I2C 0x40): RST GPIO22, INT GPIO21, on the board's SHARED
 *   I2C bus (GPIO7/8, also the codec, RTC, camera and headers), which the
 *   board object owns — so this driver takes a JC8012P4A1C_Board&.
 *
 * Copyright (c) 2025-2026 chipguyhere
 * MIT License
 */

#pragma once

#ifdef ARDUINO_ESP32P4_DEV

#ifndef BOARD_HAS_PSRAM
#error "This library requires PSRAM. Enable PSRAM in the Arduino IDE Tools menu."
#endif

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "driver/ppa.h"
#include <JC8012P4A1C_Board.h>   // shared I2C bus + pin map
#include "gsl3680_touch.h"
// pins_config.h is included by the .cpp only, so its short macro names
// (LCD_RST, TP_INT, ...) don't leak into sketches.

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

#ifndef JC8012_PANEL_TIMING_DEFINED   // shared with the sibling _display/_480display header
#define JC8012_PANEL_TIMING_DEFINED
// Panel video timing: pixel clock and porches.  The refresh rate is
//   pixelClockHz / ((800 + hsync + hbp + hfp) * (1280 + vsync + vbp + vfp)).
// The pixel clock is PLL_F240M or PLL_F160M divided by a whole number, so it
// must be a whole number of MHz that divides 240 or 160 MHz evenly (60, 80,
// 120, 40, ...).  begin() falls back to JC8012_TIMING_DEFAULT otherwise.
// (The P4's fractional APLL is selectable as the DPI clock, but on this
// board it gives no frames at all; see README-engineering.md.)
struct JC8012PanelTiming {
    uint32_t pixelClockHz;
    uint16_t hsync, hbp, hfp;
    uint16_t vsync, vbp, vfp;
};

// The default: 60 MHz with the panel vendor's porches, 880 x 1312 total
// = 51.97 Hz.
#define JC8012_TIMING_DEFAULT    JC8012PanelTiming{ 60000000, 20, 20, 40, 4, 8, 20 }

// Video frame rates.  Sync and back porches stay the vendor's; the front
// porches stretch the frame so the rate lands on the target, within a few
// ppm -- inside the board crystal's own tolerance, which every on-board
// clock (esp_timer, I2S audio) shares.
//   47.952 Hz = 48000/1001  60 MHz,  910 x 1375  exact
//   48.000 Hz               80 MHz, 1221 x 1365  +1.0 ppm
//   50.000 Hz               60 MHz,  883 x 1359  +2.5 ppm
//   59.940 Hz = 60000/1001  80 MHz, 1002 x 1332  +2.0 ppm
//   60.000 Hz               80 MHz, 1001 x 1332  +1.0 ppm
// See README-engineering.md, "Refresh rate and video timing".
#define JC8012_TIMING_47_952HZ   JC8012PanelTiming{ 60000000, 20, 20,  70, 4, 8, 83 }
#define JC8012_TIMING_48HZ       JC8012PanelTiming{ 80000000, 20, 20, 381, 4, 8, 73 }
#define JC8012_TIMING_50HZ       JC8012PanelTiming{ 60000000, 20, 20,  43, 4, 8, 67 }
#define JC8012_TIMING_59_94HZ    JC8012PanelTiming{ 80000000, 20, 20, 162, 4, 8, 40 }
#define JC8012_TIMING_60HZ       JC8012PanelTiming{ 80000000, 20, 20, 161, 4, 8, 40 }

// Not a video rate: ~40 Hz for more render time.  The vendor's 1312-row
// frame at 48 MHz with a 915-pixel line = 39.985 Hz, so each row lasts
// 19.06 us against the default's 14.67 (+30 % per raster chunk).
#define JC8012_TIMING_40HZ       JC8012PanelTiming{ 48000000, 20, 20,  75, 4, 8, 20 }
// ~30 Hz: 40 MHz with a 1016-pixel line = 30.008 Hz, 25.4 us a row.
#define JC8012_TIMING_30HZ       JC8012PanelTiming{ 40000000, 20, 20, 176, 4, 8, 20 }
#endif

// setRefreshHz() keeps at least this many lines of vertical front porch.
#ifndef JC8012_TRIM_MIN_VFP
#define JC8012_TRIM_MIN_VFP 16
#endif

struct jc8012_rcopy;   // jc8012_raster_copy.h: the raster prefetch's GDMA copier

class chipguy_JC8012P4A1C_display {
public:
    // Takes the board that owns the shared I2C bus the touch controller is on.
    // begin() starts the board (idempotent) before bringing up touch.
    explicit chipguy_JC8012P4A1C_display(JC8012P4A1C_Board &board);

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

    // Panel low-power mode, for sleeping the board (e.g. light sleep until a
    // touch).  sleep() turns the backlight off, waits for any frame the PPA is
    // still writing, sends Display Off + Sleep In, and stops the video stream
    // at a frame boundary — so the chip can then light-sleep safely.  wake()
    // restarts the stream, sends Sleep Out + Display On and
    // restores the backlight to its level before sleep().  Touch keeps working
    // throughout.  Both are no-ops if already in that state.
    void sleep();
    void wake();
    bool isAsleep() const { return _asleep; }

    // ── Low-power sleep ─────────────────────────────────────────────────────
    // Each call: waits for any finger to lift, puts the panel to sleep
    // (sleep()), light-sleeps the chip, and on waking brings the display back
    // (wake(), ~170 ms) — the sketch carries on where it was, RAM and
    // framebuffers intact.  The BOOT button wakes all three.  Measured battery
    // current while asleep (Guition's cell, radio held off with
    // board.setRadioPower(false); floors vary ~6-10 mA between boards):
    //
    //   sleepUntilSwipe()   ~16 mA at 3 s checks (~24 mA at 1 s).  Touch is off
    //                       between checks; every checkMs it is restarted and
    //                       looked at for ~70 ms.  A finger RESTING on the
    //                       screen isn't seen (the controller calibrates it
    //                       away on restart), so the gesture is a swipe lasting
    //                       up to checkMs.
    //   sleepUntilTap()     ~45 mA.  Touch stays armed; any touch wakes at once.
    //   sleepUntilButton()  ~10 mA.  Touch off; only BOOT wakes.
    //
    // If the Wi-Fi chip is running (the default), add ~22 mA to each.
    enum WakeReason : uint8_t { WAKE_SWIPE, WAKE_TAP, WAKE_BUTTON };
    WakeReason sleepUntilSwipe(uint32_t checkMs = 3000);
    WakeReason sleepUntilTap();
    WakeReason sleepUntilButton();

    // Touch driver access (the GSL3680 reports in NATIVE panel coordinates).
    gsl3680_touch& getTouchDriver() { return _touch; }

    // Start the touch controller again, e.g. after its firmware was installed
    // in NVS while this sketch was running.  Updates touchAvailable().
    bool restartTouch() { _touch_ok = _touch.begin(); return _touch_ok; }


    // The touch driver's phantom-jump rejection and 7-sample median: off by
    // default.  Turn on for a panel where a spurious contact must never yank a
    // control (a volume slider), at the cost of ~50 ms lag and very fast
    // swipes being cut short.  See gsl3680_touch.h.  Before or after begin().
    void setTouchFilter(bool on) { _touch.setFilter(on); }
    bool touchFilter() const { return _touch.filter(); }


    // False if the touch controller failed to initialize and begin() came up
    // with touch disabled (display still works; getTouch() reports no touches).
    bool touchAvailable() const { return _touch_ok; }

    // Map a raw NATIVE touch point (as returned by getTouchDriver().getTouch())
    // to LOGICAL coordinates for the active rotation, so a touch lines up with
    // what LVGL drew.  Returns false (and leaves lx/ly untouched) if the point
    // falls outside the logical area.
    bool mapTouch(uint16_t rawX, uint16_t rawY, int32_t &lx, int32_t &ly) const;

    // Debug counters from the DPI panel driver.
    uint32_t getFrameCount();
    uint32_t getUnderrunCount();

    // --- Without LVGL -----------------------------------------------------------
    // begin() normally allocates two full-screen LVGL draw buffers in PSRAM
    // (2 MB each) and a PPA client.  setDrawBuffers(false), BEFORE begin(),
    // skips them, for a sketch that draws into the framebuffers itself -- as
    // chipguy_JC8012P4A1C_terminal does.  flip() then does nothing.
    void setDrawBuffers(bool allocate) { _want_draw_buffers = allocate; }

    // The core whose interrupts the panel uses (its DMA's, which drive the
    // raster band; -1, the default: the core that calls begin()).  Call BEFORE
    // begin().  A core that holds its interrupts off for long -- a large
    // esp_cache_msync, as LVGL's flush and the PPA driver do -- delays them;
    // with a raster band that makes chunks late, so put them on the raster
    // task's core, not LVGL's.
    void setInterruptCore(int core) { _irq_core = core; }

    // How many framebuffers to allocate: 1, 2 (default) or 3.  Call BEFORE
    // begin().  A third lets a PPA-composed picture (the terminal in its PPA
    // modes) start on the next frame while the last one is still waiting to
    // be shown, at the cost of one more framebuffer of PSRAM.  One halves
    // the internal RAM the DMA lists take (setScanBlocks()), but LVGL and
    // the terminal need at least two.
    void setFramebufferCount(uint8_t n) { _num_fbs = n < 1 ? 1 : n > 3 ? 3 : n; }

    // The panel's pixel clock and porches, and so its refresh rate.  Call
    // BEFORE begin().  Use one of the JC8012_TIMING_* presets above, e.g.
    // setPanelTiming(JC8012_TIMING_59_94HZ) to show 29.97/59.94 fps video
    // one frame per refresh.  A faster clock shortens every row, so a raster
    // band has less time per chunk (rasterChunkPeriodUs()).
    void setPanelTiming(const JC8012PanelTiming &t) { _timing = t; }
    const JC8012PanelTiming &panelTiming() const { return _timing; }
    // The pixel clock actually running, in Hz (0 before begin()).
    uint32_t pixelClockHz() const { return _pixel_clock_hz; }

    // Fine-tune the refresh rate while the panel runs, without restarting
    // it: e.g. a video player following a stream whose clock differs a
    // little from the board's, by nudging the rate when its buffer drifts.
    // The pixel clock and line stay as panelTiming() set them; the frame
    // gets longer or shorter in its vertical front porch, by a fraction of a
    // line averaged over frames (each frame is a whole number of lines,
    // dithered: ~0.01 ppm resolution).  Slowing down has a wide range;
    // speeding up stops when the front porch would fall below
    // JC8012_TRIM_MIN_VFP lines (about +1.8 % on the 59.94/60 Hz presets,
    // +0.3 % on the default timing).  Returns false, changing nothing, if
    // hz is out of range or begin() hasn't run.  0 = back to the timing's
    // own rate.  Takes effect at the next frame.
    bool setRefreshHz(double hz);
    double refreshHz() const;   // the rate now set (the timing's own if untrimmed)

    // --- Hardware scrolling (used by chipguy_JC8012P4A1C_terminal) -------------
    // Make each framebuffer `rows` rows tall instead of 1280, so a scrolling
    // ring can keep rows off screen.  Call BEFORE begin(); 0 means 1280.
    void setFramebufferRows(uint16_t rows) { _fb_rows = rows; }
    uint16_t framebufferRows() const { return _fb_rows ? _fb_rows : JC8012_NATIVE_HEIGHT; }

    // How the panel reads the active framebuffer each frame, in native rows: a
    // frozen top band (rows 0..top-1), `window` rows from a ring of `ring` rows
    // starting at row `top` (beginning `offset` rows in, wrapping), then the
    // frozen bottom band from row top+ring.  Takes effect at the next frame
    // boundary, so changing only `offset` scrolls without tearing.  Plain:
    // (0, 1280, 1280, 0).  Returns false if it doesn't fit the framebuffer.
    bool setScanLayout(uint16_t top, uint16_t ring, uint16_t window, uint16_t offset);

    // The most DMA blocks a frame may be read as (default 8).  Call BEFORE
    // begin().  The hardware has no limit; this sizes the DMA lists, at 64
    // bytes of internal RAM per block per framebuffer.  A raster band needs
    // one block per chunk plus one each for the framebuffer above and below.
    void setScanBlocks(uint16_t n) { _scan_blocks = n; }

    // EXPERIMENTAL: read each frame as an arbitrary list of blocks -- `bytes`
    // from `src`, in order, together exactly one 800 x 1280 frame (2,048,000
    // bytes).  A block may end mid-row, so for example each row can come half
    // from one buffer and half from another (2560 blocks).  Every src and
    // bytes must be a multiple of 8, and n at most setScanBlocks().  The array
    // is used in place: keep it valid, and call again after changing it.
    // Sources in PSRAM must be written back from the cache (esp_cache_msync)
    // first.  n = 0 returns to the framebuffer.  Not with a raster band.
    struct ScanBlock { const void *src; uint32_t bytes; };
    bool setScanList(const ScanBlock *blocks, uint32_t n);

    // --- Raster band: drawing in step with the panel's scan -------------------
    // Native panel rows y0..y1-1 are not read from the framebuffer.  Instead fn
    // draws them, chunkRows rows at a time, into internal-RAM bounce buffers
    // (setRasterBuffers(): 2 by default, chunkRows x 1600 bytes each) just
    // before the panel scans them:
    //
    //     void fn(uint16_t* dst, int y, int rows, uint32_t frame, void* ctx)
    //         dst    rows x 800 RGB565 pixels, row y first
    //         y      first panel row to draw; rows: chunkRows
    //         frame  counts panel frames; every chunk of one frame has the same
    //                number, so a change means "first chunk of a new frame"
    //
    // Rows are the panel's NATIVE (portrait) rows whatever the rotation: the
    // PPA rotates only what LVGL draws.  fn runs in a high-priority task pinned
    // to `core`.  With N buffers it is called for chunk c as soon as chunk
    // c - (N - 1) has been read, so it has N - 2 chunks' scan time to finish
    // (rasterChunkPeriodUs(): 14.7 us per row, so ~235 us for 16 rows; with 2
    // buffers, about one chunk's time); a chunk that isn't ready when the panel
    // reaches it counts in rasterLate() and shows whatever the buffer held.  A
    // task that falls a whole queue (24 chunks) behind skips chunks
    // (rasterDropped()), so fn must not rely on seeing every chunk -- for
    // instance, on the last one of a frame to start the next.  Keep fn and its
    // data in internal RAM.  The rest of the screen is the framebuffer as usual
    // (LVGL).
    //
    // y0 and y1 are rounded outward to chunkRows multiples, and the band must
    // be a multiple of N chunks (y1 is extended, or y0 lowered, a chunk at a
    // time if not).  Each chunk is a DMA block, so the band needs chunks + 2 of
    // setScanBlocks().  Needs the plain scan layout.  fn = nullptr turns the
    // band off.  Returns false if it doesn't fit.
    typedef void (*RasterFn)(uint16_t* dst, int y, int rows, uint32_t frame, void* ctx);
    bool setRasterRows(int y0, int y1, RasterFn fn, void* ctx = nullptr, int core = 0,
                       int chunkRows = 16);
    int rasterChunkRows() const { return _rRows; }

    // 2 to 8 bounce buffers instead of 2.  Call before setRasterRows; each is
    // chunkRows x 1600 bytes of internal RAM.  With N buffers fn has N - 2
    // chunk times, and while it draws chunk c the buffer of chunk c + 1 is
    // free too (rasterBuffer(c + 1)), which is what the prefetch below uses.
    void setRasterBuffers(int n) { _rNBuf = n < 2 ? 2 : n > 8 ? 8 : n; }
    uint16_t *rasterBuffer(int chunk) const { return chunk < 0 ? nullptr : _rBuf[chunk % _rNBuf]; }

    // Chunk `chunk` of the band (0 = its first) read straight from the active
    // framebuffer instead of its bounce buffer, or (false) from the buffer
    // again.  Call it from fn for that chunk: fn then need not fill dst.  fn is
    // still called for the chunk every frame.  The setting outlasts the band:
    // clear it when done.
    void setRasterDirect(int chunk, bool direct);

    // How far the panel's DMA has got: chunks read since setRasterRows, all
    // frames end to end (frame * chunks + chunk + 1).  fn can tell from it how
    // much time it has left.
    uint32_t rasterChunksRead() const { return _rRead; }

    // --- Raster prefetch: part of each chunk copied in by DMA --------------------
    // Before fn is called for a chunk, panel columns x0..x1-1 of each of its
    // rows have been copied into its bounce buffer by the AXI GDMA, from
    //
    //     const void* src = srcFn(y, rows, frame, ctx)      row r at src + r * srcStride
    //
    // fn then draws only the rest of each row, and must not write x0..x1-1
    // (it may read them).  srcStride 0 means packed: (x1 - x0) * 2 bytes.
    // srcFn is called once per chunk, in order, when that chunk's copy is
    // started.  With 3 or more buffers that is while fn draws the chunk
    // before, so the copy costs fn no time.  With 2 it is just before fn,
    // in fn's time.  The copy of a frame's chunk 0 starts while the frame
    // before is still being drawn, so a source changed where `frame` changes
    // never tears; once srcFn has been asked for a new frame, nothing reads
    // the old source again.  The source is read by DMA: anything the CPU
    // wrote into it must be written back from the cache first.  x0 and x1 are
    // multiples of 32 (64 bytes, a cache line, so the CPU's part of a row and
    // the DMA's never share one).  Call before setRasterRows; srcFn = nullptr
    // turns it off (with the band off).  Without a free AXI GDMA the CPU
    // copies instead, which is much slower.
    typedef const void* (*RasterSrcFn)(int y, int rows, uint32_t frame, void* ctx);
    bool setRasterPrefetch(int x0, int x1, RasterSrcFn srcFn, void* ctx = nullptr, size_t srcStride = 0);

    // While each prefetch copy is in flight, hold core `core` with a spinning
    // task just under the raster task's priority, so whatever runs there below
    // it -- normally LVGL, in loop() -- is paused, and the copy has the PSRAM
    // to itself.  Tasks blocked in a call (a delay, a semaphore) stay blocked
    // and are not disturbed.  -1 (the default) pauses nothing.  Not the
    // raster task's own core.  Call before setRasterRows; while the band runs
    // it can be turned off (-1) and on again (the same core).
    void setRasterPrefetchPause(int core) { _pfPauseCore = core; }

    // Raster band instrumentation.
    uint32_t rasterChunksDrawn() const { return _rDrawn; }
    uint32_t rasterLate() const { return _rLate; }       // chunks the panel began before they were drawn
    uint32_t rasterDropped() const { return _rDropped; } // chunks never drawn: the task was a queue behind
    uint32_t rasterMerged() const;                       // chunk interrupts that came merged (caught up on)
    uint32_t takeRasterMaxUs();                          // longest chunk (draw + cache write-back), then reset
    uint32_t rasterChunkPeriodUs() const;                // the time each chunk has
    uint32_t framePeriodUs() const;                      // measured panel frame time
    // Raster prefetch instrumentation.
    uint32_t takeRasterCopyMaxUs();                      // longest copy, start to landing, then reset
    uint32_t rasterCopyWaitUs() const;                   // total time fn's task waited for a copy to land
    uint32_t rasterCopyPausedUs() const;                 // total time the pause core was held
    uint32_t rasterCopiesByCpu() const { return _pfCpu; } // copies the CPU did (no GDMA, or a copy lost)

private:
    JC8012P4A1C_Board &_board;   // declared before _touch: _touch uses its bus
    gsl3680_touch _touch;
    esp_lcd_panel_handle_t _panel_handle = nullptr;
    esp_lcd_panel_io_handle_t _io_handle = nullptr;
    int  _backlight_pct = 0;     // last level set, restored by wake()
    bool _asleep = false;
    WakeReason lightSleep(uint8_t mode, uint32_t checkMs);
    ppa_client_handle_t _ppa_client = nullptr;
    uint16_t *_draw_buffers[2] = {nullptr, nullptr};
    uint8_t  _rotation = 0;
    uint16_t _draw_w = JC8012_NATIVE_WIDTH;
    uint16_t _draw_h = JC8012_NATIVE_HEIGHT;
    bool     _touch_ok = false;
    bool     _want_draw_buffers = true;
    uint8_t  _num_fbs = 2;
    uint16_t _fb_rows = 0;
    uint16_t _scan_blocks = 0;
    int      _irq_core = -1;
    JC8012PanelTiming _timing = JC8012_TIMING_DEFAULT;
    uint32_t _pixel_clock_hz = 0;

    // Raster band
    static bool _rasterDoneISR(uint32_t chunk, void *ctx);
    static void _rasterTaskEntry(void *arg);
    void _rasterDraw(uint32_t chunk, uint32_t frame);
    RasterFn _rFn = nullptr;
    void *_rCtx = nullptr;
    uint16_t *_rBuf[8] = {};
    int _rNBuf = 2;             // bounce buffers, 2 to 8
    int _rY0 = 0;
    int _rRows = 16;            // chunk height
    int _rBufRows = 0;          // what _rBuf was allocated for
    uint32_t _rChunks = 0;
    QueueHandle_t _rQueue = nullptr;
    TaskHandle_t _rTask = nullptr;
    volatile uint32_t _rFrame = 0;       // frame number of the chunks being scanned now
    volatile uint32_t _rRead = 0;        // chunks read so far, frames end to end
    volatile uint32_t _rDoneLin = 0;     // 1 + the last chunk drawn, counted as _rRead counts
    volatile uint32_t _rIssued = 0, _rCompleted = 0;
    volatile uint32_t _rDrawn = 0, _rLate = 0, _rDropped = 0;
    volatile uint32_t _rMaxCycles = 0;
    volatile uint32_t _rLastIsrCycles = 0, _rPeriodCycles = 0;

    // Raster prefetch
    void _pfStart(uint32_t chunk, uint32_t frame);
    void _pfLand();
    static void _pfGateEntry(void *arg);
    RasterSrcFn _pfFn = nullptr;
    void *_pfCtx = nullptr;
    int _pfX0 = 0, _pfX1 = 0;
    size_t _pfStride = 0;
    volatile int _pfPauseCore = -1;
    struct jc8012_rcopy *_pfCopier = nullptr;
    TaskHandle_t _pfGate = nullptr;
    int32_t _pfChunk = -1;               // the chunk whose copy was started last (-1: none)
    uint32_t _pfFrame = 0;
    bool _pfByDma = false;               // that copy is the DMA's and not yet landed
    const uint8_t *_pfSrc = nullptr;     // where it came from
    volatile uint32_t _pfCpu = 0;
    volatile uint32_t _pfWaitUs = 0, _pfPausedUs = 0;
    uint32_t _mhz = 360;                 // CPU clock, for the timings

    bool initHardware();
    bool allocateDrawBuffers();

    // Panel reset line (LCD_RST).  The panel is held in reset across the whole
    // host-side bring-up rather than pulsed afterwards; see the PANEL_RESET_*
    // timing constants and their rationale in the .cpp.
    void assertPanelReset();
    void releasePanelFromReset();

    // Failure path for any bring-up step: backlight off, long reset assertion,
    // then reboot for a clean retry (bounded, so a dead panel can't reboot-loop
    // forever).  Always returns false, for `return panelInitFailed("...", ret)`.
    bool panelInitFailed(const char *step, esp_err_t err);

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
