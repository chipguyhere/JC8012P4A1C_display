/*******************************************************************************
 * JC8012P4A1C -- Bouncing balls at the panel's refresh rate, in landscape
 *
 * RasterBouncingBalls turned on its side, and what that takes.
 *
 * In portrait the panel scans the picture top to bottom, so the balls could
 * have the rows above an LVGL strip, and the strip could stay in the
 * framebuffer.  In landscape (the library's rotation 3, USB on the left) the
 * panel scans the picture's COLUMNS, right to left, each one top to bottom.
 * Every 16-row chunk the panel reads is a 16-pixel-wide slice of the whole
 * height of the picture, so every chunk holds both balls and a piece of the
 * LVGL strip along the bottom.  So the whole panel is the raster band, and each
 * chunk is put together in its bounce buffer from two sources:
 *
 *   - The strip: LVGL draws it (1280 x 160) into PSRAM, and the PPA turns it
 *     into a strip framebuffer laid out the way the panel reads it (see
 *     lv_setup.hpp).  The display's RASTER PREFETCH copies the chunk's piece
 *     of it (16 x 160 pixels, 5 KB) into the last 160 columns of the bounce
 *     buffer by the AXI GDMA, a chunk ahead, while the chunk before is drawn.
 *   - The balls: drawn by the CPU straight into the first 640 columns
 *     (balls.cpp), half on each core.
 *
 * With eight bounce buffers the copy for one chunk runs while the balls are
 * drawn into the chunk before, and each chunk is drawn six chunk times
 * (~1.4 ms) ahead of the panel.  While a copy is in flight the display holds LVGL's core (the
 * one loop() runs on), so the copy doesn't share the PSRAM with LVGL's
 * rendering.  Type 'p' in the Serial Monitor to turn that pause off and on
 * and compare, and 'f' to give LVGL a heavy load to compare it under.
 *
 * Requires the following libraries:
 *   - chipguy_JC8012P4A1C_display  (this library)
 *   - chipguy_JC8012P4A1C_board    (shared I2C bus + pin map)
 *   - lvgl 9
 *
 * Arduino IDE Board Settings:
 *   - Board: ESP32P4 Dev Module
 *   - PSRAM: Enabled
 *   - Flash Size: 16 MegaBytes (128 megabits)
 *   - Partition Scheme: Custom (uses the partitions.csv in this sketch folder)
 *   - USB CDC On Boot: Enabled
 *     Serial0 prints a status line every second; 'b' runs a benchmark,
 *     'p' turns the LVGL pause during copies off and on, and 'f' makes LVGL
 *     redraw the whole strip every refresh (a heavy load on the PSRAM).
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 ******************************************************************************/

#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"
#include "balls.h"

// Panel rows per chunk (picture columns per slice).  Each bounce buffer holds
// one chunk, 1600 bytes a row; the panel scans a row every 14.7 us.
#define CHUNK_ROWS 16
// Bounce buffers, 25.6 KB of internal RAM each.  With N, each chunk is drawn
// N - 2 chunk times ahead of the panel (8: ~1.4 ms, room to ride out a stall),
// and the next chunk's buffer is free for its copy (3 is the least for copying
// ahead).  The whole panel is 80 chunks, so N must divide 80: 2, 4, 5 or 8.
#define BOUNCE_BUFFERS 8
#define STR_(x) #x
#define STR(x) STR_(x)

// The LVGL strip's height along the bottom of the landscape screen: the last
// STRIP_ROWS columns of every panel row.  A multiple of 32 (the prefetch
// copies whole 64-byte cache lines).
#define STRIP_ROWS 160

// The raster task runs on this core; the helper that draws the other half of
// each chunk (and the physics) runs on the other one, beside loop() and LVGL.
#define RASTER_CORE 0

// Hold LVGL's core while each copy of the strip is in flight (see above).
#define PAUSE_LVGL_DURING_COPY true

// LVGL's refresh period.  LVGL is the controls, not the animation (that is
// the balls' job, in the bounce buffers), so it runs on casual deadlines:
// at most one strip flip per LVGL_REFRESH_MS, whenever loop() gets to it.
#define LVGL_REFRESH_MS 50

#define START_BALLS 100
#define START_SEE_THROUGH true
// Speed slider: 1..100, where 100 is the LVGL BouncingBalls demo's speed.
#define START_SPEED 20
static void speed_text(lv_obj_t *label, int v) {
    lv_label_set_text_fmt(label, "Speed: %d.%02dx", v * 5 / 100, v * 5 % 100);
}

static lv_obj_t *g_countLabel, *g_speedLabel, *g_perf;
static uint32_t g_hz10 = 0;   // the panel's refresh rate x10, measured at boot
static int g_lvglCore = 1;
static bool g_pause = PAUSE_LVGL_DURING_COPY;
static bool g_stress = false;   // Serial 'f': LVGL redraws the whole strip every refresh
extern volatile uint32_t g_physWaitUs, g_helpStolen;

static void count_cb(lv_event_t *e) {
    int v = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));
    balls_set_count(v);
    lv_label_set_text_fmt(g_countLabel, "Balls: %d", v);
}

static void speed_cb(lv_event_t *e) {
    int v = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));   // 1..100: 0.05x .. 5.00x
    balls_set_speed(v);
    speed_text(g_speedLabel, v);
}

static void see_through_cb(lv_event_t *e) {
    balls_set_see_through(lv_obj_has_state((lv_obj_t *)lv_event_get_target(e), LV_STATE_CHECKED));
}

// Once a second: the ball frames drawn, the longest chunk against the time it
// has, the strip copies, and how often LVGL got a frame to the raster.
static void perf_update(lv_timer_t *t) {
    LV_UNUSED(t);
    static uint32_t lastMs = 0, lastFrames = 0, lastLate = 0, lastFlips = 0;
    static uint32_t lastPaused = 0, lastWait = 0, lastPpa = 0, lastFlipWait = 0;
    uint32_t now = millis(), f = balls_frames(), late = display.rasterLate();
    uint32_t ms = now - lastMs;
    if (!ms) return;
    uint32_t fps10 = (f - lastFrames) * 10000 / ms;
    uint32_t chunkUs = display.takeRasterMaxUs();
    uint32_t copyUs = display.takeRasterCopyMaxUs();
    uint32_t flips = lv_setup.flips();
    uint32_t paused = display.rasterCopyPausedUs(), wait = display.rasterCopyWaitUs(), ppa = lv_setup.ppaUs();
    uint32_t flipWait = lv_setup.flipWaitUs();
    const uint32_t hz10 = g_hz10;
    uint32_t shown10 = (fps10 + 5 >= hz10 && fps10 <= hz10 + 5) ? hz10 : fps10;
    uint32_t periodUs = display.rasterChunkPeriodUs();
    uint32_t pausePct10 = (paused - lastPaused) / ms;          // per mille of the time
    lv_label_set_text_fmt(g_perf,
        "%u.%u FPS (panel %u.%u Hz) | LVGL %u flips/s\n"
        "chunk %u of %u us | strip copy %u us | late %u\n"
        "LVGL paused %u.%u%% for copies%s",
        (unsigned)(shown10 / 10), (unsigned)(shown10 % 10), (unsigned)(hz10 / 10), (unsigned)(hz10 % 10),
        (unsigned)((flips - lastFlips) * 1000 / ms),
        (unsigned)chunkUs, (unsigned)(periodUs * (BOUNCE_BUFFERS - 2)), (unsigned)copyUs, (unsigned)(late - lastLate),
        (unsigned)(pausePct10 / 10), (unsigned)(pausePct10 % 10), g_pause ? "" : " (pause off)");
    Serial0.printf("fps %u.%u  chunk %u of %u us  late %u (total %u)  dropped %u  merged %u  underruns %u  "
                   "physics wait %u us  helper's jobs done here %u | copy max %u us, waited %u us, paused %u us, by CPU %u | "
                   "LVGL flips %u, PPA %u us, flip wait %u us%s\n",
                   (unsigned)(fps10 / 10), (unsigned)(fps10 % 10), (unsigned)chunkUs,
                   (unsigned)(periodUs * (BOUNCE_BUFFERS - 2)), (unsigned)(late - lastLate), (unsigned)late,
                   (unsigned)display.rasterDropped(), (unsigned)display.rasterMerged(),
                   (unsigned)display.getUnderrunCount(), (unsigned)g_physWaitUs, (unsigned)g_helpStolen,
                   (unsigned)copyUs, (unsigned)(wait - lastWait), (unsigned)(paused - lastPaused),
                   (unsigned)display.rasterCopiesByCpu(),
                   (unsigned)(flips - lastFlips), (unsigned)(ppa - lastPpa), (unsigned)(flipWait - lastFlipWait),
                   g_stress ? " [stress]" : "");
    lastMs = now;
    lastFrames = f;
    lastLate = late;
    lastFlips = flips;
    lastPaused = paused;
    lastWait = wait;
    lastPpa = ppa;
    lastFlipWait = flipWait;
}

static lv_obj_t *make_label(lv_obj_t *parent, const char *text) {
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_color(l, lv_color_white(), 0);
    lv_label_set_text(l, text);
    return l;
}

static lv_obj_t *make_slider(lv_obj_t *parent, int x, int y, int w, int lo, int hi, int v,
                             lv_event_cb_t cb) {
    lv_obj_t *s = lv_slider_create(parent);
    lv_obj_set_size(s, w, 16);
    lv_obj_set_pos(s, x, y);
    lv_slider_set_range(s, lo, hi);
    lv_slider_set_value(s, v, LV_ANIM_OFF);
    lv_obj_add_event_cb(s, cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_set_ext_click_area(s, 12);
    return s;
}

// The strip: 1280 x STRIP_ROWS, its own LVGL display.  Sliders and the switch
// on the left, the readout on the right.
static void build_strip() {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x181820), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    const int m = 16, labelW = 120, sliderX = m + labelW, sliderW = 600;
    int y = 18;

    g_countLabel = make_label(scr, "");
    lv_label_set_text_fmt(g_countLabel, "Balls: %d", START_BALLS);
    lv_obj_set_pos(g_countLabel, m, y);
    make_slider(scr, sliderX, y + 1, sliderW, 1, BALLS_MAX, START_BALLS, count_cb);
    balls_set_count(START_BALLS);

    // See-through switch, right of the sliders.
    lv_obj_t *sw = lv_switch_create(scr);
    lv_obj_set_size(sw, 50, 26);
    lv_obj_set_pos(sw, sliderX + sliderW + 40, y + 14);
    if (START_SEE_THROUGH) lv_obj_add_state(sw, LV_STATE_CHECKED);
    balls_set_see_through(START_SEE_THROUGH);
    lv_obj_add_event_cb(sw, see_through_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *swLabel = make_label(scr, "See-through");
    lv_obj_align_to(swLabel, sw, LV_ALIGN_OUT_BOTTOM_MID, 0, 6);
    y += 44;

    g_speedLabel = make_label(scr, "");
    speed_text(g_speedLabel, START_SPEED);
    lv_obj_set_pos(g_speedLabel, m, y);
    make_slider(scr, sliderX, y + 1, sliderW, 1, 100, START_SPEED, speed_cb);
    balls_set_speed(START_SPEED);

    // Bottom left: how this demo is put together.
    lv_obj_t *note = lv_label_create(scr);
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(note, sliderX + sliderW + 80);
    lv_obj_set_style_text_color(note, lv_color_hex(0xA0A0B0), 0);
    lv_label_set_text(note,
        "The panel scans this screen's columns, so every " STR(CHUNK_ROWS) "-column slice is put together "
        "as it is scanned: the balls drawn by the CPU, this strip copied in by DMA from LVGL's buffer.");
    lv_obj_align(note, LV_ALIGN_BOTTOM_LEFT, m, -10);

    // Right: frame rate, chunk and copy timing.
    g_perf = make_label(scr, "");
    lv_obj_set_style_text_align(g_perf, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(g_perf, LV_ALIGN_RIGHT_MID, -m, 0);
    lv_timer_create(perf_update, 1000, NULL);
}

void setup() {
    Serial0.begin(115200);
    g_lvglCore = xPortGetCoreID();   // loop() runs here too: LVGL's core

    // The whole panel is the raster band: one DMA block per chunk, more than
    // the default 8.  Both must be set before the display starts.
    display.setScanBlocks(JC8012_NATIVE_HEIGHT / CHUNK_ROWS + 2);
    display.setRasterBuffers(BOUNCE_BUFFERS);

    // Landscape (rotation 3), LVGL on the bottom strip; the panel's interrupts
    // on the raster core.
    lv_setup.begin(STRIP_ROWS, RASTER_CORE, LVGL_REFRESH_MS);

    // The real frame period: wait for a few frames.
    uint32_t f0 = display.getFrameCount();
    while (display.getFrameCount() - f0 < 3) delay(5);
    uint32_t periodUs = display.framePeriodUs();
    Serial0.printf("Panel frame period %u us (%.2f Hz)\n", (unsigned)periodUs, 1e6f / periodUs);
    g_hz10 = periodUs ? (10000000 + periodUs / 2) / periodUs : 0;

    balls_init(display.width(), lv_setup.screenRow(), periodUs);
    balls_start_helper(RASTER_CORE ^ 1);
    build_strip();

    // Each chunk's strip piece is copied into the last STRIP_ROWS columns of
    // its rows, from the strip framebuffer lv_setup says.
    if (!display.setRasterPrefetch(lv_setup.panelColumn(), JC8012_NATIVE_WIDTH,
                                   lv_strip_setup_class::rasterSrc, &lv_setup))
        Serial0.println("setRasterPrefetch refused");
    if (g_pause) display.setRasterPrefetchPause(g_lvglCore);
    if (g_lvglCore == RASTER_CORE) Serial0.println("loop() is on the raster core: no LVGL pause");
    if (!display.setRasterRows(0, JC8012_NATIVE_HEIGHT, balls_raster, nullptr, RASTER_CORE, CHUNK_ROWS))
        Serial0.println("setRasterRows refused");
    Serial0.printf("Internal RAM free: %u bytes, largest block %u\n",
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

// Serial 'b': step through ball counts, opaque then see-through, two seconds
// each, and print the longest chunk and the late chunks for each.  (The
// sliders and the switch keep their own settings; touch one to restore it.)
static void bench() {
    static const int counts[] = { 50, 100, 200, 300, 400 };
    for (int st = 0; st < 2; st++) {
        balls_set_see_through(st);
        for (int c : counts) {
            balls_set_count(c);
            delay(300);
            display.takeRasterMaxUs();
            display.takeRasterCopyMaxUs();
            g_physWaitUs = 0;
            uint32_t late0 = display.rasterLate();
            uint32_t t0 = millis();
            while (millis() - t0 < 2000) { lv_timer_handler(); delay(5); }
            Serial0.printf("bench %-11s %3d balls: chunk drawn in %u us max (has %u x %d), late %u, "
                           "copy %u us max, physics wait %u us\n",
                           st ? "see-through" : "opaque", c, (unsigned)display.takeRasterMaxUs(),
                           (unsigned)display.rasterChunkPeriodUs(), BOUNCE_BUFFERS - 2,
                           (unsigned)(display.rasterLate() - late0), (unsigned)display.takeRasterCopyMaxUs(),
                           (unsigned)g_physWaitUs);
        }
    }
    balls_set_see_through(START_SEE_THROUGH);
    balls_set_count(START_BALLS);
}

void loop() {
    if (Serial0.available()) {
        int c = Serial0.read();
        if (c == 'b') bench();
        if (c == 'f') {
            g_stress = !g_stress;
            Serial0.printf("LVGL full-strip redraw every refresh: %s\n", g_stress ? "on" : "off");
        }
        if (c == 'p') {
            g_pause = !g_pause;
            display.setRasterPrefetchPause(g_pause ? g_lvglCore : -1);
            Serial0.printf("LVGL pause during copies: %s\n", g_pause ? "on" : "off");
        }
    }
    if (g_stress) lv_obj_invalidate(lv_screen_active());
    lv_timer_handler();
    delay(g_stress ? 1 : 5);
}
