/*******************************************************************************
 * JC8012P4A1C -- Bouncing balls at the panel's refresh rate
 *
 * A proof of concept: most of the screen bypasses LVGL entirely.  The
 * display driver hands balls_raster() (balls.cpp) each 16-row chunk of the
 * ball area just before the panel scans it, and the balls are drawn straight
 * into one of two internal-RAM bounce buffers -- no framebuffer, no PSRAM.
 * The two buffers take 51 KB of internal RAM in all (CHUNK_ROWS below).
 * Only the control strip at the bottom is LVGL: two sliders (ball count up to
 * 200, and speed), a see-through (75 % opaque) switch and the frame-rate
 * readout.
 *
 * Because the balls are drawn while the panel scans, every panel refresh is a
 * frame of the animation, and the physics advances once per refresh from one
 * snapshot, so nothing tears.
 *
 * Ported from the same example in chipguy_P486Panel_display, and that from
 * the chipguy_Waveshare-ESP32-S3-Touch-LCD-7-7B_display library.
 *
 * Portrait (rotation 0) only: the ball area is drawn in the panel's own
 * orientation, and the LVGL strip has to sit below it.
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
 *     Serial0 prints a status line every second, and 'b' typed into it runs
 *     a benchmark.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 ******************************************************************************/

#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"
#include "balls.h"

// Rows per chunk.  Each of the two bounce buffers holds one chunk (1600 bytes
// a row), and each chunk must be drawn in the time the panel takes to scan
// one (14.7 us a row).  Smaller saves internal RAM and leaves less time.
#define CHUNK_ROWS 16
#define STR_(x) #x
#define STR(x) STR_(x)

// The LVGL control strip's height at the bottom; the balls get the rows above
// it.  The ball area must be an even number of chunks: 1120 rows here, 70
// chunks, each its own DMA block, plus one block for the strip.
#define STRIP_ROWS 160
static_assert(STRIP_ROWS % CHUNK_ROWS == 0, "STRIP_ROWS must be a multiple of CHUNK_ROWS");
static_assert(((JC8012_NATIVE_HEIGHT - STRIP_ROWS) / CHUNK_ROWS) % 2 == 0,
              "the ball area must be an even number of chunks");
static int g_ballRows = 0;   // display.height() - STRIP_ROWS, set in setup()

// The raster task runs on this core; the helper that draws the other half of
// each chunk (and the physics) runs on the other one.
#define RASTER_CORE 0

#define START_BALLS 100
#define START_SEE_THROUGH true
// Speed slider: 1..100, where 100 is the LVGL BouncingBalls demo's speed.
// It is labeled so that 20 (a fifth of that, slow enough to follow 100+ balls)
// reads 1.0x and the top reads 5.0x.
#define START_SPEED 20
static void speed_text(lv_obj_t *label, int v) {
    lv_label_set_text_fmt(label, "Speed: %d.%02dx", v * 5 / 100, v * 5 % 100);
}

static lv_obj_t *g_countLabel, *g_speedLabel, *g_perf;
static uint32_t g_hz10 = 0;   // the panel's refresh rate x10, measured at boot
extern volatile uint32_t g_physWaitUs;

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

// Once a second: frames the ball area drew (one per panel refresh unless one
// was late), and the longest time one chunk took (drawing plus the cache
// write-back) against the time it has (one chunk's scan).
static void perf_update(lv_timer_t *t) {
    LV_UNUSED(t);
    static uint32_t lastMs = 0, lastFrames = 0, lastLate = 0;
    uint32_t now = millis(), f = balls_frames(), late = display.rasterLate();
    uint32_t fps10 = (now != lastMs) ? (f - lastFrames) * 10000 / (now - lastMs) : 0;
    uint32_t chunkUs = display.takeRasterMaxUs();
    const uint32_t hz10 = g_hz10;
    // Counting whole frames over about a second rounds to +/-0.1: within
    // 0.5 of the panel's rate, every frame was drawn, so show that rate.
    uint32_t shown10 = (fps10 + 5 >= hz10 && fps10 <= hz10 + 5) ? hz10 : fps10;
    uint32_t periodUs = display.rasterChunkPeriodUs();
    lv_label_set_text_fmt(g_perf,
        "%u.%u FPS (panel %u.%u Hz)\n"
        "chunk drawn in %u of %u us | late %u",
        (unsigned)(shown10 / 10), (unsigned)(shown10 % 10), (unsigned)(hz10 / 10), (unsigned)(hz10 % 10),
        (unsigned)chunkUs, (unsigned)periodUs, (unsigned)(late - lastLate));
    Serial0.printf("fps %u.%u  chunk %u/%u us  late %u (total %u)  dropped %u  underruns %u  physics wait %u us\n",
                   (unsigned)(fps10 / 10), (unsigned)(fps10 % 10), (unsigned)chunkUs,
                   (unsigned)periodUs, (unsigned)(late - lastLate), (unsigned)late,
                   (unsigned)display.rasterDropped(), (unsigned)display.getUnderrunCount(),
                   (unsigned)g_physWaitUs);
    lastMs = now;
    lastFrames = f;
    lastLate = late;
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

static void build_strip() {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x181820), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    const int W = display.width();
    const int m = 16, labelW = 120, swColW = 120;
    const int sliderX = m + labelW, sliderW = W - sliderX - swColW - m;
    int y = g_ballRows + 14;

    g_countLabel = make_label(scr, "");
    lv_label_set_text_fmt(g_countLabel, "Balls: %d", START_BALLS);
    lv_obj_set_pos(g_countLabel, m, y);
    make_slider(scr, sliderX, y + 1, sliderW, 1, BALLS_MAX, START_BALLS, count_cb);
    balls_set_count(START_BALLS);

    // See-through switch, in a column right of the sliders.
    lv_obj_t *sw = lv_switch_create(scr);
    lv_obj_set_size(sw, 50, 26);
    lv_obj_set_pos(sw, W - m - swColW / 2 - 25, y - 4);
    if (START_SEE_THROUGH) lv_obj_add_state(sw, LV_STATE_CHECKED);
    balls_set_see_through(START_SEE_THROUGH);
    lv_obj_add_event_cb(sw, see_through_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *swLabel = make_label(scr, "See-through");
    lv_obj_align_to(swLabel, sw, LV_ALIGN_OUT_BOTTOM_MID, 0, 6);
    y += 40;

    g_speedLabel = make_label(scr, "");
    speed_text(g_speedLabel, START_SPEED);
    lv_obj_set_pos(g_speedLabel, m, y);
    make_slider(scr, sliderX, y + 1, sliderW, 1, 100, START_SPEED, speed_cb);
    balls_set_speed(START_SPEED);

    // Bottom left: how this demo gets its frame rate.
    lv_obj_t *note = lv_label_create(scr);
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(note, W / 2);
    lv_obj_set_style_text_color(note, lv_color_hex(0xA0A0B0), 0);
    lv_label_set_text(note,
        "The balls bypass LVGL: they are drawn straight into the display's "
        "scan-out buffers, " STR(CHUNK_ROWS) " rows at a time. Only this strip is LVGL.");
    lv_obj_align(note, LV_ALIGN_BOTTOM_LEFT, m, -8);

    // Bottom right: frame rate and chunk timing.
    g_perf = make_label(scr, "");
    lv_obj_set_style_text_align(g_perf, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(g_perf, LV_ALIGN_BOTTOM_RIGHT, -m, -8);
    lv_timer_create(perf_update, 1000, NULL);
}

void setup() {
    Serial0.begin(115200);

    // The ball area is one DMA block per chunk and the strip one more: more
    // than the default 8.  Must be set before the display starts.
    display.setScanBlocks(JC8012_NATIVE_HEIGHT / CHUNK_ROWS + 2);

    lv_setup.begin(0);   // portrait: the strip at the bottom of the ball area

    // The real frame period: wait for a few frames.
    uint32_t f0 = display.getFrameCount();
    while (display.getFrameCount() - f0 < 3) delay(5);
    uint32_t periodUs = display.framePeriodUs();
    Serial0.printf("Panel frame period %u us (%.2f Hz)\n", (unsigned)periodUs, 1e6f / periodUs);
    g_hz10 = periodUs ? (10000000 + periodUs / 2) / periodUs : 0;

    g_ballRows = display.height() - STRIP_ROWS;
    balls_init(display.width(), g_ballRows, periodUs);
    balls_start_helper(RASTER_CORE ^ 1);   // half of each chunk on the other core
    build_strip();
    if (!display.setRasterRows(0, g_ballRows, balls_raster, nullptr, RASTER_CORE, CHUNK_ROWS))
        Serial0.println("setRasterRows refused");
    Serial0.printf("Internal RAM free: %u bytes\n",
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

// Serial 'b': step through ball counts, opaque then see-through, two seconds
// each, and print the longest chunk and the late chunks for each.  (The
// sliders and the switch keep their own settings; touch one to restore it.)
static void bench() {
    static const int counts[] = { 20, 50, 100, 150, 200 };
    for (int st = 0; st < 2; st++) {
        balls_set_see_through(st);
        for (int c : counts) {
            balls_set_count(c);
            delay(300);
            display.takeRasterMaxUs();
            g_physWaitUs = 0;
            uint32_t late0 = display.rasterLate();
            uint32_t t0 = millis();
            while (millis() - t0 < 2000) { lv_timer_handler(); delay(5); }
            Serial0.printf("bench %-11s %3d balls: chunk drawn in %u us max (has %u), late %u, physics wait %u us\n",
                           st ? "see-through" : "opaque", c, (unsigned)display.takeRasterMaxUs(),
                           (unsigned)display.rasterChunkPeriodUs(),
                           (unsigned)(display.rasterLate() - late0), (unsigned)g_physWaitUs);
        }
    }
    balls_set_see_through(START_SEE_THROUGH);
    balls_set_count(START_BALLS);
}

void loop() {
    if (Serial0.available() && Serial0.read() == 'b') bench();
    lv_timer_handler();
    delay(5);
}
