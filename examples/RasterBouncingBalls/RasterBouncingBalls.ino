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
 * readout.  "Advanced" swaps the speed slider for the panel's refresh-rate
 * controls: a rate selector (30 to 60 Hz, including the video rates), and a
 * Trim slider and Wobble checkbox that move the rate while the panel runs
 * (display.setRefreshHz()), as a video player following a stream would.
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
 *     a benchmark ('t' measures setRefreshHz(), 'w' toggles Wobble, 'n' restarts at the next rate).
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

// Refresh-rate controls, behind the "Advanced" button: a rate selector, a
// Trim slider and a Wobble checkbox.  The pixel clock can only be set before
// the display starts, so picking a rate saves it in RTC memory (kept across
// ESP.restart(), not a power cycle) and restarts.  Trim and Wobble use
// display.setRefreshHz(), which changes the rate while the panel runs: Trim
// by hand, Wobble slowly and sinusoidally, the way a player following a
// stream's clock would.  Set to 0 for the plain demo at the default 52 Hz;
// the commented-out setPanelTiming() lines in setup() show how a sketch
// picks one rate for good.
#define RATE_CONTROLS 1
#if RATE_CONTROLS
struct RateChoice { const char *name; JC8012PanelTiming timing; };
static const RateChoice kRates[] = {
    { "30",     JC8012_TIMING_30HZ },
    { "40",     JC8012_TIMING_40HZ },
    { "47.952", JC8012_TIMING_47_952HZ },
    { "48",     JC8012_TIMING_48HZ },
    { "50",     JC8012_TIMING_50HZ },
    { "52",     JC8012_TIMING_DEFAULT },
    { "59.94",  JC8012_TIMING_59_94HZ },
    { "60",     JC8012_TIMING_60HZ },
};
#define RATE_COUNT (sizeof(kRates) / sizeof(kRates[0]))
#define RATE_DEFAULT_IDX 5          // after a power-on: the default timing
#define RATE_MAGIC 0x52A7E5u
#define TRIM_MIN (-100)             // trim slider, in 0.01 % steps: -1.00 %
#define WOBBLE_AMPL 25              // +/-0.25 % around the slider ...
#define WOBBLE_PERIOD_MS 20000      // ... over 20 s
RTC_NOINIT_ATTR static uint32_t s_rateMagic, s_rateIdx, s_advanced;
static uint32_t g_rateIdx = RATE_DEFAULT_IDX;
static bool g_advanced = false, g_wobble = false;
static double g_baseHz = 0;         // the timing's own rate
static int g_trimMax = 100;         // speeding up is limited by the front porch
static int g_wobbleCenter = 0;
static lv_obj_t *g_trimLabel, *g_trimSlider, *g_wobbleBox;
static lv_obj_t *g_basic[3], *g_adv[4];   // shown in the basic / advanced view
#endif

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
    const uint32_t hz1e4 = (uint32_t)(display.refreshHz() * 10000 + 0.5);   // the rate set
    // Counting whole frames over about a second rounds to +/-0.1: within
    // 0.5 of the panel's rate, every frame was drawn, so show that rate.
    uint32_t shown10 = (fps10 + 5 >= hz10 && fps10 <= hz10 + 5) ? hz10 : fps10;
    uint32_t periodUs = display.rasterChunkPeriodUs();
    lv_label_set_text_fmt(g_perf,
        "%u.%u FPS (panel %u.%04u Hz)\n"
        "chunk drawn in %u of %u us | late %u",
        (unsigned)(shown10 / 10), (unsigned)(shown10 % 10), (unsigned)(hz1e4 / 10000), (unsigned)(hz1e4 % 10000),
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

#if RATE_CONTROLS
static void rate_select(uint32_t idx) {
    s_rateIdx = idx % RATE_COUNT;
    s_advanced = g_advanced;
    s_rateMagic = RATE_MAGIC;
    Serial0.printf("Restarting at %s Hz\n", kRates[s_rateIdx].name);
    Serial0.flush();
    display.setBacklight(0);   // hide the stream stopping mid-frame
    ESP.restart();
}

static void rate_matrix_cb(lv_event_t *e) {
    uint32_t id = lv_buttonmatrix_get_selected_button((lv_obj_t *)lv_event_get_target(e));
    if (id != LV_BUTTONMATRIX_BUTTON_NONE && id != g_rateIdx) rate_select(id);
}

// pct100: the trim in 0.01 % steps, fractions allowed (the wobble's).
static void trim_apply(double pct100) {
    display.setRefreshHz(g_baseHz * (1 + pct100 * 1e-4));
    int v = (int)lroundf(pct100);
    lv_label_set_text_fmt(g_trimLabel, "Trim: %c%d.%02d%%", v < 0 ? '-' : '+', abs(v) / 100, abs(v) % 100);
}

static void trim_cb(lv_event_t *e) {
    int v = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));
    g_wobbleCenter = v;
    if (!g_wobble) trim_apply(v);   // the wobble timer picks the new centre up
}

static void wobble_set(bool on) {
    g_wobble = on;
    if (on) lv_obj_add_state(g_wobbleBox, LV_STATE_CHECKED);
    else    lv_obj_remove_state(g_wobbleBox, LV_STATE_CHECKED);
    g_wobbleCenter = lv_slider_get_value(g_trimSlider);
    if (!on) {
        lv_slider_set_value(g_trimSlider, g_wobbleCenter, LV_ANIM_OFF);
        trim_apply(g_wobbleCenter);
    }
}

static void wobble_cb(lv_event_t *e) {
    wobble_set(lv_obj_has_state((lv_obj_t *)lv_event_get_target(e), LV_STATE_CHECKED));
}

// Every 50 ms: move the rate (and the slider's knob, unless a finger is on
// it) along the sine.
static void wobble_tick(lv_timer_t *t) {
    LV_UNUSED(t);
    if (!g_wobble) return;
    double ph = (millis() % WOBBLE_PERIOD_MS) * (2 * M_PI / WOBBLE_PERIOD_MS);
    double v = g_wobbleCenter + WOBBLE_AMPL * sin(ph);
    if (v < TRIM_MIN) v = TRIM_MIN;
    if (v > g_trimMax) v = g_trimMax;
    trim_apply(v);
    if (!lv_obj_has_state(g_trimSlider, LV_STATE_PRESSED))
        lv_slider_set_value(g_trimSlider, (int)lroundf(v), LV_ANIM_OFF);
}

static void show_advanced(bool on) {
    g_advanced = on;
    for (lv_obj_t *o : g_basic) { if (on) lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN); else lv_obj_remove_flag(o, LV_OBJ_FLAG_HIDDEN); }
    for (lv_obj_t *o : g_adv)   { if (on) lv_obj_remove_flag(o, LV_OBJ_FLAG_HIDDEN); else lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN); }
    if (!on) {   // the basic view runs at the chosen rate, untrimmed
        wobble_set(false);
        lv_slider_set_value(g_trimSlider, 0, LV_ANIM_OFF);
        g_wobbleCenter = 0;
        trim_apply(0);
    }
}

static void advanced_cb(lv_event_t *e) {
    show_advanced(lv_obj_has_state((lv_obj_t *)lv_event_get_target(e), LV_STATE_CHECKED));
}
#endif

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
    lv_obj_t *speedSlider = make_slider(scr, sliderX, y + 1, sliderW, 1, 100, START_SPEED, speed_cb);
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

#if RATE_CONTROLS
    // Under the see-through switch: Advanced swaps the Speed row and the
    // note above for the refresh-rate controls.
    lv_obj_t *ab = lv_button_create(scr);
    lv_obj_set_size(ab, swColW, 32);
    lv_obj_set_pos(ab, W - m - swColW, y + 14);
    lv_obj_add_flag(ab, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(ab, advanced_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *al = lv_label_create(ab);
    lv_label_set_text(al, "Advanced");
    lv_obj_center(al);

    // In the Speed row: Trim, -1.00 % .. as far up as setRefreshHz() allows
    // (the front porch can only shrink so far), and Wobble.
    {
        const JC8012PanelTiming &t = display.panelTiming();
        const double hTotal = JC8012_NATIVE_WIDTH + t.hsync + t.hbp + t.hfp;
        const double minLines = JC8012_NATIVE_HEIGHT + t.vsync + t.vbp + JC8012_TRIM_MIN_VFP;
        const double maxHz = display.pixelClockHz() / (hTotal * minLines);
        g_trimMax = (int)floor((maxHz / g_baseHz - 1) * 1e4);
        if (g_trimMax > 100) g_trimMax = 100;
    }
    const int boxW = 110;
    g_trimLabel = make_label(scr, "");
    lv_obj_set_pos(g_trimLabel, m, y);
    g_trimSlider = make_slider(scr, sliderX, y + 1, sliderW - boxW - 20, TRIM_MIN, g_trimMax, 0, trim_cb);
    {   // green, so Trim doesn't pass for the (blue) Speed slider it replaces
        const lv_color_t green = lv_palette_main(LV_PALETTE_GREEN);
        lv_obj_set_style_bg_color(g_trimSlider, green, LV_PART_MAIN);   // track (theme keeps it faint)
        lv_obj_set_style_bg_color(g_trimSlider, green, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(g_trimSlider, green, LV_PART_KNOB);
    }
    g_wobbleBox = lv_checkbox_create(scr);
    lv_checkbox_set_text(g_wobbleBox, "Wobble");
    lv_obj_set_style_text_color(g_wobbleBox, lv_color_white(), 0);
    lv_obj_set_pos(g_wobbleBox, sliderX + sliderW - boxW, y - 4);
    lv_obj_add_event_cb(g_wobbleBox, wobble_cb, LV_EVENT_VALUE_CHANGED, NULL);
    trim_apply(0);
    lv_timer_create(wobble_tick, 50, NULL);

    // In the note's place: the rate selector (restarts at the new rate).
    static const char *map[RATE_COUNT + 1];
    for (size_t i = 0; i < RATE_COUNT; i++) map[i] = kRates[i].name;
    map[RATE_COUNT] = "";
    lv_obj_t *bm = lv_buttonmatrix_create(scr);
    lv_buttonmatrix_set_map(bm, map);
    lv_buttonmatrix_set_button_ctrl_all(bm, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(bm, true);
    lv_buttonmatrix_set_button_ctrl(bm, g_rateIdx, LV_BUTTONMATRIX_CTRL_CHECKED);
    lv_obj_set_size(bm, W / 2 + 70, 44);
    lv_obj_set_style_pad_all(bm, 3, 0);
    lv_obj_set_style_pad_gap(bm, 3, 0);
    lv_obj_align(bm, LV_ALIGN_BOTTOM_LEFT, m, -6);
    lv_obj_add_event_cb(bm, rate_matrix_cb, LV_EVENT_VALUE_CHANGED, NULL);

    g_basic[0] = g_speedLabel; g_basic[1] = speedSlider; g_basic[2] = note;
    g_adv[0] = g_trimLabel; g_adv[1] = g_trimSlider; g_adv[2] = g_wobbleBox; g_adv[3] = bm;
    if (g_advanced) lv_obj_add_state(ab, LV_STATE_CHECKED);
    show_advanced(g_advanced);
#endif

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

    // Refresh rate.  Off by default: the panel runs at 52.0 Hz (60 MHz pixel
    // clock).  Uncomment ONE line to run at a video frame rate instead, so
    // each video frame is shown for a whole number of refreshes (24p and
    // 23.976p at 48/47.952, 25p at 50, 29.97p at 59.94, 30p at 60).  See
    // JC8012_TIMING_* in the library header for the clocks and porches.
    // At 59.94/60 Hz each 16-row chunk has ~200 us instead of 234, so the
    // see-through balls near 200 run late (the readout shows it).
    //display.setPanelTiming(JC8012_TIMING_47_952HZ);    // 48000/1001
    //display.setPanelTiming(JC8012_TIMING_48HZ);
    //display.setPanelTiming(JC8012_TIMING_50HZ);
    //display.setPanelTiming(JC8012_TIMING_59_94HZ);     // 60000/1001
    //display.setPanelTiming(JC8012_TIMING_60HZ);
    //display.setPanelTiming(JC8012_TIMING_40HZ);       // more time per chunk
    //display.setPanelTiming(JC8012_TIMING_30HZ);       // more still
#if RATE_CONTROLS
    if (s_rateMagic == RATE_MAGIC && s_rateIdx < RATE_COUNT) {
        g_rateIdx = s_rateIdx;
        g_advanced = s_advanced;
    }
    s_rateMagic = 0;   // a crash loop at a bad rate falls back to the default
    display.setPanelTiming(kRates[g_rateIdx].timing);
    Serial0.printf("Rate: %s Hz\n", kRates[g_rateIdx].name);
#endif

    lv_setup.begin(0);   // portrait: the strip at the bottom of the ball area

    // The real frame period: wait for a few frames.
    uint32_t f0 = display.getFrameCount();
    while (display.getFrameCount() - f0 < 3) delay(5);
    uint32_t periodUs = display.framePeriodUs();
    Serial0.printf("Panel frame period %u us (%.3f Hz), pixel clock %u Hz\n", (unsigned)periodUs,
                   1e6f / periodUs, (unsigned)display.pixelClockHz());
    g_hz10 = periodUs ? (10000000 + periodUs / 2) / periodUs : 0;

    g_ballRows = display.height() - STRIP_ROWS;
    balls_init(display.width(), g_ballRows, periodUs);
    balls_start_helper(RASTER_CORE ^ 1);   // half of each chunk on the other core
#if RATE_CONTROLS
    g_baseHz = display.refreshHz();
#endif
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

#if RATE_CONTROLS
// Serial 't': measure setRefreshHz() at fixed offsets from the timing's own
// rate, 10 s each (about 90 s, and the screen stops updating meanwhile).
// The rate is timed at frame edges by polling the frame counter.
static double trim_measure(uint32_t ms) {
    uint32_t f = display.getFrameCount();
    while (display.getFrameCount() == f) {}
    uint32_t f0 = display.getFrameCount();
    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < (int64_t)ms * 1000) {}
    uint32_t g = display.getFrameCount();
    while (display.getFrameCount() == g) {}
    int64_t t1 = esp_timer_get_time();
    return (double)(display.getFrameCount() - f0) * 1e6 / (double)(t1 - t0);
}
// 't': step the rate by fixed offsets from the timing's own and measure each.
static void trim_test() {
    wobble_set(false);
    display.setRefreshHz(0);
    const double base = display.refreshHz();
    static const double ppm[] = { 0, 100, -100, 1000, -1000, 0.5, 5000, -5000, 0 };
    for (double p : ppm) {
        bool ok = display.setRefreshHz(base * (1 + p * 1e-6));
        uint32_t u0 = display.getUnderrunCount();
        double want = display.refreshHz();
        double got = trim_measure(10000);
        Serial0.printf("trim %+8.1f ppm: %s set %.6f Hz, measured %.6f Hz (%+.2f ppm), underruns %u\n",
                       p, ok ? "ok     " : "REFUSED", want, got, (got / want - 1) * 1e6,
                       (unsigned)(display.getUnderrunCount() - u0));
    }
    display.setRefreshHz(0);
    trim_apply(lv_slider_get_value(g_trimSlider));   // back to the slider's trim
}
#endif

void loop() {
    int c = Serial0.available() ? Serial0.read() : -1;
    if (c == 'b') bench();
#if RATE_CONTROLS
    if (c == 't') trim_test();
    if (c == 'w') { wobble_set(!g_wobble); Serial0.printf("wobble %s\n", g_wobble ? "on" : "off"); }
    if (c == 'n') rate_select(g_rateIdx + 1);
#endif
    lv_timer_handler();
    delay(5);
}
