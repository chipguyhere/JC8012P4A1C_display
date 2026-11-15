/*
 * balls.cpp - Bouncing balls rasterized into the bounce buffer, row by row.
 *
 * Ported from the RasterBouncingBalls example of the
 * chipguy_Waveshare-ESP32-S3-Touch-LCD-7-7B_display library.  The drawing is
 * the same; the S3's 128-bit SIMD stores are replaced with plain 32-bit C
 * (the P4's SIMD is a different instruction set), which the P4 at 360 MHz
 * comfortably affords.
 *
 * Runs in the display's raster task (core 0, top priority), with a helper
 * task on the other core, so it is written to be short and predictable:
 *   - All code is IRAM_ATTR and all data is in internal RAM: no flash or
 *     PSRAM access to wait for.
 *   - Each chunk is split: the raster task draws the top half of its rows
 *     and the helper draws the bottom half, at the same time.
 *   - The physics runs on the helper after the frame's last chunk, while the
 *     panel scans the LVGL strip, and produces the next frame's snapshot.
 *     The first chunk of each frame switches to it.  Every chunk of a frame
 *     draws from one snapshot, so the picture can't tear.
 *   - Span ends are anti-aliased from the circle's exact half-width, looked
 *     up in a table, so no square roots are taken while drawing.
 *   - The background gradient is ordered-dithered (4x4 Bayer): each row is a
 *     repeating 4-pixel pattern of two neighbouring RGB565 shades, so the
 *     gradient has no visible steps.
 *   - See-through balls are 75 % opaque: 3/4 of the ball's colour, worked out
 *     once per frame, plus 1/4 of what's under it -- one shift, mask and add
 *     per two pixels.  It reads the pixels back, so it still costs more than
 *     opaque balls.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 */

// The drawing is the hot path; the core builds at -Os.
#pragma GCC optimize ("O2")

#include "balls.h"
#include <Arduino.h>
#include <math.h>
#include <esp_random.h>
#include <esp_cpu.h>

#define MAX_D   64          // largest diameter the half-width table covers
#define MAX_W   800         // widest ball area supported
#define MAX_H   1280

struct Ball {
    float x, y;     // top-left, pixels
    float vx, vy;   // pixels per 16 ms
    int   d;        // diameter
    uint16_t color; // RGB565
};

// What a chunk needs to draw one ball, fixed for the frame.
struct DrawBall {
    uint32_t c2;    // colour, two pixels
    uint32_t c34;   // 3/4 of the colour, two pixels (see quarter())
    int32_t  cx8;   // centre x, 1/256 pixel
    int16_t  top;   // first row
    int16_t  d;     // diameter (rows, and the half-width table to use)
    uint32_t fg;    // colour, spread for blending (see blend())
};

// Two snapshots: one being drawn, the next being computed.
struct Snapshot {
    DrawBall b[BALLS_MAX];
    int      n;
    bool     seeThrough;
};

static Ball      s_balls[BALLS_MAX];
static Snapshot  s_snap[2];
static const Snapshot* s_cur = &s_snap[0];  // what this frame draws
static volatile bool   s_nextReady = false; // s_snap[other] holds the next frame
static uint16_t  s_hw[MAX_D + 1][MAX_D];   // half-width per diameter and row, 1/256 px
// Background per row: the 4-pixel dither pattern, as two 32-bit words.
static uint32_t  s_bg[MAX_H][2];

static int       s_W = 0, s_H = 0;
static float     s_gravity = 0, s_stepTicks = 1;
static int       s_minD = 0, s_maxD = 0;
static uint32_t  s_mhz = 360;
static volatile int      s_count = 100;
static volatile int      s_speed = 20;     // 1..100, 100 = the LVGL demo's speed
static volatile bool     s_seeThrough = true;
static volatile uint32_t s_frames = 0;
static uint32_t  s_lastFrame = 0xFFFFFFFF;

void balls_set_count(int n) { s_count = n < 1 ? 1 : n > BALLS_MAX ? BALLS_MAX : n; }
void balls_set_speed(int v) { s_speed = v; }
void balls_set_see_through(bool on) { s_seeThrough = on; }
uint32_t balls_frames(void) { return s_frames; }

/* ----------------------------------------------------------------- setup */

static inline float frand(float a, float b) { return a + (esp_random() / 4294967296.0f) * (b - a); }
static inline int irand(int a, int b) { return a + (int)(esp_random() % (uint32_t)(b - a + 1)); }

static inline uint16_t rgb565(int r, int g, int b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// Upward speed that tops out between 30 % and 100 % of the area's height.
static float launch_speed(void) {
    return sqrtf(2.0f * s_gravity * frand(0.30f, 1.0f) * (float)s_H);
}

static void init_ball(Ball* b) {
    b->d  = irand(s_minD, s_maxD);
    b->x  = frand(0, (float)(s_W - b->d));
    b->y  = frand(0, (float)(s_H - b->d));
    b->vx = frand(s_W * 0.003f, s_W * 0.010f) * ((esp_random() & 1) ? 1.0f : -1.0f);
    b->vy = -launch_speed();
    b->color = rgb565(64 + esp_random() % 192, 64 + esp_random() % 192, 64 + esp_random() % 192);
}

// A quarter of each RGB565 field, two pixels per word: shift right two, and
// clear the bits that fell in from the field above (R bits 11-13, G 5-8, B 0-2
// survive).  c - quarter(c) is three quarters, and 3/4 a + 1/4 b never
// overflows a field (at most 24 of 31, 48 of 63 before the 1/4 b is added).
#define QUARTER_MASK 0x39E739E7u
static inline uint32_t IRAM_ATTR quarter(uint32_t c) { return (c >> 2) & QUARTER_MASK; }

static void step_into(Snapshot* out);

void balls_init(int width, int height, uint32_t framePeriodUs) {
    s_W = width > MAX_W ? MAX_W : width;
    s_H = height > MAX_H ? MAX_H : height;
    s_mhz = getCpuFrequencyMhz();
    // The same physics as the LVGL BouncingBalls demo, whose units are per
    // 16 ms tick.  One panel frame is this many ticks.
    s_stepTicks = framePeriodUs ? framePeriodUs / 16000.0f : 1.0f;
    s_gravity = s_H * 0.0010f;
    int base = s_W < s_H ? s_W : s_H;
    s_minD = base / 16; if (s_minD < 12) s_minD = 12;
    s_maxD = base / 9;  if (s_maxD > MAX_D) s_maxD = MAX_D;
    if (s_maxD < s_minD + 2) s_maxD = s_minD + 2;

    // Half-width of a circle of diameter d at the middle of each of its rows.
    for (int d = 1; d <= MAX_D; d++) {
        float r = d * 0.5f;
        for (int row = 0; row < d; row++) {
            float dy = row + 0.5f - r;
            float h2 = r * r - dy * dy;
            s_hw[d][row] = h2 > 0 ? (uint16_t)(sqrtf(h2) * 256.0f + 0.5f) : 0;
        }
    }
    // A vertical gradient, dark blue at the top to near black at the floor,
    // ordered-dithered so its 565 steps don't show as bands.
    static const uint8_t bayer[4][4] = { { 0, 8, 2, 10 }, { 12, 4, 14, 6 },
                                         { 3, 11, 1, 9 }, { 15, 7, 13, 5 } };
    for (int y = 0; y < s_H; y++) {
        float t = (float)y / (float)(s_H - 1);
        // Channel values in their own units: red and blue 0..31, green 0..63.
        float r = 5.0f - 3.5f * t, g = 9.0f - 6.0f * t, b = 17.0f - 13.0f * t;
        uint16_t px[4];
        for (int x = 0; x < 4; x++) {
            float th = (bayer[y & 3][x] + 0.5f) / 16.0f;
            int r5 = (int)(r + th), g6 = (int)(g + th), b5 = (int)(b + th);
            px[x] = (uint16_t)((r5 << 11) | (g6 << 5) | b5);
        }
        s_bg[y][0] = px[0] | ((uint32_t)px[1] << 16);
        s_bg[y][1] = px[2] | ((uint32_t)px[3] << 16);
    }
    for (int i = 0; i < BALLS_MAX; i++) init_ball(&s_balls[i]);
    // The first frame switches to s_snap[1].
    s_cur = &s_snap[0];
    step_into(&s_snap[1]);
    s_nextReady = true;
}

/* ----------------------------------------------------------------- physics */

// Advance one frame -- always exactly one frame's worth of time, so a late
// or missed frame can't make the motion jump -- and snapshot what to draw.
static void IRAM_ATTR step_into(Snapshot* out) {
    const int n = s_count;
    // s_speed / 100 of the LVGL demo's speed.
    const float s = s_stepTicks * s_speed * 0.01f;
    for (int i = 0; i < n; i++) {
        Ball* b = &s_balls[i];
        b->vy += s_gravity * s;
        b->x  += b->vx * s;
        b->y  += b->vy * s;
        if (b->x < 0)                { b->x = 0;              b->vx = -b->vx; }
        if (b->x + b->d > s_W)       { b->x = s_W - b->d;     b->vx = -b->vx; }
        if (b->y < 0)                { b->y = 0; if (b->vy < 0) b->vy = -b->vy; }
        if (b->y + b->d > s_H)       { b->y = s_H - b->d;     b->vy = -launch_speed(); }

        DrawBall* o = &out->b[i];
        uint32_t c2 = b->color | ((uint32_t)b->color << 16);
        o->c2  = c2;
        o->c34 = c2 - quarter(c2);
        o->cx8 = (int32_t)((b->x + b->d * 0.5f) * 256.0f);
        o->top = (int16_t)(b->y + 0.5f);
        o->d   = (int16_t)b->d;
        o->fg  = c2 & 0x07E0F81F;
    }
    out->n = n;
    out->seeThrough = s_seeThrough;
}

/* ----------------------------------------------------------------- drawing */

// fg (already spread) over bg at alpha a/32.
static inline uint16_t IRAM_ATTR blend(uint32_t fg, uint16_t bg, uint32_t a) {
    uint32_t b = (bg | ((uint32_t)bg << 16)) & 0x07E0F81F;
    uint32_t r = ((((fg - b) * a) >> 5) + b) & 0x07E0F81F;
    return (uint16_t)(r | (r >> 16));
}

// 75 % of the ball over b, two RGB565 pixels per word.
static inline uint32_t IRAM_ATTR mix3(const DrawBall& ball, uint32_t b) { return ball.c34 + quarter(b); }

// The solid inside of a span, pixels [x, xe): a half-word to reach 32-bit
// alignment, then whole words, then a final half-word.
static inline void IRAM_ATTR inside(uint16_t* row, int x, int xe, const DrawBall& b, bool see) {
    const uint32_t c2 = b.c2;
    uint16_t* p = row + x;
    int n = xe - x;
    if (n <= 0) return;
    if ((uintptr_t)p & 2) { *p = (uint16_t)(see ? mix3(b, *p) : c2); p++; n--; }
    uint32_t* q = (uint32_t*)p;
    int w = n >> 1;
    if (see) { for (int i = 0; i < w; i++) q[i] = mix3(b, q[i]); }
    else     { for (int i = 0; i < w; i++) q[i] = c2; }
    if (n & 1) { uint16_t* t = (uint16_t*)(q + w); *t = (uint16_t)(see ? mix3(b, *t) : c2); }
}

// One anti-aliased span, [l8, r8) in 1/256 pixel.  Edge pixels take the
// ball at their coverage (3/4 of it when see-through).
static inline void IRAM_ATTR span(uint16_t* row, int32_t l8, int32_t r8, const DrawBall& b, bool see) {
    const int32_t lim = s_W << 8;
    if (l8 < 0) l8 = 0;
    if (r8 > lim) r8 = lim;
    if (r8 <= l8) return;
    const uint32_t k = see ? 3 : 4;                   // coverage scale, in quarters
    int xl = l8 >> 8, xr = r8 >> 8;
    if (xl == xr) {                                   // inside one pixel
        row[xl] = blend(b.fg, row[xl], ((uint32_t)(r8 - l8) * k) >> 5);
        return;
    }
    row[xl] = blend(b.fg, row[xl], ((256 - (l8 & 255)) * k) >> 5);
    if (r8 & 255) row[xr] = blend(b.fg, row[xr], ((uint32_t)(r8 & 255) * k) >> 5);
    inside(row, xl + 1, xr, b, see);
}

// Draw rows y .. y+rows-1 of snapshot sn into dst (row y first).
static void IRAM_ATTR draw_rows(const Snapshot* sn, uint16_t* dst, int y, int rows) {
    const int W = s_W;
    // The balls these rows cross.
    uint8_t sel[BALLS_MAX];
    int ns = 0;
    for (int i = 0; i < sn->n; i++) {
        const DrawBall& b = sn->b[i];
        if (b.top < y + rows && b.top + b.d > y) sel[ns++] = (uint8_t)i;
    }
    const bool see = sn->seeThrough;
    for (int r = 0; r < rows; r++) {
        const int py = y + r;
        uint16_t* row = dst + (size_t)r * W;
        uint32_t* q = (uint32_t*)row;                 // rows are 4-byte aligned
        // Background, two words at a time (the pattern is 4 pixels).
        const uint32_t w0 = py < s_H ? s_bg[py][0] : 0, w1 = py < s_H ? s_bg[py][1] : 0;
        for (int k = 0; k < W / 2; k += 2) { q[k] = w0; q[k + 1] = w1; }
        if (py >= s_H) continue;                      // below the area: plain black
        for (int k = 0; k < ns; k++) {                // balls, in order: later on top
            const DrawBall& b = sn->b[sel[k]];
            int br = py - b.top;
            if ((unsigned)br >= (unsigned)b.d) continue;
            int32_t h = s_hw[b.d][br];
            span(row, b.cx8 - h, b.cx8 + h, b, see);
        }
    }
}

/* ----------------------------------------------------------------- helper core */

// The helper task draws the bottom half of every chunk at the same time as
// the raster task draws the top half, and after the frame's last chunk it runs
// the physics for the next frame.  One job at a time: the raster task hands it
// one by notification and waits for it (s_helpDone) before handing another.
static TaskHandle_t        s_helper = nullptr;
static volatile int        s_helpJob = 0;       // 0 = draw rows, 1 = physics
static const Snapshot*     s_helpSnap = nullptr;
static uint16_t* volatile  s_helpDst = nullptr;
static volatile int        s_helpY = 0, s_helpRows = 0;
static volatile bool       s_helpDone = true;

static Snapshot* next_snap() { return s_cur == &s_snap[0] ? &s_snap[1] : &s_snap[0]; }

static void helper_task(void*) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_helpJob == 1) {
            step_into(next_snap());
            s_nextReady = true;
        } else {
            draw_rows(s_helpSnap, s_helpDst, s_helpY, s_helpRows);
        }
        s_helpDone = true;
    }
}

void balls_start_helper(int core) {
    if (s_helper) return;
    // Above loop() and LVGL; a job is well under a millisecond.
    xTaskCreatePinnedToCore(helper_task, "balls_help", 3072, nullptr,
                            configMAX_PRIORITIES - 2, &s_helper, core);
}

static inline void IRAM_ATTR help_wait() { while (!s_helpDone) {} }

volatile uint32_t g_physWaitUs = 0;   // the first chunk's wait for the physics (worst)

void IRAM_ATTR balls_raster(uint16_t* dst, int y, int rows, uint32_t frame, void* ctx) {
    (void)ctx;
    if (frame != s_lastFrame) {        // first chunk of a new panel frame
        s_lastFrame = frame;
        uint32_t t = esp_cpu_get_cycle_count();
        help_wait();                   // the physics, normally done long ago, during the LVGL strip
        // If the frame's last chunk was skipped (see setRasterRows), nobody
        // started the physics: do it here.
        if (!s_nextReady) step_into(next_snap());
        t = (esp_cpu_get_cycle_count() - t) / s_mhz;
        if (t > g_physWaitUs) g_physWaitUs = t;
        s_cur = next_snap();
        s_nextReady = false;
        s_frames = s_frames + 1;
    }
    if (!s_helper) {
        draw_rows(s_cur, dst, y, rows);
        if (y + rows >= s_H) { step_into(next_snap()); s_nextReady = true; }
        return;
    }
    const int top = rows / 2;
    help_wait();
    s_helpJob = 0;
    s_helpSnap = s_cur;
    s_helpDst = dst + (size_t)top * s_W;
    s_helpY = y + top;
    s_helpRows = rows - top;
    s_helpDone = false;
    xTaskNotifyGive(s_helper);
    draw_rows(s_cur, dst, y, top);
    help_wait();                       // the other half, from the other core
    if (y + rows >= s_H) {             // the frame's last chunk: next frame's physics
        s_helpJob = 1;
        s_helpDone = false;
        xTaskNotifyGive(s_helper);
    }
}
