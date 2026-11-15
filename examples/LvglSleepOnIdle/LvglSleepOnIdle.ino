/*******************************************************************************
 * JC8012P4A1C LvglSleepOnIdle
 *
 * A battery-friendly LVGL app: after IDLE_SECONDS without a touch the board
 * goes to sleep — screen off, chip in light sleep — and a swipe across the
 * screen (or the BOOT button) brings it straight back where it was.
 *
 * How long a battery lasts in each state (measured on the board, Guition's
 * ~2.5-3 Ah cell; floors vary a little between boards):
 *
 *   awake, backlight on       well over 1 A at full brightness
 *   asleep, swipe to wake     ~16 mA  (touch checked every 3 s)  ~a week
 *   asleep, tap to wake       ~45 mA  (change SLEEP_MODE below)  ~2.5 days
 *   asleep, BOOT to wake      ~10 mA                             ~11 days
 *
 * Swipe, don't press: a finger already resting on the screen when touch is
 * checked isn't seen, so move it across the glass for up to 3 seconds.  The
 * finger that wakes the board is ignored until it lifts, so it can't press a
 * button by accident.
 *
 * The Wi-Fi chip is switched off at the start (board.setRadioPower(false)):
 * it would otherwise cost ~22 mA, awake or asleep.  Remove that line if your
 * app uses Wi-Fi.
 *
 * Requires the following libraries:
 *   - chipguy_JC8012P4A1C_display  (display + touch driver, this library)
 *   - chipguy_JC8012P4A1C_board  (shared I2C bus + pin map)
 *   - lvgl 9
 *
 * Arduino IDE Board Settings: ESP32P4 Dev Module, PSRAM Enabled, Flash Size
 * 16MB, Partition Scheme Custom, USB CDC On Boot Enabled.
 ******************************************************************************/

#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"

static const uint32_t IDLE_SECONDS = 30;

// 0 = swipe to wake (~16 mA), 1 = tap to wake (~45 mA), 2 = BOOT only (~10 mA)
#define SLEEP_MODE 0

static uint32_t  lastActivity = 0;
static bool      sleepRequested = false;
static uint32_t  sleeps = 0;
static lv_obj_t *statusLabel;
static lv_obj_t *countdownLabel;

static void noteActivity() { lastActivity = millis(); }

static void onSleepButton(lv_event_t *) { sleepRequested = true; }

static void buildUi() {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x102030), 0);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Sleep on idle");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 60);

    statusLabel = lv_label_create(scr);
    lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(statusLabel, lv_color_hex(0xA0C0E0), 0);
    lv_label_set_text(statusLabel, "Awake. Touch anything to stay awake.");
    lv_obj_set_width(statusLabel, lv_pct(90));
    lv_label_set_long_mode(statusLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(statusLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(statusLabel, LV_ALIGN_CENTER, 0, -80);

    countdownLabel = lv_label_create(scr);
    lv_obj_set_style_text_font(countdownLabel, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(countdownLabel, lv_color_white(), 0);
    lv_obj_align(countdownLabel, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *btn = lv_button_create(scr);
    lv_obj_set_size(btn, 320, 110);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 160);
    lv_obj_add_event_cb(btn, onSleepButton, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, "Sleep now");
    lv_obj_set_style_text_font(bl, &lv_font_montserrat_28, 0);
    lv_obj_center(bl);
}

void setup() {
    Serial0.begin(115200);

    lv_setup.begin();
    board.setRadioPower(false);          // no Wi-Fi in this sketch: save ~22 mA
    lv_setup.onTouch = noteActivity;     // every touch resets the idle timer

    buildUi();
    lastActivity = millis();
}

void loop() {
    lv_timer_handler();
    delay(5);

    uint32_t idle = millis() - lastActivity;
    static uint32_t shown = UINT32_MAX;
    uint32_t left = idle >= IDLE_SECONDS * 1000 ? 0 : (IDLE_SECONDS * 1000 - idle + 999) / 1000;
    if (left != shown) {
        shown = left;
        lv_label_set_text_fmt(countdownLabel, "Sleeping in %lu s", (unsigned long)left);
    }

    if (sleepRequested || idle >= IDLE_SECONDS * 1000) {
        sleepRequested = false;
        Serial0.println("Going to sleep");
        Serial0.flush();

#if SLEEP_MODE == 1
        auto why = lv_setup.sleepUntilTap();
#elif SLEEP_MODE == 2
        auto why = lv_setup.sleepUntilButton();
#else
        auto why = lv_setup.sleepUntilSwipe();   // touch checked every 3 s
#endif

        sleeps++;
        const char *how = why == display.WAKE_SWIPE ? "a swipe"
                        : why == display.WAKE_TAP   ? "a tap" : "the BOOT button";
        lv_label_set_text_fmt(statusLabel, "Woken by %s (slept %lu times)", how, (unsigned long)sleeps);
        Serial0.printf("Woke: %s\n", how);
        lastActivity = millis();
    }
}
