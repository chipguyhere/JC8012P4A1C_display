/*
 * JD9365 MIPI-DSI display driver for the JC8012P4A1C ESP32-P4 LCD.
 * See chipguy_JC8012P4A1C_display.h for the wiring and rotation overview.
 *
 * Copyright (c) 2025-2026 chipguyhere
 * MIT License
 */

#ifdef ARDUINO_ESP32P4_DEV

#include "chipguy_JC8012P4A1C_display.h"
#include "pins_config.h"
#include "esp_lcd_panel_dpi_bb.h"
#include "esp_lcd_jd9365.h"
#include "jc8012_raster_copy.h"

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_ldo_regulator.h"
#include "driver/gpio.h"
#include "driver/ppa.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_cache.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_attr.h"

// Display hardware configuration
#define MIPI_DSI_PHY_PWR_LDO_CHAN 3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
#define MIPI_DPI_PX_FORMAT LCD_COLOR_PIXEL_FORMAT_RGB565

#define PANEL_WIDTH  JC8012_NATIVE_WIDTH
#define PANEL_HEIGHT JC8012_NATIVE_HEIGHT
#define PANEL_FB_SIZE ((size_t)PANEL_WIDTH * PANEL_HEIGHT * sizeof(uint16_t))

// Panel reset timing.
//
// The JD9365 is held in reset for the whole host-side bring-up (DSI PHY LDO,
// DSI bus, DBI IO, DPI panel) rather than being pulsed afterwards.  That gives
// the assertion a much longer floor than the old 10 ms pulse, and keeps the
// panel from watching the D-PHY lanes come alive underneath it.
//
// This matters most on a warm reboot.  A watchdog reset restarts the SoC but
// never power-cycles the panel: it keeps its rails and whatever internal state
// it was in when the DSI stream stopped mid-transfer.  There is no panel
// power-enable GPIO on this board (pins_config.h has only LCD_RST and LCD_LED),
// so the reset line is the only recovery lever firmware has.
#define PANEL_RESET_MIN_LOW_MS  50    // floor on the low assertion during init
#define PANEL_RESET_SETTLE_MS   200   // after release, before the first command
#define PANEL_RESET_RECOVERY_MS 1000  // long assertion before a failure reboot

// How many consecutive failed bring-ups to reboot through before giving up.
// Without a bound, a permanently dead panel would reboot-loop forever, which
// hides the serial log and makes the board undiagnosable.
#define PANEL_INIT_MAX_ATTEMPTS 3

static const char *TAG = "JC8012P4A1C";

// Consecutive failed display bring-ups.  RTC_NOINIT memory survives a software
// reset (both our own esp_restart() and a watchdog reboot) but holds garbage
// after a true power-on, hence the magic-value seed.  Cleared on every
// successful begin(), so the budget is "consecutive failures", not "failures
// ever".
#define PANEL_ATTEMPT_MAGIC 0x4A433830u
RTC_NOINIT_ATTR static uint32_t s_panel_init_magic;
RTC_NOINIT_ATTR static uint32_t s_panel_init_attempts;

// PPA rotation angle for each rotation index.
// Rotation follows the usual Arduino display convention: 1 (90) turns the
// picture 90 degrees clockwise.  The PPA's angles are counter-clockwise, so
// 1 and 3 swap on the way in.
static ppa_srm_rotation_angle_t ppa_angle_for(uint8_t rotation) {
    switch (rotation) {
        case 1:  return PPA_SRM_ROTATION_ANGLE_270;
        case 2:  return PPA_SRM_ROTATION_ANGLE_180;
        case 3:  return PPA_SRM_ROTATION_ANGLE_90;
        default: return PPA_SRM_ROTATION_ANGLE_0;
    }
}

// Run fn(arg) on `core` and wait for it: in a short task pinned there, unless
// that is this core already (or core < 0).  Interrupts allocated inside it are
// taken on that core.
struct RunOnCore { void (*fn)(void *); void *arg; SemaphoreHandle_t done; };
static void run_on_core_task(void *p)
{
    RunOnCore *r = (RunOnCore *)p;
    r->fn(r->arg);
    xSemaphoreGive(r->done);
    vTaskDelete(nullptr);
}
static void run_on_core(int core, void (*fn)(void *), void *arg)
{
    if (core < 0 || core == xPortGetCoreID()) { fn(arg); return; }
    RunOnCore r = { fn, arg, xSemaphoreCreateBinary() };
    xTaskCreatePinnedToCore(run_on_core_task, "jc8012_on_core", 8192, &r, uxTaskPriorityGet(nullptr),
                            nullptr, core);
    xSemaphoreTake(r.done, portMAX_DELAY);
    vSemaphoreDelete(r.done);
}

chipguy_JC8012P4A1C_display::chipguy_JC8012P4A1C_display(JC8012P4A1C_Board &board)
    : _board(board), _touch(board.Wire, TP_RST, TP_INT)
{
}

// Drive LCD_RST low (asserted).  Configures the pin every time so this is safe
// to call at any point, including before the DSI PHY is powered and from the
// failure path after a partial bring-up.
void chipguy_JC8012P4A1C_display::assertPanelReset()
{
    gpio_config_t rst_gpio_config = {
        .pin_bit_mask = 1ULL << LCD_RST,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&rst_gpio_config);
    if (ret != ESP_OK) {
        // Not fatal on its own, but worth knowing about: every recovery path
        // below depends on this pin actually being drivable.
        ESP_LOGE(TAG, "Reset GPIO config failed: %s", esp_err_to_name(ret));
    }
    gpio_set_level((gpio_num_t)LCD_RST, 0);
}

// Release LCD_RST and wait out the panel's post-reset settle before anything
// tries to talk to it.
void chipguy_JC8012P4A1C_display::releasePanelFromReset()
{
    gpio_set_level((gpio_num_t)LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(PANEL_RESET_SETTLE_MS));
}

// Failure path for any step of the display bring-up.  Parks the panel in reset
// with the backlight off, holds it there far longer than a normal init pulse,
// and reboots so the whole sequence gets a clean run from power-on state.
//
// Bounded by PANEL_INIT_MAX_ATTEMPTS: once the budget is spent this returns
// instead of rebooting, leaving the board halted with a readable serial log.
// Always returns false so callers can `return panelInitFailed("...", ret)`.
// Pass ESP_OK for steps that have no esp_err_t of their own.
bool chipguy_JC8012P4A1C_display::panelInitFailed(const char *step, esp_err_t err)
{
    if (err == ESP_OK) {
        ESP_LOGE(TAG, "Display init failed at: %s", step);
    } else {
        ESP_LOGE(TAG, "Display init failed at: %s (%s)", step, esp_err_to_name(err));
    }

    setBacklight(0);
    assertPanelReset();

    if (s_panel_init_attempts >= PANEL_INIT_MAX_ATTEMPTS) {
        ESP_LOGE(TAG, "Giving up after %u consecutive attempts. "
                      "Panel left in reset, backlight off.",
                 (unsigned)s_panel_init_attempts);
        return false;
    }

    ESP_LOGE(TAG, "Holding panel in reset for %d ms, then rebooting "
                  "(attempt %u of %d)",
             PANEL_RESET_RECOVERY_MS, (unsigned)s_panel_init_attempts,
             PANEL_INIT_MAX_ATTEMPTS);
    vTaskDelay(pdMS_TO_TICKS(PANEL_RESET_RECOVERY_MS));

    esp_restart();
    return false;  // not reached
}

// Common hardware init (DSI bus, panel, reset, vendor commands, backlight).
bool chipguy_JC8012P4A1C_display::initHardware()
{
    esp_err_t ret;

    // Assert reset before anything else and hold it for the whole host-side
    // bring-up below.  See the PANEL_RESET_* notes at the top of this file.
    assertPanelReset();
    TickType_t reset_asserted_at = xTaskGetTickCount();

    // Power on MIPI DSI PHY
    esp_ldo_channel_handle_t ldo_mipi_phy = nullptr;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ret = esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy);
    if (ret != ESP_OK) {
        return panelInitFailed("esp_ldo_acquire_channel (MIPI DSI PHY)", ret);
    }
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");

    // Backlight off during init
    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << LCD_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    ret = gpio_config(&bk_gpio_config);
    if (ret != ESP_OK) {
        return panelInitFailed("gpio_config (backlight)", ret);
    }
    gpio_set_level((gpio_num_t)LCD_LED, 0);

    // Create MIPI DSI bus
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = nullptr;
    esp_lcd_dsi_bus_config_t bus_config = JD9365_PANEL_BUS_DSI_2CH_CONFIG();
    ret = esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus);
    if (ret != ESP_OK) {
        return panelInitFailed("esp_lcd_new_dsi_bus", ret);
    }

    // Install MIPI DSI LCD control panel IO
    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    esp_lcd_dbi_io_config_t dbi_config = JD9365_PANEL_IO_DBI_CONFIG();
    ret = esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &_io_handle);
    if (ret != ESP_OK) {
        return panelInitFailed("esp_lcd_new_panel_io_dbi", ret);
    }

    // Create DPI panel with double-buffering (native 800x1280)
    ESP_LOGI(TAG, "Creating panel %dx%d with %d framebuffers", PANEL_WIDTH, PANEL_HEIGHT, _num_fbs);

    chipguy_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 60,
        .pixel_format = MIPI_DPI_PX_FORMAT,
        .in_color_format = (lcd_color_format_t)0,
        .out_color_format = (lcd_color_format_t)0,
        .num_fbs = _num_fbs,
        .video_timing = {
            .h_size = PANEL_WIDTH,
            .v_size = PANEL_HEIGHT,
            .hsync_pulse_width = 20,
            .hsync_back_porch = 20,
            .hsync_front_porch = 40,
            .vsync_pulse_width = 4,
            .vsync_back_porch = 8,
            .vsync_front_porch = 20,
        },
        .virtual_v_pixels = _fb_rows,    // 0 = 1280; taller for the terminal's scroll ring
        .num_dma_blocks = _scan_blocks,  // 0 = 8
        .flags = {
            .use_dma2d = false,
            .disable_lp = false,
        },
    };

    ret = chipguy_lcd_new_panel_dpi(mipi_dsi_bus, &dpi_config, &_panel_handle);
    if (ret != ESP_OK) {
        return panelInitFailed("chipguy_lcd_new_panel_dpi", ret);
    }

    // Host side is up; release the panel from reset.  The bring-up above
    // already took some time, so top it up only if it came in under the
    // minimum low assertion.
    TickType_t held_ticks = xTaskGetTickCount() - reset_asserted_at;
    TickType_t min_low_ticks = pdMS_TO_TICKS(PANEL_RESET_MIN_LOW_MS);
    if (held_ticks < min_low_ticks) {
        vTaskDelay(min_low_ticks - held_ticks);
    }
    releasePanelFromReset();
    ESP_LOGI(TAG, "Hardware reset complete (held low >= %d ms, settled %d ms)",
             PANEL_RESET_MIN_LOW_MS, PANEL_RESET_SETTLE_MS);

    // Read LCD ID (for debugging)
    uint8_t lcd_id[3] = {0};
    esp_lcd_panel_io_rx_param(_io_handle, 0x04, lcd_id, 3);
    ESP_LOGI(TAG, "LCD ID: %02X %02X %02X", lcd_id[0], lcd_id[1], lcd_id[2]);

    // Send preliminary init commands.  MADCTL 0xC0 (MY|MX) sets the panel's
    // base scan orientation; logical rotation is layered on top by the PPA.
    esp_lcd_panel_io_tx_param(_io_handle, 0xE0, (uint8_t[]){0x00}, 1);
    esp_lcd_panel_io_tx_param(_io_handle, 0x36, (uint8_t[]){0xC0}, 1);
    esp_lcd_panel_io_tx_param(_io_handle, 0x3A, (uint8_t[]){0x55}, 1);
    esp_lcd_panel_io_tx_param(_io_handle, 0x80, (uint8_t[]){0x01}, 1);

    // Send vendor-specific init commands (defined in esp_lcd_jd9365.c)
    extern const jd9365_lcd_init_cmd_t vendor_specific_init_default[];
    extern const size_t vendor_specific_init_default_size;
    ESP_LOGI(TAG, "Sending %d vendor init commands", vendor_specific_init_default_size);
    for (size_t i = 0; i < vendor_specific_init_default_size; i++) {
        const jd9365_lcd_init_cmd_t *cmd = &vendor_specific_init_default[i];
        if (cmd->data_bytes > 0) {
            esp_lcd_panel_io_tx_param(_io_handle, cmd->cmd, cmd->data, cmd->data_bytes);
        } else {
            esp_lcd_panel_io_tx_param(_io_handle, cmd->cmd, NULL, 0);
        }
        if (cmd->delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(cmd->delay_ms));
        }
    }
    ESP_LOGI(TAG, "Vendor init commands sent");

    // Initialize panel (starts DMA and video mode)
    ESP_LOGI(TAG, "Initializing panel...");
    ret = esp_lcd_panel_init(_panel_handle);
    if (ret != ESP_OK) {
        return panelInitFailed("esp_lcd_panel_init", ret);
    }

    // Send Sleep Out and Display On after video mode is enabled
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_lcd_panel_io_tx_param(_io_handle, 0x11, NULL, 0);  // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_lcd_panel_io_tx_param(_io_handle, 0x29, NULL, 0);  // Display On
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "Display On command sent");

    ESP_LOGI(TAG, "Panel initialized, fb0=%p, fb1=%p",
             getFramebuffer(0), getFramebuffer(1));

    // Clear both native framebuffers to black (all their rows: they may be
    // taller than the panel, see setFramebufferRows())
    size_t fb_bytes = (size_t)PANEL_WIDTH * framebufferRows() * sizeof(uint16_t);
    for (int i = 0; i < _num_fbs; i++) {
        uint16_t *fb = getFramebuffer(i);
        if (fb) {
            memset(fb, 0, fb_bytes);
            esp_cache_msync(fb, fb_bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        }
    }

    // Initialize touch controller.  A watchdog reboot can catch the GSL3680's
    // I2C bus mid-transaction and leave it wedged, so a failure here is worth a
    // reset-and-retry while the budget lasts.  Once it is spent, come up with a
    // working display and touch disabled rather than a dark panel: the display
    // is still useful without touch, and going dark hides the serial log that
    // says what went wrong.
    //
    // The touch controller is on the board's shared I2C bus; starting it here
    // is idempotent, so it doesn't matter whether the sketch already did.
    _board.begin();
    if (_touch.begin()) {
        _touch_ok = true;
        ESP_LOGI(TAG, "Touch controller initialized");
    } else if (_touch.retryMightHelp() && s_panel_init_attempts < PANEL_INIT_MAX_ATTEMPTS) {
        // Only a controller that didn't answer is worth a reboot.  Firmware
        // missing from NVS won't appear on its own: come up without touch.
        return panelInitFailed("gsl3680 touch init", ESP_OK);
    } else {
        _touch_ok = false;
        ESP_LOGE(TAG, "Touch unavailable (%s); continuing with touch disabled.",
                 gsl3680_touch::statusText(_touch.status()));
    }

    return true;
}

// Allocate two full-resolution draw buffers (PSRAM) plus the PPA client.
bool chipguy_JC8012P4A1C_display::allocateDrawBuffers()
{
    size_t buf_size = framebufferSize();

    // Align the draw buffers to the cache line size.  esp_cache_msync() (used
    // here and in flip()) requires the start address AND size to be a multiple
    // of that line size, or it fails with an alignment error and skips the
    // writeback.  The ESP32-P4 data cache line is 128 bytes; the old hardcoded
    // 64 left the buffer only 64-aligned, tripping the error on whichever of the
    // two buffers landed off a 128 boundary.  (This driver is P4-only.)
    size_t cache_line_size = 128;
    // framebufferSize() is a multiple of 128 already, but round up defensively
    // so the whole-buffer msync size stays aligned too.
    buf_size = (buf_size + cache_line_size - 1) & ~(cache_line_size - 1);

    for (int i = 0; i < 2; i++) {
        _draw_buffers[i] = (uint16_t *)heap_caps_aligned_alloc(cache_line_size, buf_size,
                                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!_draw_buffers[i]) {
            ESP_LOGE(TAG, "Failed to allocate %dx%d draw buffer %d", _draw_w, _draw_h, i);
            return false;
        }
        memset(_draw_buffers[i], 0, buf_size);
        esp_cache_msync(_draw_buffers[i], buf_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        ESP_LOGI(TAG, "%dx%d draw buffer %d @ %p", _draw_w, _draw_h, i, _draw_buffers[i]);
    }

    // Create PPA SRM client for rotation
    ppa_client_config_t ppa_cfg = {
        .oper_type = PPA_OPERATION_SRM,
    };
    esp_err_t ret = ppa_register_client(&ppa_cfg, &_ppa_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PPA client registration failed: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "PPA SRM client registered");

    return true;
}

bool chipguy_JC8012P4A1C_display::begin(uint16_t rotation)
{
    // Seed or advance the consecutive-failure budget.  A cold boot leaves
    // RTC_NOINIT memory undefined, so key the seed off both the magic value and
    // the reset reason.  The reason is logged either way: a black screen that
    // correlates with ESP_RST_TASK_WDT / ESP_RST_INT_WDT / ESP_RST_WDT and never
    // with ESP_RST_POWERON is the signature of a panel that came back wedged
    // from a warm reboot.
    esp_reset_reason_t reset_reason = esp_reset_reason();
    if (s_panel_init_magic != PANEL_ATTEMPT_MAGIC ||
        reset_reason == ESP_RST_POWERON) {
        s_panel_init_magic = PANEL_ATTEMPT_MAGIC;
        s_panel_init_attempts = 0;
    }
    s_panel_init_attempts++;
    ESP_LOGI(TAG, "begin(): reset reason %d, display init attempt %u",
             (int)reset_reason, (unsigned)s_panel_init_attempts);

    _rotation = jc8012_normalize_rotation(rotation);

    // Logical size LVGL renders at: landscape swaps width/height.
    if (_rotation == 1 || _rotation == 3) {
        _draw_w = JC8012_NATIVE_HEIGHT;   // 1280
        _draw_h = JC8012_NATIVE_WIDTH;    // 800
    } else {
        _draw_w = JC8012_NATIVE_WIDTH;    // 800
        _draw_h = JC8012_NATIVE_HEIGHT;   // 1280
    }

    // initHardware() has already been through panelInitFailed() (and so may not
    // return at all); allocateDrawBuffers() reports its own detail and is
    // routed through the same recovery here.
    // The panel's interrupts (its DMA's, which drive the raster band, and the
    // bridge's) are taken on the core that brings it up.
    struct InitCall { chipguy_JC8012P4A1C_display *d; bool ok; } ic = { this, false };
    run_on_core(_irq_core, [](void *p) { InitCall *c = (InitCall *)p; c->ok = c->d->initHardware(); }, &ic);
    if (!ic.ok) return false;
    if (_want_draw_buffers && !allocateDrawBuffers()) {
        return panelInitFailed("draw buffer / PPA client allocation", ESP_OK);
    }

    setBacklight(100);

    // Made it: clear the budget so a later warm reboot gets a full set of
    // retries rather than inheriting this boot's count.
    s_panel_init_attempts = 0;

    ESP_LOGI(TAG, "begin(): rotation %d, logical %dx%d", _rotation, _draw_w, _draw_h);
    return true;
}

uint16_t *chipguy_JC8012P4A1C_display::getDrawBuffer(uint8_t index)
{
    if (index >= 2) return nullptr;
    return _draw_buffers[index];
}

uint16_t *chipguy_JC8012P4A1C_display::getFramebuffer(uint8_t index)
{
    if (!_panel_handle || index >= _num_fbs) {
        return nullptr;
    }

    void *fb_addrs[3] = {nullptr, nullptr, nullptr};
    esp_err_t ret = chipguy_lcd_dpi_panel_get_frame_buffer(_panel_handle, _num_fbs,
                                                           &fb_addrs[0], &fb_addrs[1], &fb_addrs[2]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get framebuffer: %s", esp_err_to_name(ret));
        return nullptr;
    }
    return (uint16_t *)fb_addrs[index];
}

void chipguy_JC8012P4A1C_display::setActiveFramebuffer(uint8_t index, bool wait_for_vsync)
{
    if (_panel_handle) {
        chipguy_lcd_dpi_panel_set_active_fb(_panel_handle, index, wait_for_vsync);
    }
}

void chipguy_JC8012P4A1C_display::ppaTaskFunc(void *arg)
{
    auto *self = (chipguy_JC8012P4A1C_display *)arg;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        auto &w = self->_ppa_work;
        uint16_t *src = self->_draw_buffers[w.draw_buf_index];
        uint16_t *target_fb = self->getFramebuffer(w.target_fb_index);

        // No scaling: rotate the full-resolution draw buffer straight into the
        // native framebuffer.  Under 90/270 the in/out dimensions transpose,
        // which is exactly what the PPA produces for those angles.
        ppa_srm_oper_config_t srm_config = {
            .in = {
                .buffer = (const void *)src,
                .pic_w = self->_draw_w,
                .pic_h = self->_draw_h,
                .block_w = self->_draw_w,
                .block_h = self->_draw_h,
                .block_offset_x = 0,
                .block_offset_y = 0,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
            .out = {
                .buffer = (void *)target_fb,
                .buffer_size = PANEL_FB_SIZE,
                .pic_w = PANEL_WIDTH,
                .pic_h = PANEL_HEIGHT,
                .block_offset_x = 0,
                .block_offset_y = 0,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
            .rotation_angle = ppa_angle_for(self->_rotation),
            .scale_x = 1.0f,
            .scale_y = 1.0f,
            .rgb_swap = 0,
            .byte_swap = 0,
            .mode = PPA_TRANS_MODE_BLOCKING,
        };

        esp_err_t ret = ppa_do_scale_rotate_mirror(self->_ppa_client, &srm_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PPA rotate failed: %s", esp_err_to_name(ret));
        } else {
            // Invalidate cache for the target framebuffer (PPA wrote via DMA)
            esp_cache_msync(target_fb, PANEL_FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
            // Switch display to show this framebuffer
            self->setActiveFramebuffer(w.target_fb_index, false);
        }

        xSemaphoreGive(self->_ppa_done_sem);
    }
}

void chipguy_JC8012P4A1C_display::flip(uint8_t draw_buf_index, uint8_t target_fb_index)
{
    if (!_ppa_client || draw_buf_index >= 2) return;
    uint16_t *src = _draw_buffers[draw_buf_index];
    if (!src) return;
    if (!getFramebuffer(target_fb_index)) return;

    // Create PPA task on first call
    if (!_ppa_task_handle) {
        _ppa_done_sem = xSemaphoreCreateBinary();
        xSemaphoreGive(_ppa_done_sem);
        xTaskCreatePinnedToCore(ppaTaskFunc, "ppa_rotate", 4096, this, 5, &_ppa_task_handle, 1);
    }

    // Wait for previous PPA operation to finish
    xSemaphoreTake(_ppa_done_sem, portMAX_DELAY);

    // Flush CPU cache for the source buffer before handing off to PPA
    esp_cache_msync(src, framebufferSize(), ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    // Store work parameters and wake the PPA task
    _ppa_work = { draw_buf_index, target_fb_index };
    xTaskNotifyGive(_ppa_task_handle);
}

bool chipguy_JC8012P4A1C_display::waitVsync(uint32_t timeout_ms)
{
    return chipguy_lcd_dpi_panel_wait_vsync(timeout_ms);
}

uint8_t chipguy_JC8012P4A1C_display::getNumFramebuffers()
{
    if (_panel_handle) {
        return chipguy_lcd_dpi_panel_get_num_fbs(_panel_handle);
    }
    return 0;
}

bool chipguy_JC8012P4A1C_display::mapTouch(uint16_t rawX, uint16_t rawY, int32_t &lx, int32_t &ly) const
{
    // The GSL3680 reports in NATIVE panel coordinates: x in [0,800), y in
    // [0,1280).  Invert the PPA rotation so the point lands where LVGL drew.
    int32_t px = rawX, py = rawY;
    int32_t x, y;
    switch (_rotation) {
        case 3:  // landscape, picture turned 90 counter-clockwise (PPA 90)
            x = (JC8012_NATIVE_HEIGHT - 1) - py;
            y = px;
            break;
        case 2:  // portrait flipped (PPA 180)
            x = (JC8012_NATIVE_WIDTH  - 1) - px;
            y = (JC8012_NATIVE_HEIGHT - 1) - py;
            break;
        case 1:  // landscape, picture turned 90 clockwise (PPA 270)
            x = py;
            y = (JC8012_NATIVE_WIDTH - 1) - px;
            break;
        default: // portrait (PPA 0)
            x = px;
            y = py;
            break;
    }
    if (x < 0 || x >= _draw_w || y < 0 || y >= _draw_h) return false;
    lx = x;
    ly = y;
    return true;
}

bool chipguy_JC8012P4A1C_display::setScanLayout(uint16_t top, uint16_t ring, uint16_t window,
                                                uint16_t offset)
{
    if (!_panel_handle) return false;
    chipguy_lcd_dpi_layout_t l = { top, ring, window, offset };
    return chipguy_lcd_dpi_panel_set_layout(_panel_handle, &l) == ESP_OK;
}

bool chipguy_JC8012P4A1C_display::setScanList(const ScanBlock *blocks, uint32_t n)
{
    static_assert(sizeof(ScanBlock) == sizeof(chipguy_lcd_dpi_block_t), "ScanBlock layout");
    if (!_panel_handle) return false;
    return chipguy_lcd_dpi_panel_set_blocks(_panel_handle,
                                            reinterpret_cast<const chipguy_lcd_dpi_block_t *>(blocks),
                                            n) == ESP_OK;
}

// --- Raster band -------------------------------------------------------------

// Jobs on the raster queue: the chunk in the low bits, its frame above.
#define RASTER_JOB_CHUNK_BITS 11
#define RASTER_JOB_CHUNK_MASK ((1u << RASTER_JOB_CHUNK_BITS) - 1)

// How many chunks ahead of the one just read the next job is.  With 2 buffers
// the one just freed is refilled (chunk + 2).  With N >= 3 it is chunk + N - 1:
// that buffer was freed a chunk earlier, and the one just freed is left for
// the prefetch of the chunk after it.
static inline uint32_t raster_lead(int nbuf) { return nbuf > 2 ? (uint32_t)nbuf - 1 : 2; }

// From the DMA interrupt: chunk `chunk` has been read out of its buffer.
// Queue the chunk `lead` on (wrapping into the next frame) to be drawn.
IRAM_ATTR bool chipguy_JC8012P4A1C_display::_rasterDoneISR(uint32_t chunk, void *ctx)
{
    chipguy_JC8012P4A1C_display *d = (chipguy_JC8012P4A1C_display *)ctx;
    uint32_t now = esp_cpu_get_cycle_count();
    if (chunk > 0) d->_rPeriodCycles = now - d->_rLastIsrCycles;
    d->_rLastIsrCycles = now;
    d->_rRead = d->_rRead + 1;
    const uint32_t n = d->_rChunks;
    // The DMA now begins chunk number _rRead (frames counted end to end):
    // has it been drawn?  Not asked of chunk 0 of the next frame, which isn't
    // read until the frame boundary.
    if (chunk + 1 < n && (int32_t)(d->_rDoneLin - d->_rRead) <= 0) d->_rLate = d->_rLate + 1;

    uint32_t frame = d->_rFrame, t = chunk + raster_lead(d->_rNBuf);
    if (t >= n) { t -= n; frame++; }
    if (chunk == n - 1) d->_rFrame = d->_rFrame + 1;
    uint32_t job = t | (frame << RASTER_JOB_CHUNK_BITS);
    d->_rIssued = d->_rIssued + 1;
    BaseType_t woke = pdFALSE;
    if (xQueueSendFromISR(d->_rQueue, &job, &woke) != pdTRUE) {
        // The task is a whole queue behind: this chunk won't be drawn (its
        // buffer shows what it held).
        d->_rCompleted = d->_rCompleted + 1;
        d->_rDropped = d->_rDropped + 1;
    }
    return woke == pdTRUE;
}

// --- Raster prefetch: the GDMA copies part of each chunk in ahead of fn -----

// Start the copy for `chunk` of `frame` into its buffer: by the GDMA if there
// is one (and the pause core held while it runs), else by the CPU here.
void chipguy_JC8012P4A1C_display::_pfStart(uint32_t chunk, uint32_t frame)
{
    // One copy at a time.  One still in flight is for a chunk that won't be
    // drawn (its job was dropped): let it land.
    if (_pfByDma) {
        jc8012_rcopy_wait(_pfCopier, 2000);
        _pfByDma = false;
    }
    const int R = _rRows;
    uint8_t *dst = (uint8_t *)_rBuf[chunk % _rNBuf] + (size_t)_pfX0 * 2;
    const size_t len = (size_t)(_pfX1 - _pfX0) * 2;
    const size_t stride = _pfStride ? _pfStride : len;
    _pfChunk = (int32_t)chunk;
    _pfFrame = frame;
    _pfSrc = (const uint8_t *)_pfFn(_rY0 + (int)chunk * R, R, frame, _pfCtx);
    if (!_pfSrc) return;                      // nothing to copy this time
    if (_pfCopier && jc8012_rcopy_start(_pfCopier, dst, PANEL_WIDTH * 2, _pfSrc, stride, len, R)) {
        _pfByDma = true;
        if (_pfGate && _pfPauseCore >= 0) xTaskNotifyGive(_pfGate);
        return;
    }
    for (int r = 0; r < R; r++) memcpy(dst + (size_t)r * PANEL_WIDTH * 2, _pfSrc + (size_t)r * stride, len);
    _pfCpu = _pfCpu + 1;
}

// Wait for the copy started for the chunk about to be drawn, and drop what
// the cache holds of its buffer (the DMA wrote round it).  The previous fn into
// this buffer ended with a write-back, so nothing dirty is lost.
void chipguy_JC8012P4A1C_display::_pfLand()
{
    if (!_pfByDma) return;
    const uint32_t t0 = esp_cpu_get_cycle_count();
    const bool ok = jc8012_rcopy_wait(_pfCopier, 2000);
    _pfWaitUs = _pfWaitUs + (esp_cpu_get_cycle_count() - t0) / _mhz;
    _pfByDma = false;
    uint16_t *buf = _rBuf[_pfChunk % _rNBuf];
    if (ok) {
        esp_cache_msync(buf, (size_t)_rRows * PANEL_WIDTH * 2, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        return;
    }
    // Lost (the channels were reset): the CPU copies it after all.
    uint8_t *dst = (uint8_t *)buf + (size_t)_pfX0 * 2;
    const size_t len = (size_t)(_pfX1 - _pfX0) * 2;
    const size_t stride = _pfStride ? _pfStride : len;
    for (int r = 0; r < _rRows; r++) memcpy(dst + (size_t)r * PANEL_WIDTH * 2, _pfSrc + (size_t)r * stride, len);
    _pfCpu = _pfCpu + 1;
}

// Holds the pause core while a copy is in flight: notified as each starts,
// it spins until the copy's interrupt clears the flag.  Runs just under the
// raster task, so it preempts LVGL but not the raster task or its helpers.
IRAM_ATTR void chipguy_JC8012P4A1C_display::_pfGateEntry(void *arg)
{
    chipguy_JC8012P4A1C_display *d = (chipguy_JC8012P4A1C_display *)arg;
    volatile int *busy = jc8012_rcopy_busy(d->_pfCopier);
    const uint32_t lim = 2000u * d->_mhz;     // 2 ms: the copy is lost, stop holding
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const uint32_t t0 = esp_cpu_get_cycle_count();
        uint32_t dt = 0;
        while (*busy && (dt = esp_cpu_get_cycle_count() - t0) < lim) {}
        d->_pfPausedUs = d->_pfPausedUs + dt / d->_mhz;
    }
}

bool chipguy_JC8012P4A1C_display::setRasterPrefetch(int x0, int x1, RasterSrcFn srcFn, void *ctx,
                                                    size_t srcStride)
{
    if (_rFn) {
        ESP_LOGE(TAG, "setRasterPrefetch: turn the band off first");
        return false;
    }
    if (!srcFn) {
        _pfFn = nullptr;
        return true;
    }
    if (x0 < 0 || x1 > PANEL_WIDTH || x0 >= x1 || (x0 % 32) || (x1 % 32)) {
        ESP_LOGE(TAG, "setRasterPrefetch: columns %d..%d must be multiples of 32 within 0..%d", x0, x1, PANEL_WIDTH);
        return false;
    }
    if (srcStride && srcStride < (size_t)(x1 - x0) * 2) {
        ESP_LOGE(TAG, "setRasterPrefetch: srcStride %u is shorter than a row's %d bytes",
                 (unsigned)srcStride, (x1 - x0) * 2);
        return false;
    }
    _pfX0 = x0;
    _pfX1 = x1;
    _pfStride = srcStride;
    _pfCtx = ctx;
    _pfFn = srcFn;
    return true;
}

// ---------------------------------------------------------------------------

void chipguy_JC8012P4A1C_display::_rasterDraw(uint32_t chunk, uint32_t frame)
{
    const int R = _rRows;
    uint16_t *dst = _rBuf[chunk % _rNBuf];
    uint32_t t0 = esp_cpu_get_cycle_count();
    if (_pfFn) {
        // This chunk's copy was normally started while the chunk before was
        // drawn; if not (the first chunks, 2 buffers, a job dropped), now.
        if (_pfChunk != (int32_t)chunk || _pfFrame != frame) _pfStart(chunk, frame);
        _pfLand();
        // The next chunk's buffer is free with 3 or more buffers (see
        // raster_lead): its copy runs while fn draws this one.
        if (_rNBuf >= 3) {
            uint32_t c1 = chunk + 1, f1 = frame;
            if (c1 >= _rChunks) { c1 = 0; f1++; }
            _pfStart(c1, f1);
        }
    }
    _rFn(dst, _rY0 + (int)chunk * R, R, frame, _rCtx);
    esp_cache_msync(dst, (size_t)R * PANEL_WIDTH * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    uint32_t dt = esp_cpu_get_cycle_count() - t0;
    if (dt > _rMaxCycles) _rMaxCycles = dt;
    _rDrawn = _rDrawn + 1;
}

void chipguy_JC8012P4A1C_display::_rasterTaskEntry(void *arg)
{
    chipguy_JC8012P4A1C_display *d = (chipguy_JC8012P4A1C_display *)arg;
    for (;;) {
        uint32_t job;
        xQueueReceive(d->_rQueue, &job, portMAX_DELAY);
        const uint32_t chunk = job & RASTER_JOB_CHUNK_MASK, frame = job >> RASTER_JOB_CHUNK_BITS;
        if (d->_rFn) d->_rasterDraw(chunk, frame);
        d->_rDoneLin = frame * d->_rChunks + chunk + 1;
        d->_rCompleted = d->_rCompleted + 1;
    }
}

bool chipguy_JC8012P4A1C_display::setRasterRows(int y0, int y1, RasterFn fn, void *ctx, int core,
                                                int chunkRows)
{
    if (!_panel_handle) return false;
    const int R = chunkRows;

    if (!fn) {
        chipguy_lcd_dpi_raster_t off = {};
        chipguy_lcd_dpi_panel_set_raster(_panel_handle, &off);
        waitVsync(50);
        waitVsync(50);   // the band is off and its last chunk read
        _rFn = nullptr;
        _rChunks = 0;
        if (_pfByDma) jc8012_rcopy_wait(_pfCopier, 2000);
        _pfByDma = false;
        _pfChunk = -1;
        return true;
    }
    if (_rFn) {
        ESP_LOGE(TAG, "setRasterRows: turn the band off first");
        return false;
    }

    // Round outward to whole chunks, and to a multiple of the buffers.
    if (y0 < 0) y0 = 0;
    if (y1 > PANEL_HEIGHT) y1 = PANEL_HEIGHT;
    if (R < 1 || R > PANEL_HEIGHT / 2) return false;
    y0 = y0 / R * R;
    y1 = (y1 + R - 1) / R * R;
    if (y1 > PANEL_HEIGHT || y1 <= y0) return false;
    while (((y1 - y0) / R) % _rNBuf) {
        if (y1 + R <= PANEL_HEIGHT) y1 += R;
        else if (y0 >= R) y0 -= R;
        else return false;
    }
    if ((y1 - y0) / R > (int)RASTER_JOB_CHUNK_MASK) return false;

    size_t bytes = (size_t)R * PANEL_WIDTH * 2;
    if (_rBufRows != R) {
        for (int i = 0; i < 8; i++) { heap_caps_free(_rBuf[i]); _rBuf[i] = nullptr; }
        _rBufRows = R;
    }
    for (int i = 0; i < _rNBuf; i++) {
        if (!_rBuf[i]) {
            _rBuf[i] = (uint16_t *)heap_caps_aligned_calloc(128, 1, bytes,
                                                            MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            if (!_rBuf[i]) {
                ESP_LOGE(TAG, "setRasterRows: no internal RAM for a %u-byte bounce buffer", (unsigned)bytes);
                return false;
            }
        }
    }
    _mhz = getCpuFrequencyMhz();
    // 24 deep: after a stall (interrupts held off) the jobs come in a burst,
    // and a dropped one leaves a stale buffer on the glass.
    if (!_rQueue) _rQueue = xQueueCreate(24, sizeof(uint32_t));
    if (!_rTask) {
        xTaskCreatePinnedToCore(_rasterTaskEntry, "raster", 4096, this,
                                configMAX_PRIORITIES - 1, &_rTask, core);
    }

    // The prefetch's copier (its interrupt on this core) and the task that
    // holds the pause core while a copy runs.
    if (_pfFn && !_pfCopier) {
        // Its interrupt on the raster task's core, which waits for it: a core
        // that holds its interrupts off (a long cache operation) delays it.
        struct NewCall { struct jc8012_rcopy *rc; int rows; } nc = { nullptr, R };
        run_on_core(core, [](void *p) { NewCall *c = (NewCall *)p; c->rc = jc8012_rcopy_new(c->rows); }, &nc);
        _pfCopier = nc.rc;
        if (!_pfCopier) ESP_LOGW(TAG, "setRasterRows: no AXI GDMA for the prefetch; the CPU copies");
    }
    if (_pfFn && _pfCopier && _pfPauseCore >= 0 && !_pfGate) {
        if (_pfPauseCore == core) {
            ESP_LOGW(TAG, "setRasterRows: the prefetch can't pause the raster task's own core");
        } else {
            xTaskCreatePinnedToCore(_pfGateEntry, "raster_gate", 2048, this,
                                    configMAX_PRIORITIES - 3, &_pfGate, _pfPauseCore);
        }
    }
    _pfByDma = false;
    _pfChunk = -1;

    _rY0 = y0;
    _rRows = R;
    _rChunks = (y1 - y0) / R;
    _rCtx = ctx;
    _rFn = fn;
    _rFrame = 0;
    _rRead = 0;
    _rDrawn = 0;
    _rLate = 0;
    _rDropped = 0;

    // The first frame's first chunks, drawn here; the interrupts take over.
    const uint32_t lead = raster_lead(_rNBuf);
    for (uint32_t c = 0; c < lead; c++) _rasterDraw(c, 0);
    _rDoneLin = lead;
    _rIssued = _rCompleted = 0;

    chipguy_lcd_dpi_raster_t r = {};
    r.y0 = y0;
    r.chunk_rows = R;
    r.chunks = _rChunks;
    for (int i = 0; i < _rNBuf; i++) r.bufs[i] = _rBuf[i];
    r.nbufs = (uint32_t)_rNBuf;
    r.done = _rasterDoneISR;
    r.ctx = this;
    if (chipguy_lcd_dpi_panel_set_raster(_panel_handle, &r) != ESP_OK) {
        _rFn = nullptr;
        _rChunks = 0;
        return false;
    }
    ESP_LOGI(TAG, "Raster band rows %d..%d: %lu chunks of %d rows, %d buffers%s", y0, y1 - 1, _rChunks, R,
             _rNBuf, _pfFn ? ", prefetched" : "");
    return true;
}

uint32_t chipguy_JC8012P4A1C_display::rasterMerged() const
{
    return _panel_handle ? chipguy_lcd_dpi_panel_raster_merged(_panel_handle) : 0;
}

IRAM_ATTR void chipguy_JC8012P4A1C_display::setRasterDirect(int chunk, bool direct)
{
    if (_panel_handle && chunk >= 0) chipguy_lcd_dpi_panel_raster_direct(_panel_handle, (uint32_t)chunk, direct);
}

uint32_t chipguy_JC8012P4A1C_display::takeRasterMaxUs()
{
    uint32_t c = _rMaxCycles;
    _rMaxCycles = 0;
    return c / getCpuFrequencyMhz();
}

uint32_t chipguy_JC8012P4A1C_display::takeRasterCopyMaxUs() { return jc8012_rcopy_take_max_us(_pfCopier); }
uint32_t chipguy_JC8012P4A1C_display::rasterCopyWaitUs() const { return _pfWaitUs; }
uint32_t chipguy_JC8012P4A1C_display::rasterCopyPausedUs() const { return _pfPausedUs; }

uint32_t chipguy_JC8012P4A1C_display::rasterChunkPeriodUs() const
{
    return _rPeriodCycles / getCpuFrequencyMhz();
}

uint32_t chipguy_JC8012P4A1C_display::framePeriodUs() const
{
    int64_t a, b;
    chipguy_lcd_dpi_panel_get_frame_timestamps(&a, &b);
    int64_t d = a > b ? a - b : b - a;
    return (uint32_t)d;
}

uint32_t chipguy_JC8012P4A1C_display::getFrameCount()
{
    return chipguy_lcd_dpi_panel_get_frame_count();
}

uint32_t chipguy_JC8012P4A1C_display::getUnderrunCount()
{
    return chipguy_lcd_dpi_panel_get_underrun_count();
}

void chipguy_JC8012P4A1C_display::setBacklight(int percentage)
{
    _backlight_pct = percentage < 0 ? 0 : (percentage > 100 ? 100 : percentage);
    if (percentage <= 0) {
        analogWrite(LCD_LED, 0);
        return;
    }
    if (percentage > 100) percentage = 100;

    // Active-high: higher duty = brighter
    int duty = percentage * 255 / 100;
    analogWrite(LCD_LED, duty);
}

void chipguy_JC8012P4A1C_display::sleep()
{
    if (_asleep || !_io_handle) return;
    int keep = _backlight_pct;
    setBacklight(0);
    _backlight_pct = keep;           // setBacklight(0) recorded 0; wake() wants the old level

    // Let a flip in flight finish so the panel isn't mid-update.
    if (_ppa_done_sem) {
        xSemaphoreTake(_ppa_done_sem, portMAX_DELAY);
        xSemaphoreGive(_ppa_done_sem);
    }

    esp_lcd_panel_io_tx_param(_io_handle, 0x28, NULL, 0);   // Display Off
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_lcd_panel_io_tx_param(_io_handle, 0x10, NULL, 0);   // Sleep In
    vTaskDelay(pdMS_TO_TICKS(120));                          // JD9365 needs 120 ms

    // Stop the video stream at a frame boundary, so the chip can light-sleep
    // without cutting the DSI DMA off mid-transfer (which hangs the bus and
    // trips the hardware watchdog).
    esp_err_t err = chipguy_lcd_dpi_panel_pause(_panel_handle, 100);
    if (err != ESP_OK) ESP_LOGE(TAG, "sleep(): stream pause failed: %s", esp_err_to_name(err));
    _asleep = true;
}

void chipguy_JC8012P4A1C_display::wake()
{
    if (!_asleep || !_io_handle) return;
    chipguy_lcd_dpi_panel_resume(_panel_handle);           // video stream back on
    esp_lcd_panel_io_tx_param(_io_handle, 0x11, NULL, 0);   // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_lcd_panel_io_tx_param(_io_handle, 0x29, NULL, 0);   // Display On
    vTaskDelay(pdMS_TO_TICKS(20));
    _asleep = false;
    setBacklight(_backlight_pct);
}


// ── Low-power sleep ─────────────────────────────────────────────────────────
// Timings measured on the board (see the power library's README-engineering):
// after touch resume(false) the controller calibrates, and a moving finger is
// first reported ~118 ms later.  Sleep through the first part, poll the rest.
static constexpr uint32_t SWIPE_QUIET_MS  = 100;
static constexpr uint32_t SWIPE_DETECT_MS = 170;

static bool bootHeld()
{
    return gpio_get_level((gpio_num_t)JC8012P4A1C_Board::BOOT_BUTTON_PIN) == 0;
}

chipguy_JC8012P4A1C_display::WakeReason chipguy_JC8012P4A1C_display::sleepUntilSwipe(uint32_t checkMs) { return lightSleep(1, checkMs); }
chipguy_JC8012P4A1C_display::WakeReason chipguy_JC8012P4A1C_display::sleepUntilTap()                   { return lightSleep(2, 0); }
chipguy_JC8012P4A1C_display::WakeReason chipguy_JC8012P4A1C_display::sleepUntilButton()                { return lightSleep(0, 0); }

// mode: 0 = button only, 1 = swipe (duty-cycled touch), 2 = tap (touch armed)
chipguy_JC8012P4A1C_display::WakeReason chipguy_JC8012P4A1C_display::lightSleep(uint8_t mode, uint32_t checkMs)
{
    const gpio_num_t BOOT = (gpio_num_t)JC8012P4A1C_Board::BOOT_BUTTON_PIN;
    const gpio_num_t TINT = (gpio_num_t)JC8012P4A1C_Board::TOUCH_INT_PIN;
    if (!_touch_ok) mode = 0;

    // Let the finger that asked for sleep (if any) lift first, so it doesn't
    // immediately wake us or get read as the wake gesture.
    if (_touch_ok) {
        uint32_t quietSince = millis();
        while (millis() - quietSince < 300) {
            uint16_t x, y;
            if (_touch.getTouch(&x, &y)) quietSince = millis();
            delay(15);
        }
    }

    sleep();   // backlight off, Display Off + Sleep In, video stream paused

    // Keep these pins as driven during light sleep: the C6's reset (whatever
    // board.setRadioPower() chose), the touch reset/INT, the panel reset, BOOT.
    gpio_sleep_sel_dis((gpio_num_t)JC8012P4A1C_Board::C6_RESET_PIN);
    gpio_sleep_sel_dis((gpio_num_t)JC8012P4A1C_Board::TOUCH_RST_PIN);
    gpio_sleep_sel_dis(TINT);
    gpio_sleep_sel_dis((gpio_num_t)JC8012P4A1C_Board::LCD_RST_PIN);
    gpio_sleep_sel_dis(BOOT);

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    gpio_wakeup_enable(BOOT, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    WakeReason why = WAKE_BUTTON;
    if (mode == 2) {
        // Touch armed: its INT line idles high and goes low on a touch.
        int idle = gpio_get_level(TINT);
        gpio_wakeup_enable(TINT, idle ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
        esp_light_sleep_start();
        gpio_wakeup_disable(TINT);
        why = bootHeld() ? WAKE_BUTTON : WAKE_TAP;
    } else if (mode == 1) {
        for (;;) {
            _touch.suspend();                                   // touch in reset
            esp_sleep_enable_timer_wakeup((uint64_t)checkMs * 1000ULL);
            esp_light_sleep_start();
            _touch.resume(false);                               // ~80 ms, no reload
            if (bootHeld()) break;
            uint32_t t0 = millis();
            esp_sleep_enable_timer_wakeup((uint64_t)SWIPE_QUIET_MS * 1000ULL);
            esp_light_sleep_start();                            // through calibration
            if (bootHeld()) break;
            bool seen = false;
            while (millis() - t0 < SWIPE_DETECT_MS) {
                uint16_t x, y;
                if (_touch.getTouch(&x, &y)) { seen = true; break; }
                delay(5);
            }
            if (seen) { why = WAKE_SWIPE; break; }
        }
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    } else {
        if (_touch_ok) _touch.suspend();
        esp_light_sleep_start();
        if (_touch_ok) _touch.resume(false);
    }

    gpio_wakeup_disable(BOOT);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    wake();
    return why;
}

#endif // ARDUINO_ESP32P4_DEV
