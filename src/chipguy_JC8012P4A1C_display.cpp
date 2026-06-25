/*
 * JD9365 MIPI-DSI display driver for the JC8012P4A1C ESP32-P4 LCD.
 * See chipguy_JC8012P4A1C_display.h for the wiring and rotation overview.
 *
 * Copyright (c) 2025 chipguyhere
 * MIT License
 */

#ifdef ARDUINO_ESP32P4_DEV

#include "chipguy_JC8012P4A1C_display.h"
#include "esp_lcd_panel_dpi_bb.h"
#include "esp_lcd_jd9365.h"

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
#include "esp_heap_caps.h"

// Display hardware configuration
#define MIPI_DSI_PHY_PWR_LDO_CHAN 3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
#define MIPI_DPI_PX_FORMAT LCD_COLOR_PIXEL_FORMAT_RGB565
#define NUM_FRAMEBUFFERS 2

#define PANEL_WIDTH  JC8012_NATIVE_WIDTH
#define PANEL_HEIGHT JC8012_NATIVE_HEIGHT
#define PANEL_FB_SIZE ((size_t)PANEL_WIDTH * PANEL_HEIGHT * sizeof(uint16_t))

static const char *TAG = "JC8012P4A1C";

// PPA rotation angle for each rotation index.
static ppa_srm_rotation_angle_t ppa_angle_for(uint8_t rotation) {
    switch (rotation) {
        case 1:  return PPA_SRM_ROTATION_ANGLE_90;
        case 2:  return PPA_SRM_ROTATION_ANGLE_180;
        case 3:  return PPA_SRM_ROTATION_ANGLE_270;
        default: return PPA_SRM_ROTATION_ANGLE_0;
    }
}

chipguy_JC8012P4A1C_display::chipguy_JC8012P4A1C_display()
    : _touch(TP_I2C_SDA, TP_I2C_SCL, TP_RST, TP_INT)
{
}

// Common hardware init (DSI bus, panel, reset, vendor commands, backlight).
bool chipguy_JC8012P4A1C_display::initHardware()
{
    // Power on MIPI DSI PHY
    esp_ldo_channel_handle_t ldo_mipi_phy = nullptr;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");

    // Backlight off during init
    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << LCD_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level((gpio_num_t)LCD_LED, 0);

    // Create MIPI DSI bus
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = nullptr;
    esp_lcd_dsi_bus_config_t bus_config = JD9365_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

    // Install MIPI DSI LCD control panel IO
    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    esp_lcd_dbi_io_config_t dbi_config = JD9365_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &_io_handle));

    // Create DPI panel with double-buffering (native 800x1280)
    ESP_LOGI(TAG, "Creating panel %dx%d with %d framebuffers", PANEL_WIDTH, PANEL_HEIGHT, NUM_FRAMEBUFFERS);

    chipguy_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 60,
        .pixel_format = MIPI_DPI_PX_FORMAT,
        .in_color_format = (lcd_color_format_t)0,
        .out_color_format = (lcd_color_format_t)0,
        .num_fbs = NUM_FRAMEBUFFERS,
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
        .virtual_v_pixels = 0,
        .flags = {
            .use_dma2d = false,
            .disable_lp = false,
        },
    };

    ESP_ERROR_CHECK(chipguy_lcd_new_panel_dpi(mipi_dsi_bus, &dpi_config, &_panel_handle));

    // Perform hardware reset
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << LCD_RST,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level((gpio_num_t)LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_LOGI(TAG, "Hardware reset complete");

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
    esp_err_t ret = esp_lcd_panel_init(_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Panel init failed: %s", esp_err_to_name(ret));
        return false;
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

    // Clear both native framebuffers to black
    uint16_t *fb0 = getFramebuffer(0);
    uint16_t *fb1 = getFramebuffer(1);
    if (fb0) {
        memset(fb0, 0, PANEL_FB_SIZE);
        esp_cache_msync(fb0, PANEL_FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }
    if (fb1) {
        memset(fb1, 0, PANEL_FB_SIZE);
        esp_cache_msync(fb1, PANEL_FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }

    // Initialize touch controller
    _touch.begin();
    ESP_LOGI(TAG, "Touch controller initialized");

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
    _rotation = jc8012_normalize_rotation(rotation);

    // Logical size LVGL renders at: landscape swaps width/height.
    if (_rotation == 1 || _rotation == 3) {
        _draw_w = JC8012_NATIVE_HEIGHT;   // 1280
        _draw_h = JC8012_NATIVE_WIDTH;    // 800
    } else {
        _draw_w = JC8012_NATIVE_WIDTH;    // 800
        _draw_h = JC8012_NATIVE_HEIGHT;   // 1280
    }

    if (!initHardware()) return false;
    if (!allocateDrawBuffers()) return false;

    setBacklight(100);
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
    if (!_panel_handle || index >= NUM_FRAMEBUFFERS) {
        return nullptr;
    }

    void *fb_addrs[NUM_FRAMEBUFFERS] = {nullptr, nullptr};
    esp_err_t ret = chipguy_lcd_dpi_panel_get_frame_buffer(_panel_handle, NUM_FRAMEBUFFERS,
                                                           &fb_addrs[0], &fb_addrs[1]);
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
        case 1:  // landscape (PPA 90)
            x = (JC8012_NATIVE_HEIGHT - 1) - py;
            y = px;
            break;
        case 2:  // portrait flipped (PPA 180)
            x = (JC8012_NATIVE_WIDTH  - 1) - px;
            y = (JC8012_NATIVE_HEIGHT - 1) - py;
            break;
        case 3:  // landscape flipped (PPA 270)
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
    if (percentage <= 0) {
        analogWrite(LCD_LED, 0);
        return;
    }
    if (percentage > 100) percentage = 100;

    // Active-high: higher duty = brighter
    int duty = percentage * 255 / 100;
    analogWrite(LCD_LED, duty);
}

#endif // ARDUINO_ESP32P4_DEV
