/*
 * SPDX-FileCopyrightText: 2024 chipguyhere
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 MIPI DSI DPI Panel Driver with Hardware Scrolling
 * Forked from ESP-IDF esp_lcd_panel_dpi.c
 *
 * Supports:
 * - Hardware vertical scrolling (any row offset, wrap-around)
 * - Hardware horizontal scrolling (8-pixel granularity due to 16-byte DMA alignment)
 * - Virtual framebuffer for pre-drawing content ahead of scroll position
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Event callbacks for DPI panel (chipguy custom)
 */
typedef struct {
    esp_lcd_dpi_panel_color_trans_done_cb_t on_color_trans_done;  // When color transfer done
    esp_lcd_dpi_panel_refresh_done_cb_t on_refresh_done;          // When frame refresh done
} chipguy_lcd_dpi_panel_event_callbacks_t;

/**
 * @brief Configuration for DPI panel with scrolling support (chipguy custom)
 */
typedef struct {
    uint8_t virtual_channel;
    mipi_dsi_dpi_clock_source_t dpi_clk_src;
    uint32_t dpi_clock_freq_mhz;
    lcd_color_rgb_pixel_format_t pixel_format;
    lcd_color_format_t in_color_format;
    lcd_color_format_t out_color_format;
    uint8_t num_fbs;  // Number of framebuffers
    esp_lcd_video_timing_t video_timing;
    uint32_t virtual_v_pixels;     // Virtual framebuffer height for scrolling (0 = same as display height)
    struct {
        uint32_t use_dma2d: 1;
        uint32_t disable_lp: 1;
    } flags;
} chipguy_lcd_dpi_panel_config_t;

/**
 * @brief Create a new DPI panel with hardware scrolling support
 *
 * @param bus MIPI DSI bus handle
 * @param panel_config Panel configuration
 * @param ret_panel Returned panel handle
 * @return ESP_OK on success
 */
esp_err_t chipguy_lcd_new_panel_dpi(
    esp_lcd_dsi_bus_handle_t bus,
    const chipguy_lcd_dpi_panel_config_t *panel_config,
    esp_lcd_panel_handle_t *ret_panel
);

/**
 * @brief Register event callbacks for DPI panel
 *
 * @param panel Panel handle
 * @param cbs Callback functions
 * @param user_ctx User context passed to callbacks
 * @return ESP_OK on success
 */
esp_err_t chipguy_lcd_dpi_panel_register_event_callbacks(
    esp_lcd_panel_handle_t panel,
    const chipguy_lcd_dpi_panel_event_callbacks_t *cbs,
    void *user_ctx
);

/**
 * @brief Get framebuffer address
 *
 * @param panel Panel handle
 * @param fb_num Number of framebuffers to get
 * @param fb0 Pointer to receive first framebuffer address
 * @param ... Additional framebuffer pointers
 * @return ESP_OK on success
 */
esp_err_t chipguy_lcd_dpi_panel_get_frame_buffer(
    esp_lcd_panel_handle_t panel,
    uint32_t fb_num,
    void **fb0,
    ...
);

/**
 * @brief Set scroll position (both X and Y)
 *
 * X scrolling requires 8-pixel alignment due to 16-byte DMA alignment requirement.
 * The X value will be rounded down to the nearest 8-pixel boundary.
 * Y scrolling supports any row offset with automatic wrap-around.
 *
 * @param panel Panel handle
 * @param x_pixels X scroll offset in pixels (will be aligned to 8-pixel boundary)
 * @param y_rows Y scroll offset in rows (wraps within virtual_v_pixels)
 */
void chipguy_lcd_dpi_panel_set_scroll(
    esp_lcd_panel_handle_t panel,
    int x_pixels,
    int y_rows
);

/**
 * @brief Set Y scroll offset only (vertical scrolling)
 *
 * The y_offset wraps within virtual_v_pixels automatically.
 * This is the most common scrolling operation for text displays.
 *
 * @param panel Panel handle
 * @param y_rows Y offset in rows (0 to virtual_v_pixels-1)
 */
void chipguy_lcd_dpi_panel_set_scroll_y(
    esp_lcd_panel_handle_t panel,
    int y_rows
);

/**
 * @brief Check if a scroll change is still pending
 *
 * Scroll changes are applied at frame boundaries to avoid tearing.
 * This function checks if there's a pending scroll waiting to be applied.
 *
 * @param panel Panel handle
 * @return true if there's a pending scroll waiting to be applied at next frame
 */
bool chipguy_lcd_dpi_panel_is_scroll_pending(esp_lcd_panel_handle_t panel);

/**
 * @brief Get the virtual framebuffer height
 *
 * @param panel Panel handle
 * @return Virtual height in rows (for wrap-around calculations)
 */
uint32_t chipguy_lcd_dpi_panel_get_virtual_height(esp_lcd_panel_handle_t panel);

/**
 * @brief Get the visible display height
 *
 * @param panel Panel handle
 * @return Display height in rows
 */
uint32_t chipguy_lcd_dpi_panel_get_display_height(esp_lcd_panel_handle_t panel);

/**
 * @brief Get the display width
 *
 * @param panel Panel handle
 * @return Display width in pixels
 */
uint32_t chipguy_lcd_dpi_panel_get_display_width(esp_lcd_panel_handle_t panel);

/**
 * @brief Get total frame count since initialization
 *
 * Useful for debugging and timing measurements.
 *
 * @return Number of frames completed
 */
uint32_t chipguy_lcd_dpi_panel_get_frame_count(void);

/**
 * @brief Get DMA underrun count since initialization
 *
 * Underruns indicate the DMA couldn't keep up with the display timing.
 * This usually results in visual artifacts (blue screen on this display).
 *
 * @return Number of underrun events
 */
uint32_t chipguy_lcd_dpi_panel_get_underrun_count(void);

/**
 * @brief Wait for next vsync (frame boundary)
 *
 * Blocks until the current frame completes and next frame begins.
 * Use this to synchronize scroll updates with display refresh for smooth animation.
 *
 * @param timeout_ms Maximum time to wait in milliseconds (0 = no timeout)
 * @return true if vsync occurred, false if timeout
 */
bool chipguy_lcd_dpi_panel_wait_vsync(uint32_t timeout_ms);

/**
 * @brief Set active framebuffer for display
 *
 * Switches which framebuffer is being displayed. This resets scroll to (0,0).
 * Use this for double-buffering with LVGL or similar graphics libraries.
 *
 * @param panel Panel handle
 * @param fb_index Framebuffer index (0 or 1 for double-buffering)
 * @param wait If true, wait for vsync before returning
 */
void chipguy_lcd_dpi_panel_set_active_fb(
    esp_lcd_panel_handle_t panel,
    uint8_t fb_index,
    bool wait
);

/**
 * @brief Get number of framebuffers allocated
 *
 * @param panel Panel handle
 * @return Number of framebuffers (1-3)
 */
uint8_t chipguy_lcd_dpi_panel_get_num_fbs(esp_lcd_panel_handle_t panel);

/**
 * @brief Get frame timestamps for scanline estimation
 *
 * Returns timestamps (in microseconds since boot) of the two most recent frame
 * completions. One timestamp is updated on even frames, one on odd frames.
 * Use these to estimate current scanline position for tear-free rendering.
 *
 * @param ts_even Pointer to receive even frame timestamp (may be NULL)
 * @param ts_odd Pointer to receive odd frame timestamp (may be NULL)
 */
void chipguy_lcd_dpi_panel_get_frame_timestamps(int64_t *ts_even, int64_t *ts_odd);

/**
 * @brief Estimate current scanline position
 *
 * Uses frame timestamps to estimate which row the display is currently drawing.
 * This allows "race-the-beam" techniques to update scroll registers or framebuffer
 * content after the beam has passed, avoiding the need to wait for vsync.
 *
 * @param panel Panel handle
 * @param confidence_us Optional pointer to receive confidence value (microseconds
 *                      until the estimate becomes stale, i.e. until next frame boundary)
 * @return Estimated row (0 to v_pixels-1), or -1 if estimation not possible
 */
int32_t chipguy_lcd_dpi_panel_estimate_scanline(
    esp_lcd_panel_handle_t panel,
    int32_t *confidence_us
);

#ifdef __cplusplus
}
#endif
