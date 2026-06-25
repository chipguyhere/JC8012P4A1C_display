/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2024 chipguyhere
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 MIPI DSI DPI Panel Driver with Hardware Scrolling
 * Forked from ESP-IDF v5.5 esp_lcd_panel_dpi.c
 *
 * Supports:
 * - Hardware vertical scrolling (any row offset, wrap-around)
 * - Hardware horizontal scrolling (8-pixel granularity due to 16-byte DMA alignment)
 * - Virtual framebuffer for pre-drawing content ahead of scroll position
 */

#ifdef ARDUINO_ESP32P4_DEV

#include <sys/param.h>
#include <stdarg.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_intr_alloc.h"
#include "esp_clk_tree.h"
#include "esp_cache.h"
#include "esp_memory_utils.h"
#include "esp_private/dw_gdma.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk_tree_common.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "hal/color_hal.h"
#include "hal/mipi_dsi_hal.h"
#include "hal/mipi_dsi_ll.h"
#include "soc/mipi_dsi_periph.h"

#include "esp_lcd_panel_dpi_bb.h"

// Copy the private bus structure definition from mipi_dsi_priv.h
typedef struct esp_lcd_dsi_bus_t {
    int bus_id;
    mipi_dsi_hal_context_t hal;
    void *pm_lock;
} esp_lcd_dsi_bus_t;

static const char *TAG = "chipguy_dpi";

#define DPI_PANEL_MAX_FB_NUM 3
#define STANDARD_MODE_LLI_ITEMS 2  // 2 LLI items for wrap-around scrolling
#define DMA_ALIGNMENT_BYTES 16     // DMA requires 16-byte alignment (8 pixels in RGB565)

// Debug counters
static volatile uint32_t g_frame_count = 0;
static volatile uint32_t g_underrun_count = 0;

// Frame timestamps for scanline estimation (alternating even/odd)
// Updated at end of each frame in ISR
static volatile int64_t g_frame_timestamp_even = 0;  // Timestamp when even frames end
static volatile int64_t g_frame_timestamp_odd = 0;   // Timestamp when odd frames end

#define DSI_MEM_ALLOC_CAPS (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

typedef struct esp_lcd_dpi_panel_t esp_lcd_dpi_panel_t;

static esp_err_t dpi_panel_del(esp_lcd_panel_t *panel);
static esp_err_t dpi_panel_init(esp_lcd_panel_t *panel);
static esp_err_t dpi_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);

struct esp_lcd_dpi_panel_t {
    esp_lcd_panel_t base;
    esp_lcd_dsi_bus_handle_t bus;
    uint8_t virtual_channel;
    uint8_t cur_fb_index;
    uint8_t num_fbs;
    uint8_t *fbs[DPI_PANEL_MAX_FB_NUM];
    uint32_t h_pixels;
    uint32_t v_pixels;           // Display height (visible)
    uint32_t virtual_v_pixels;   // Framebuffer height (can be > v_pixels for scrolling)
    size_t fb_size;
    size_t bytes_per_row;
    size_t bits_per_pixel;
    lcd_color_format_t in_color_format;
    lcd_color_format_t out_color_format;
    dw_gdma_channel_handle_t dma_chan;
    dw_gdma_link_list_handle_t link_lists[DPI_PANEL_MAX_FB_NUM];
    intr_handle_t brg_intr;

    // Event callbacks
    esp_lcd_dpi_panel_color_trans_done_cb_t on_color_trans_done;
    esp_lcd_dpi_panel_refresh_done_cb_t on_refresh_done;
    void *user_ctx;

    // Scroll state - pending values applied at frame boundary to avoid tearing
    volatile int scroll_y;           // Current Y scroll offset in rows
    volatile int pending_scroll_y;
    volatile bool scroll_y_pending;

    volatile int scroll_x_pixels;    // Current X scroll offset in pixels (must be 8-pixel aligned)
    volatile int pending_scroll_x_pixels;
    volatile bool scroll_x_pending;
};

// Helper to configure LLIs for scrolling with fully circular framebuffer
// Treats the entire framebuffer as a circular buffer of bytes
IRAM_ATTR
static void configure_lli_for_scroll(esp_lcd_dpi_panel_t *dpi_panel,
                                      int lli_index,
                                      int scroll_y,
                                      int scroll_x_bytes)
{
    uint8_t fb_index = dpi_panel->cur_fb_index;
    dw_gdma_link_list_handle_t link_list = dpi_panel->link_lists[fb_index];
    uint8_t *fb_start = dpi_panel->fbs[fb_index];

    // Calculate linear byte offset into framebuffer (circular)
    size_t fb_size = dpi_panel->fb_size;
    size_t start_offset = (scroll_y * dpi_panel->bytes_per_row + scroll_x_bytes) % fb_size;

    // Total bytes to display per frame
    size_t display_bytes = dpi_panel->v_pixels * dpi_panel->bytes_per_row;

    // Bytes until wrap-around
    size_t bytes_until_wrap = fb_size - start_offset;

    dw_gdma_lli_handle_t lli = dw_gdma_link_list_get_item(link_list, lli_index);

    if (bytes_until_wrap >= display_bytes) {
        // No wrap needed - split into 2 halves for DMA efficiency
        size_t half_bytes = (display_bytes / 2) & ~15;  // Align to 16 bytes

        if (lli_index == 0) {
            dw_gdma_block_transfer_config_t cfg = {
                .src = { .addr = (uint32_t)(fb_start + start_offset),
                         .burst_mode = DW_GDMA_BURST_MODE_INCREMENT,
                         .burst_items = DW_GDMA_BURST_ITEMS_512, .burst_len = 16,
                         .width = DW_GDMA_TRANS_WIDTH_64 },
                .dst = { .addr = MIPI_DSI_BRG_MEM_BASE,
                         .burst_mode = DW_GDMA_BURST_MODE_FIXED,
                         .burst_items = DW_GDMA_BURST_ITEMS_256, .burst_len = 16,
                         .width = DW_GDMA_TRANS_WIDTH_64 },
                .size = half_bytes * 8 / 64,
            };
            dw_gdma_lli_config_transfer(lli, &cfg);
            dw_gdma_lli_set_block_markers(lli, (dw_gdma_block_markers_t){.is_valid=true, .is_last=false});
        } else {
            dw_gdma_block_transfer_config_t cfg = {
                .src = { .addr = (uint32_t)(fb_start + start_offset + half_bytes),
                         .burst_mode = DW_GDMA_BURST_MODE_INCREMENT,
                         .burst_items = DW_GDMA_BURST_ITEMS_512, .burst_len = 16,
                         .width = DW_GDMA_TRANS_WIDTH_64 },
                .dst = { .addr = MIPI_DSI_BRG_MEM_BASE,
                         .burst_mode = DW_GDMA_BURST_MODE_FIXED,
                         .burst_items = DW_GDMA_BURST_ITEMS_256, .burst_len = 16,
                         .width = DW_GDMA_TRANS_WIDTH_64 },
                .size = (display_bytes - half_bytes) * 8 / 64,
            };
            dw_gdma_lli_config_transfer(lli, &cfg);
            dw_gdma_lli_set_block_markers(lli, (dw_gdma_block_markers_t){.is_valid=true, .is_last=true});
        }
    } else {
        // Wrap needed - LLI 0 reads to end of buffer, LLI 1 wraps to start
        size_t bytes_part1 = bytes_until_wrap & ~15;  // Align to 16 bytes
        size_t bytes_part2 = display_bytes - bytes_part1;

        if (lli_index == 0) {
            dw_gdma_block_transfer_config_t cfg = {
                .src = { .addr = (uint32_t)(fb_start + start_offset),
                         .burst_mode = DW_GDMA_BURST_MODE_INCREMENT,
                         .burst_items = DW_GDMA_BURST_ITEMS_512, .burst_len = 16,
                         .width = DW_GDMA_TRANS_WIDTH_64 },
                .dst = { .addr = MIPI_DSI_BRG_MEM_BASE,
                         .burst_mode = DW_GDMA_BURST_MODE_FIXED,
                         .burst_items = DW_GDMA_BURST_ITEMS_256, .burst_len = 16,
                         .width = DW_GDMA_TRANS_WIDTH_64 },
                .size = bytes_part1 * 8 / 64,
            };
            dw_gdma_lli_config_transfer(lli, &cfg);
            dw_gdma_lli_set_block_markers(lli, (dw_gdma_block_markers_t){.is_valid=true, .is_last=false});
        } else {
            dw_gdma_block_transfer_config_t cfg = {
                .src = { .addr = (uint32_t)fb_start,  // Wrap to start of buffer
                         .burst_mode = DW_GDMA_BURST_MODE_INCREMENT,
                         .burst_items = DW_GDMA_BURST_ITEMS_512, .burst_len = 16,
                         .width = DW_GDMA_TRANS_WIDTH_64 },
                .dst = { .addr = MIPI_DSI_BRG_MEM_BASE,
                         .burst_mode = DW_GDMA_BURST_MODE_FIXED,
                         .burst_items = DW_GDMA_BURST_ITEMS_256, .burst_len = 16,
                         .width = DW_GDMA_TRANS_WIDTH_64 },
                .size = bytes_part2 * 8 / 64,
            };
            dw_gdma_lli_config_transfer(lli, &cfg);
            dw_gdma_lli_set_block_markers(lli, (dw_gdma_block_markers_t){.is_valid=true, .is_last=true});
        }
    }

    // Re-link LLI 0 -> LLI 1
    if (lli_index == 0) {
        dw_gdma_lli_handle_t lli1 = dw_gdma_link_list_get_item(link_list, 1);
        dw_gdma_lli_set_next(lli, lli1);
    }
}

// ISR callback when full DMA transfer completes (frame done)
IRAM_ATTR
static bool dma_full_trans_done_cb(dw_gdma_channel_handle_t chan, const dw_gdma_trans_done_event_data_t *event_data, void *user_data)
{
    bool yield_needed = false;
    esp_lcd_dpi_panel_t *dpi_panel = (esp_lcd_dpi_panel_t *)user_data;

    // Record timestamp for scanline estimation (alternating even/odd)
    int64_t now = esp_timer_get_time();
    if (g_frame_count & 1) {
        g_frame_timestamp_odd = now;
    } else {
        g_frame_timestamp_even = now;
    }

    g_frame_count++;

    // Apply pending scroll values at frame boundary
    if (dpi_panel->scroll_y_pending) {
        dpi_panel->scroll_y = dpi_panel->pending_scroll_y;
        dpi_panel->scroll_y_pending = false;
    }
    if (dpi_panel->scroll_x_pending) {
        dpi_panel->scroll_x_pixels = dpi_panel->pending_scroll_x_pixels;
        dpi_panel->scroll_x_pending = false;
    }

    // Convert X scroll from pixels to bytes (must be 16-byte aligned)
    int scroll_x_bytes = (dpi_panel->scroll_x_pixels * dpi_panel->bits_per_pixel / 8);

    // Configure both LLIs for next frame
    configure_lli_for_scroll(dpi_panel, 0, dpi_panel->scroll_y, scroll_x_bytes);
    configure_lli_for_scroll(dpi_panel, 1, dpi_panel->scroll_y, scroll_x_bytes);

    // Restart DMA
    uint8_t fb_index = dpi_panel->cur_fb_index;
    dw_gdma_link_list_handle_t link_list = dpi_panel->link_lists[fb_index];
    dw_gdma_channel_use_link_list(chan, link_list);
    dw_gdma_channel_enable_ctrl(chan, true);

    // Call refresh done callback
    if (dpi_panel->on_refresh_done) {
        if (dpi_panel->on_refresh_done(&dpi_panel->base, NULL, dpi_panel->user_ctx)) {
            yield_needed = true;
        }
    }

    return yield_needed;
}

// Bridge ISR handler for underrun detection
IRAM_ATTR
static void mipi_dsi_bridge_isr_handler(void *args)
{
    esp_lcd_dpi_panel_t *dpi_panel = (esp_lcd_dpi_panel_t *)args;
    mipi_dsi_hal_context_t *hal = &dpi_panel->bus->hal;

    uint32_t intr_status = mipi_dsi_brg_ll_get_interrupt_status(hal->bridge);
    mipi_dsi_brg_ll_clear_interrupt_status(hal->bridge, intr_status);

    if (intr_status & MIPI_DSI_BRG_LL_EVENT_UNDERRUN) {
        g_underrun_count++;
        if (g_underrun_count <= 5) {
            ESP_DRAM_LOGE(TAG, "underrun #%lu!", g_underrun_count);
        }
    }
}

// Create DMA resources
static esp_err_t dpi_panel_create_dma_link(esp_lcd_dpi_panel_t *dpi_panel)
{
    dw_gdma_channel_handle_t dma_chan = NULL;
    dw_gdma_link_list_handle_t link_list = NULL;

    dw_gdma_channel_alloc_config_t dma_alloc_config = {
        .src = {
            .block_transfer_type = DW_GDMA_BLOCK_TRANSFER_LIST,
            .role = DW_GDMA_ROLE_MEM,
            .handshake_type = DW_GDMA_HANDSHAKE_HW,
            .num_outstanding_requests = 5,
        },
        .dst = {
            .block_transfer_type = DW_GDMA_BLOCK_TRANSFER_LIST,
            .role = DW_GDMA_ROLE_PERIPH_DSI,
            .handshake_type = DW_GDMA_HANDSHAKE_HW,
            .num_outstanding_requests = 2,
        },
        .flow_controller = DW_GDMA_FLOW_CTRL_SELF,
        .chan_priority = 1,
    };
    ESP_RETURN_ON_ERROR(dw_gdma_new_channel(&dma_alloc_config, &dma_chan), TAG, "create DMA channel failed");
    dpi_panel->dma_chan = dma_chan;

    // Create linked list with 2 items for wrap-around scrolling
    dw_gdma_link_list_config_t link_list_config = {
        .num_items = STANDARD_MODE_LLI_ITEMS,
        .link_type = DW_GDMA_LINKED_LIST_TYPE_SINGLY,
    };
    for (int i = 0; i < dpi_panel->num_fbs; i++) {
        ESP_RETURN_ON_ERROR(dw_gdma_new_link_list(&link_list_config, &link_list), TAG, "create DMA link list failed");
        dpi_panel->link_lists[i] = link_list;
    }

    dw_gdma_event_callbacks_t dsi_dma_cbs = {
        .on_full_trans_done = dma_full_trans_done_cb,
    };
    ESP_RETURN_ON_ERROR(dw_gdma_channel_register_event_callbacks(dma_chan, &dsi_dma_cbs, dpi_panel), TAG, "register DMA callbacks failed");

    return ESP_OK;
}

esp_err_t chipguy_lcd_new_panel_dpi(esp_lcd_dsi_bus_handle_t bus, const chipguy_lcd_dpi_panel_config_t *panel_config, esp_lcd_panel_handle_t *ret_panel)
{
    ESP_LOGI(TAG, "Creating DPI panel with hardware scrolling support");

    esp_err_t ret = ESP_OK;
    esp_lcd_dpi_panel_t *dpi_panel = NULL;

    ESP_RETURN_ON_FALSE(bus && panel_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(panel_config->virtual_channel < 4, ESP_ERR_INVALID_ARG, TAG, "invalid virtual channel %d", panel_config->virtual_channel);
    ESP_RETURN_ON_FALSE(panel_config->dpi_clock_freq_mhz, ESP_ERR_INVALID_ARG, TAG, "invalid DPI clock frequency");

    // Determine bits per pixel
    size_t bits_per_pixel = 16;  // Default RGB565
    switch (panel_config->pixel_format) {
    case LCD_COLOR_PIXEL_FORMAT_RGB565:
        bits_per_pixel = 16;
        break;
    case LCD_COLOR_PIXEL_FORMAT_RGB666:
        bits_per_pixel = 18;
        break;
    case LCD_COLOR_PIXEL_FORMAT_RGB888:
        bits_per_pixel = 24;
        break;
    }

    lcd_color_format_t in_color_format = COLOR_TYPE_ID(COLOR_SPACE_RGB, panel_config->pixel_format);
    if (panel_config->in_color_format) {
        in_color_format = panel_config->in_color_format;
        color_space_pixel_format_t in_color_id = { .color_type_id = in_color_format };
        bits_per_pixel = color_hal_pixel_format_get_bit_depth(in_color_id);
    }
    lcd_color_format_t out_color_format = panel_config->out_color_format ? panel_config->out_color_format : in_color_format;

    // Virtual height for scrolling (defaults to display height if not specified)
    uint32_t display_height = panel_config->video_timing.v_size;
    uint32_t virtual_height = panel_config->virtual_v_pixels > 0 ? panel_config->virtual_v_pixels : display_height;

    size_t bytes_per_row = panel_config->video_timing.h_size * bits_per_pixel / 8;
    size_t fb_size = bytes_per_row * virtual_height;

    int bus_id = bus->bus_id;
    mipi_dsi_hal_context_t *hal = &bus->hal;

    ESP_LOGI(TAG, "Display: %lux%lu, Virtual height: %lu, FB size: %u bytes",
             panel_config->video_timing.h_size, display_height, virtual_height, fb_size);

    // Allocate panel structure
    dpi_panel = heap_caps_calloc(1, sizeof(esp_lcd_dpi_panel_t), DSI_MEM_ALLOC_CAPS);
    ESP_GOTO_ON_FALSE(dpi_panel, ESP_ERR_NO_MEM, err, TAG, "no memory for DPI panel");

    dpi_panel->virtual_channel = panel_config->virtual_channel;
    dpi_panel->in_color_format = in_color_format;
    dpi_panel->out_color_format = out_color_format;
    dpi_panel->bus = bus;
    dpi_panel->h_pixels = panel_config->video_timing.h_size;
    dpi_panel->v_pixels = display_height;
    dpi_panel->virtual_v_pixels = virtual_height;
    dpi_panel->bytes_per_row = bytes_per_row;
    dpi_panel->fb_size = fb_size;
    dpi_panel->bits_per_pixel = bits_per_pixel;

    // Initialize scroll state
    dpi_panel->scroll_y = 0;
    dpi_panel->pending_scroll_y = 0;
    dpi_panel->scroll_y_pending = false;
    dpi_panel->scroll_x_pixels = 0;
    dpi_panel->pending_scroll_x_pixels = 0;
    dpi_panel->scroll_x_pending = false;

    // Allocate framebuffer(s)
    dpi_panel->num_fbs = panel_config->num_fbs > 0 ? panel_config->num_fbs : 1;
    ESP_GOTO_ON_FALSE(dpi_panel->num_fbs <= DPI_PANEL_MAX_FB_NUM, ESP_ERR_INVALID_ARG, err, TAG, "too many framebuffers");

    for (int i = 0; i < dpi_panel->num_fbs; i++) {
        uint8_t *frame_buffer = heap_caps_calloc(1, fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
        ESP_GOTO_ON_FALSE(frame_buffer, ESP_ERR_NO_MEM, err, TAG, "no memory for frame buffer");
        dpi_panel->fbs[i] = frame_buffer;
        ESP_LOGI(TAG, "Framebuffer %d @ %p", i, frame_buffer);
        ESP_GOTO_ON_ERROR(esp_cache_msync(frame_buffer, fb_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED),
                          err, TAG, "cache write back failed");
    }

    ESP_GOTO_ON_ERROR(dpi_panel_create_dma_link(dpi_panel), err, TAG, "create DMA failed");

    // Clock configuration
    mipi_dsi_dpi_clock_source_t dpi_clk_src = panel_config->dpi_clk_src ? panel_config->dpi_clk_src : MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    uint32_t dpi_clk_src_freq_hz = 0;
    ESP_GOTO_ON_ERROR(esp_clk_tree_src_get_freq_hz(dpi_clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &dpi_clk_src_freq_hz),
                      err, TAG, "get clock source frequency failed");

    uint32_t dpi_div = mipi_dsi_hal_host_dpi_calculate_divider(hal, dpi_clk_src_freq_hz / 1000 / 1000, panel_config->dpi_clock_freq_mhz);
    ESP_GOTO_ON_ERROR(esp_clk_tree_enable_src((soc_module_clk_t)dpi_clk_src, true), err, TAG, "clock source enable failed");

    PERIPH_RCC_ATOMIC() {
        mipi_dsi_ll_set_dpi_clock_source(bus_id, dpi_clk_src);
        mipi_dsi_ll_set_dpi_clock_div(bus_id, dpi_div);
        mipi_dsi_ll_enable_dpi_clock(bus_id, true);
    }

    // Install interrupt service
    int isr_flags = ESP_INTR_FLAG_LOWMED;
    ESP_GOTO_ON_ERROR(esp_intr_alloc(soc_mipi_dsi_signals[bus_id].brg_irq_id, isr_flags, mipi_dsi_bridge_isr_handler,
                                     dpi_panel, &dpi_panel->brg_intr), err, TAG, "allocate DSI Bridge interrupt failed");

    // Configure MIPI DSI host
    mipi_dsi_host_ll_dpi_set_vcid(hal->host, panel_config->virtual_channel);
    mipi_dsi_host_ll_dpi_set_color_coding(hal->host, out_color_format, 0);
    mipi_dsi_host_ll_dpi_set_timing_polarity(hal->host, false, false, false, false, false);

    if (panel_config->flags.disable_lp) {
        mipi_dsi_host_ll_dpi_enable_lp_horizontal_timing(hal->host, false, false);
        mipi_dsi_host_ll_dpi_enable_lp_vertical_timing(hal->host, false, false, false, false);
        mipi_dsi_host_ll_dpi_enable_lp_command(hal->host, false);
    } else {
        mipi_dsi_host_ll_dpi_enable_lp_horizontal_timing(hal->host, true, true);
        mipi_dsi_host_ll_dpi_enable_lp_vertical_timing(hal->host, true, true, true, true);
        mipi_dsi_host_ll_dpi_enable_lp_command(hal->host, true);
    }

    mipi_dsi_host_ll_dpi_enable_frame_ack(hal->host, true);
    mipi_dsi_host_ll_dpi_set_video_burst_type(hal->host, MIPI_DSI_LL_VIDEO_BURST_WITH_SYNC_PULSES);
    mipi_dsi_host_ll_dpi_set_video_packet_pixel_num(hal->host, panel_config->video_timing.h_size);
    mipi_dsi_host_ll_dpi_set_trunks_num(hal->host, 0);
    mipi_dsi_host_ll_dpi_set_null_packet_size(hal->host, 0);

    mipi_dsi_hal_host_dpi_set_horizontal_timing(hal, panel_config->video_timing.hsync_pulse_width,
                                                panel_config->video_timing.hsync_back_porch,
                                                panel_config->video_timing.h_size,
                                                panel_config->video_timing.hsync_front_porch);
    mipi_dsi_hal_host_dpi_set_vertical_timing(hal, panel_config->video_timing.vsync_pulse_width,
                                              panel_config->video_timing.vsync_back_porch,
                                              panel_config->video_timing.v_size,
                                              panel_config->video_timing.vsync_front_porch);

    mipi_dsi_brg_ll_set_num_pixel_bits(hal->bridge, panel_config->video_timing.h_size * panel_config->video_timing.v_size * bits_per_pixel);
    mipi_dsi_brg_ll_set_underrun_discard_count(hal->bridge, panel_config->video_timing.h_size);
    mipi_dsi_brg_ll_set_input_color_format(hal->bridge, in_color_format);
    mipi_dsi_brg_ll_set_output_color_format(hal->bridge, out_color_format, 0);
    mipi_dsi_brg_ll_set_flow_controller(hal->bridge, MIPI_DSI_LL_FLOW_CONTROLLER_DMA);
    mipi_dsi_brg_ll_set_multi_block_number(hal->bridge, STANDARD_MODE_LLI_ITEMS);
    mipi_dsi_brg_ll_set_burst_len(hal->bridge, 256);
    mipi_dsi_brg_ll_set_empty_threshold(hal->bridge, 1024 - 256);
    mipi_dsi_brg_ll_enable(hal->bridge, true);
    mipi_dsi_brg_ll_update_dpi_config(hal->bridge);

    dpi_panel->base.del = dpi_panel_del;
    dpi_panel->base.init = dpi_panel_init;
    dpi_panel->base.draw_bitmap = dpi_panel_draw_bitmap;
    *ret_panel = &dpi_panel->base;

    ESP_LOGI(TAG, "DPI panel created successfully");
    return ESP_OK;

err:
    if (dpi_panel) {
        dpi_panel_del(&dpi_panel->base);
    }
    return ret;
}

static esp_err_t dpi_panel_del(esp_lcd_panel_t *panel)
{
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    esp_lcd_dsi_bus_handle_t bus = dpi_panel->bus;
    int bus_id = bus->bus_id;
    mipi_dsi_hal_context_t *hal = &bus->hal;

    PERIPH_RCC_ATOMIC() {
        mipi_dsi_ll_enable_dpi_clock(bus_id, false);
    }
    mipi_dsi_brg_ll_enable(hal->bridge, false);

    if (dpi_panel->dma_chan) {
        dw_gdma_del_channel(dpi_panel->dma_chan);
    }

    for (int i = 0; i < DPI_PANEL_MAX_FB_NUM; i++) {
        if (dpi_panel->fbs[i]) {
            free(dpi_panel->fbs[i]);
        }
        if (dpi_panel->link_lists[i]) {
            dw_gdma_del_link_list(dpi_panel->link_lists[i]);
        }
    }

    if (dpi_panel->brg_intr) {
        esp_intr_free(dpi_panel->brg_intr);
    }

    free(dpi_panel);
    return ESP_OK;
}

static esp_err_t dpi_panel_init(esp_lcd_panel_t *panel)
{
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    esp_lcd_dsi_bus_handle_t bus = dpi_panel->bus;
    mipi_dsi_hal_context_t *hal = &bus->hal;
    dw_gdma_channel_handle_t dma_chan = dpi_panel->dma_chan;

    ESP_LOGI(TAG, "Initializing DPI panel");

    // Configure LLIs for initial display (no scroll)
    size_t rows_per_lli = dpi_panel->v_pixels / STANDARD_MODE_LLI_ITEMS;
    size_t bytes_per_lli = rows_per_lli * dpi_panel->bytes_per_row;

    for (int i = 0; i < dpi_panel->num_fbs; i++) {
        dw_gdma_link_list_handle_t link_list = dpi_panel->link_lists[i];
        uint8_t *fb_base = dpi_panel->fbs[i];

        for (int lli_idx = 0; lli_idx < STANDARD_MODE_LLI_ITEMS; lli_idx++) {
            uint8_t *src_addr = fb_base + (lli_idx * bytes_per_lli);

            dw_gdma_block_transfer_config_t dma_transfer_config = {
                .src = {
                    .addr = (uint32_t)src_addr,
                    .burst_mode = DW_GDMA_BURST_MODE_INCREMENT,
                    .burst_items = DW_GDMA_BURST_ITEMS_512,
                    .burst_len = 16,
                    .width = DW_GDMA_TRANS_WIDTH_64,
                },
                .dst = {
                    .addr = MIPI_DSI_BRG_MEM_BASE,
                    .burst_mode = DW_GDMA_BURST_MODE_FIXED,
                    .burst_items = DW_GDMA_BURST_ITEMS_256,
                    .burst_len = 16,
                    .width = DW_GDMA_TRANS_WIDTH_64,
                },
                .size = bytes_per_lli * 8 / 64,
            };

            dw_gdma_lli_handle_t lli = dw_gdma_link_list_get_item(link_list, lli_idx);
            dw_gdma_lli_config_transfer(lli, &dma_transfer_config);

            if (lli_idx < STANDARD_MODE_LLI_ITEMS - 1) {
                dw_gdma_lli_handle_t next_lli = dw_gdma_link_list_get_item(link_list, lli_idx + 1);
                dw_gdma_lli_set_next(lli, next_lli);
            }

            dw_gdma_block_markers_t markers = {
                .is_valid = true,
                .is_last = (lli_idx == STANDARD_MODE_LLI_ITEMS - 1),
            };
            dw_gdma_lli_set_block_markers(lli, markers);
        }
    }

    dpi_panel->cur_fb_index = 0;
    dw_gdma_channel_use_link_list(dma_chan, dpi_panel->link_lists[0]);
    dw_gdma_channel_enable_ctrl(dma_chan, true);

    // Enable video mode
    mipi_dsi_host_ll_enable_video_mode(hal->host, true);
    mipi_dsi_host_ll_set_clock_lane_state(hal->host, MIPI_DSI_LL_CLOCK_LANE_STATE_AUTO);
    mipi_dsi_brg_ll_enable_dpi_output(hal->bridge, true);
    mipi_dsi_brg_ll_update_dpi_config(hal->bridge);
    mipi_dsi_brg_ll_enable_interrupt(hal->bridge, MIPI_DSI_BRG_LL_EVENT_UNDERRUN, true);

    ESP_LOGI(TAG, "DPI panel initialized, video mode enabled");
    return ESP_OK;
}

static esp_err_t dpi_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);

    uint8_t *frame_buffer = dpi_panel->fbs[dpi_panel->cur_fb_index];
    uint8_t *draw_buffer = (uint8_t *)color_data;
    size_t bits_per_pixel = dpi_panel->bits_per_pixel;

    x_start = MAX(x_start, 0);
    x_end = MIN(x_end, (int)dpi_panel->h_pixels);
    y_start = MAX(y_start, 0);
    y_end = MIN(y_end, (int)dpi_panel->v_pixels);

    const uint8_t *from = draw_buffer;
    uint8_t *to = frame_buffer + (y_start * dpi_panel->h_pixels + x_start) * bits_per_pixel / 8;
    uint32_t copy_bytes_per_line = (x_end - x_start) * bits_per_pixel / 8;
    uint32_t bytes_per_line = bits_per_pixel * dpi_panel->h_pixels / 8;

    for (int y = y_start; y < y_end; y++) {
        memcpy(to, from, copy_bytes_per_line);
        to += bytes_per_line;
        from += copy_bytes_per_line;
    }

    uint8_t *cache_sync_start = frame_buffer + (y_start * dpi_panel->h_pixels) * bits_per_pixel / 8;
    size_t cache_sync_size = (y_end - y_start) * dpi_panel->h_pixels * bits_per_pixel / 8;
    esp_cache_msync(cache_sync_start, cache_sync_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);

    if (dpi_panel->on_color_trans_done) {
        dpi_panel->on_color_trans_done(&dpi_panel->base, NULL, dpi_panel->user_ctx);
    }

    return ESP_OK;
}

// === Public API ===

esp_err_t chipguy_lcd_dpi_panel_register_event_callbacks(esp_lcd_panel_handle_t panel, const chipguy_lcd_dpi_panel_event_callbacks_t *cbs, void *user_ctx)
{
    ESP_RETURN_ON_FALSE(panel && cbs, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);

    dpi_panel->on_color_trans_done = cbs->on_color_trans_done;
    dpi_panel->on_refresh_done = cbs->on_refresh_done;
    dpi_panel->user_ctx = user_ctx;

    return ESP_OK;
}

esp_err_t chipguy_lcd_dpi_panel_get_frame_buffer(esp_lcd_panel_handle_t panel, uint32_t fb_num, void **fb0, ...)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);

    ESP_RETURN_ON_FALSE(fb_num && fb_num <= dpi_panel->num_fbs, ESP_ERR_INVALID_ARG, TAG, "invalid frame buffer number");

    void **fb_itor = fb0;
    va_list args;
    va_start(args, fb0);
    for (uint32_t i = 0; i < fb_num; i++) {
        if (fb_itor) {
            *fb_itor = dpi_panel->fbs[i];
            fb_itor = va_arg(args, void **);
        }
    }
    va_end(args);
    return ESP_OK;
}

void chipguy_lcd_dpi_panel_set_scroll(esp_lcd_panel_handle_t panel, int x_pixels, int y_rows)
{
    if (!panel) return;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);

    // Normalize Y to [0, virtual_v_pixels)
    int virtual_height = dpi_panel->virtual_v_pixels;
    y_rows = y_rows % virtual_height;
    if (y_rows < 0) y_rows += virtual_height;

    // X must be 8-pixel aligned (16-byte alignment for RGB565)
    // Round to nearest 8 pixels
    x_pixels = (x_pixels / 8) * 8;

    // Clamp X to valid range (no wrap-around for X)
    if (x_pixels < 0) x_pixels = 0;
    // Don't allow X scroll beyond what keeps display on-screen
    // (For now, just clamp to 0 - full X scrolling would need wider framebuffer)

    // Set pending values - ISR will apply at next frame boundary
    dpi_panel->pending_scroll_y = y_rows;
    dpi_panel->scroll_y_pending = true;

    dpi_panel->pending_scroll_x_pixels = x_pixels;
    dpi_panel->scroll_x_pending = true;
}

void chipguy_lcd_dpi_panel_set_scroll_y(esp_lcd_panel_handle_t panel, int y_rows)
{
    if (!panel) return;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);

    int virtual_height = dpi_panel->virtual_v_pixels;
    y_rows = y_rows % virtual_height;
    if (y_rows < 0) y_rows += virtual_height;

    dpi_panel->pending_scroll_y = y_rows;
    dpi_panel->scroll_y_pending = true;
}

bool chipguy_lcd_dpi_panel_is_scroll_pending(esp_lcd_panel_handle_t panel)
{
    if (!panel) return false;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    return dpi_panel->scroll_y_pending || dpi_panel->scroll_x_pending;
}

uint32_t chipguy_lcd_dpi_panel_get_frame_count(void)
{
    return g_frame_count;
}

uint32_t chipguy_lcd_dpi_panel_get_underrun_count(void)
{
    return g_underrun_count;
}

uint32_t chipguy_lcd_dpi_panel_get_virtual_height(esp_lcd_panel_handle_t panel)
{
    if (!panel) return 0;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    return dpi_panel->virtual_v_pixels;
}

uint32_t chipguy_lcd_dpi_panel_get_display_height(esp_lcd_panel_handle_t panel)
{
    if (!panel) return 0;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    return dpi_panel->v_pixels;
}

uint32_t chipguy_lcd_dpi_panel_get_display_width(esp_lcd_panel_handle_t panel)
{
    if (!panel) return 0;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    return dpi_panel->h_pixels;
}

bool chipguy_lcd_dpi_panel_wait_vsync(uint32_t timeout_ms)
{
    uint32_t start_frame = g_frame_count;
    uint32_t start_time = xTaskGetTickCount();
    uint32_t timeout_ticks = timeout_ms / portTICK_PERIOD_MS;

    // Wait for frame count to change (indicating vsync occurred)
    while (g_frame_count == start_frame) {
        if (timeout_ms > 0) {
            uint32_t elapsed = xTaskGetTickCount() - start_time;
            if (elapsed >= timeout_ticks) {
                return false;  // Timeout
            }
        }
        // Yield to other tasks briefly
        vTaskDelay(1);
    }
    return true;
}

void chipguy_lcd_dpi_panel_set_active_fb(esp_lcd_panel_handle_t panel, uint8_t fb_index, bool wait)
{
    if (!panel) return;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);

    // Validate framebuffer index
    if (fb_index >= dpi_panel->num_fbs) {
        ESP_LOGW(TAG, "Invalid framebuffer index %d (have %d)", fb_index, dpi_panel->num_fbs);
        return;
    }

    // Switch to new framebuffer
    // Note: Scroll state is managed by higher-level code
    dpi_panel->cur_fb_index = fb_index;

    if (wait) {
        // Wait for the switch to take effect at next vsync
        chipguy_lcd_dpi_panel_wait_vsync(50);
    }
}

uint8_t chipguy_lcd_dpi_panel_get_num_fbs(esp_lcd_panel_handle_t panel)
{
    if (!panel) return 0;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    return dpi_panel->num_fbs;
}

// Get frame timestamps for scanline estimation
// Returns timestamps (microseconds) of two most recent frame ends
void chipguy_lcd_dpi_panel_get_frame_timestamps(int64_t *ts_even, int64_t *ts_odd)
{
    if (ts_even) *ts_even = g_frame_timestamp_even;
    if (ts_odd) *ts_odd = g_frame_timestamp_odd;
}

// Estimate current scanline position based on frame timestamps
// Returns estimated row (0 to v_pixels-1), or -1 if estimation not possible
// Also returns confidence: how many microseconds until estimate becomes stale
int32_t chipguy_lcd_dpi_panel_estimate_scanline(esp_lcd_panel_handle_t panel, int32_t *confidence_us)
{
    if (!panel) return -1;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);

    // Get current time and frame count atomically-ish
    int64_t now = esp_timer_get_time();
    uint32_t frame = g_frame_count;
    int64_t ts_even = g_frame_timestamp_even;
    int64_t ts_odd = g_frame_timestamp_odd;

    // Determine which timestamp is most recent
    int64_t last_frame_ts, prev_frame_ts;
    if (frame & 1) {
        // Last completed frame was even (frame_count is now odd)
        last_frame_ts = ts_even;
        prev_frame_ts = ts_odd;
    } else {
        // Last completed frame was odd (frame_count is now even)
        last_frame_ts = ts_odd;
        prev_frame_ts = ts_even;
    }

    // Need at least 2 frames of history
    if (last_frame_ts == 0 || prev_frame_ts == 0) {
        return -1;
    }

    // Calculate frame period
    int64_t frame_period_us = last_frame_ts - prev_frame_ts;
    if (frame_period_us <= 0) {
        return -1;  // Invalid timestamps
    }

    // Time elapsed since last frame end
    int64_t elapsed_us = now - last_frame_ts;

    // If elapsed > frame_period, we missed a frame boundary
    if (elapsed_us < 0 || elapsed_us > frame_period_us) {
        // Estimate is stale
        if (confidence_us) *confidence_us = 0;
        return -1;
    }

    // Estimate scanline: elapsed / frame_period * v_pixels
    int32_t estimated_row = (int32_t)((elapsed_us * dpi_panel->v_pixels) / frame_period_us);

    // Clamp to valid range
    if (estimated_row >= (int32_t)dpi_panel->v_pixels) {
        estimated_row = dpi_panel->v_pixels - 1;
    }

    // Confidence = time remaining until next frame boundary
    if (confidence_us) {
        *confidence_us = (int32_t)(frame_period_us - elapsed_us);
    }

    return estimated_row;
}

#endif // ARDUINO_ESP32P4_DEV
