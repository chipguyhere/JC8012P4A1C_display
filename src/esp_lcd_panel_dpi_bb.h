/*
 * SPDX-FileCopyrightText: 2024-2026 chipguyhere
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 MIPI DSI DPI Panel Driver with page flipping and hardware scrolling.
 * Forked from ESP-IDF esp_lcd_panel_dpi.c; see the .c file for what changed.
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
    uint32_t virtual_v_pixels;     // Framebuffer height in rows, >= the panel's (0 = same as the panel)
    uint16_t num_dma_blocks;       // most DMA blocks a frame may use, 4..4096 (0 = 8): 64 bytes of
                                   // internal RAM each, per framebuffer; a raster band needs one per chunk
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
 * @brief How each frame is read out of the framebuffer (rows, top to bottom)
 *
 * The panel shows, in order:
 *   1. rows [0, top) of the framebuffer -- a frozen top band;
 *   2. `window` rows taken from a ring of `ring` rows that starts at framebuffer
 *      row `top`, beginning `offset` rows into the ring and wrapping round its
 *      end -- the scrolling area;
 *   3. the remaining (v_pixels - top - window) rows from framebuffer row
 *      top + ring onward -- a frozen bottom band.
 * The plain layout is { 0, v_pixels, v_pixels, 0 }.  A ring longer than the
 * window holds rows that are not on screen, to draw into before they scroll in;
 * the framebuffer must then be taller than the panel (virtual_v_pixels).
 */
typedef struct {
    uint32_t top;
    uint32_t ring;
    uint32_t window;
    uint32_t offset;
} chipguy_lcd_dpi_layout_t;

/**
 * @brief Change the layout, effective from the next frame boundary (no tearing)
 *
 * Scrolling is a change of `offset` alone.  Applies to whichever framebuffer is
 * active.  Returns ESP_ERR_INVALID_ARG if the layout does not fit.
 */
esp_err_t chipguy_lcd_dpi_panel_set_layout(
    esp_lcd_panel_handle_t panel,
    const chipguy_lcd_dpi_layout_t *layout
);

/**
 * @brief True until the frame boundary that applies the last set_layout()
 */
bool chipguy_lcd_dpi_panel_is_layout_pending(esp_lcd_panel_handle_t panel);

/**
 * @brief Raster band: rows scanned from bounce buffers instead of the framebuffer
 *
 * Display rows [y0, y0 + chunks * chunk_rows) are read, chunk_rows at a time,
 * from bufs[0], bufs[1], ... bufs[nbufs - 1], bufs[0], ... (chunk c from
 * bufs[c % nbufs]); the other rows come from the active framebuffer as usual.
 * Each chunk is its own DMA block, so chunks plus the framebuffer pieces above
 * and below must fit in the panel's num_dma_blocks (the DMA lists' capacity;
 * there is no hardware limit).  `chunks` must be a multiple of nbufs, so
 * chunk 0 of every frame uses bufs[0].
 *
 * `done` is called from the DMA interrupt when chunk c has been read out of its
 * buffer (the panel scans it within a few rows): from then on bufs[c % nbufs]
 * may be refilled, normally with chunk c + nbufs (wrapping into the next
 * frame).  Which chunks are done is read from where the DMA is, so if this
 * core's interrupts were held off long enough for two chunks' interrupts to
 * merge, both are reported (chipguy_lcd_dpi_panel_raster_merged counts them).
 * It returns true if it woke a higher-priority task.
 *
 * Needs the plain layout.  Takes effect at the next frame boundary; fill
 * the first chunks before calling.  chunks = 0 turns the band off.
 */
typedef bool (*chipguy_lcd_dpi_raster_done_cb_t)(uint32_t chunk, void *ctx);

typedef struct {
    uint32_t y0;
    uint32_t chunk_rows;
    uint32_t chunks;
    void *bufs[8];               // each chunk_rows full rows; DMA-capable, 8-byte aligned
    uint32_t nbufs;              // 2 (0 means 2) to 8 bounce buffers, chunk c from bufs[c % nbufs];
                                 // chunks must be a multiple of it
    chipguy_lcd_dpi_raster_done_cb_t done;
    void *ctx;
} chipguy_lcd_dpi_raster_t;

/**
 * @brief EXPERIMENTAL: a frame read as an arbitrary list of blocks
 *
 * Each block is `bytes` bytes read from `src`, in order, and the blocks
 * together are exactly one frame (h_size x v_size pixels); a block may end
 * mid-row, and the next block's source can be anywhere.  Every src and bytes
 * must be a multiple of 8, and n at most num_dma_blocks.  The array is used
 * in place and must stay valid while the list is set; call this again after
 * changing it.  Applied at the next frame boundary, and the DMA list is
 * rebuilt then, so a long list costs time in that interrupt (measured on
 * the JC8012).  Overrides the framebuffer, layout and raster band until
 * called with n = 0.  Not with a raster band.
 */
typedef struct {
    const void *src;
    uint32_t bytes;
} chipguy_lcd_dpi_block_t;

esp_err_t chipguy_lcd_dpi_panel_set_blocks(esp_lcd_panel_handle_t panel,
                                           const chipguy_lcd_dpi_block_t *blocks, uint32_t n);

esp_err_t chipguy_lcd_dpi_panel_set_raster(
    esp_lcd_panel_handle_t panel,
    const chipguy_lcd_dpi_raster_t *raster
);

/**
 * @brief Read one raster chunk straight from the framebuffer
 *
 * direct = true: chunk `chunk` is read from the active framebuffer's rows, not its bounce buffer, so
 * its callback need not fill anything; false: from the bounce buffer again. Takes effect when the
 * DMA next reaches that chunk; the done callback is called for it either way. From the raster
 * callback for that chunk, it applies to the chunk about to be read.
 */
void chipguy_lcd_dpi_panel_raster_direct(esp_lcd_panel_handle_t panel, uint32_t chunk, bool direct);

/**
 * @brief Chunk interrupts that came merged with another (the chunks then reported from where
 * the DMA is: two at once, as they are done), since the panel was made
 */
uint32_t chipguy_lcd_dpi_panel_raster_merged(esp_lcd_panel_handle_t panel);

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
 * @brief Pause / resume the video stream (chipguy custom)
 *
 * pause() lets the DMA finish the current frame, then stops it and takes the
 * DSI host and bridge out of video mode -- required before light sleep, which
 * otherwise gates the clocks mid-transfer and hangs the bus.  Put the panel in
 * Sleep In first.  resume() restarts the stream from the active framebuffer.
 */
esp_err_t chipguy_lcd_dpi_panel_pause(esp_lcd_panel_handle_t panel, uint32_t timeout_ms);
esp_err_t chipguy_lcd_dpi_panel_resume(esp_lcd_panel_handle_t panel);

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
 * Switches which framebuffer is being displayed, from the next frame boundary.
 * The layout is unchanged.
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
