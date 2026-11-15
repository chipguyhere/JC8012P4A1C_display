/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2024-2026 chipguyhere
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 MIPI DSI DPI Panel Driver with page flipping and hardware scrolling
 * Forked from ESP-IDF v5.5 esp_lcd_panel_dpi.c, and modified:
 * - the DMA descriptor list is rebuilt at every frame boundary, so a page flip
 *   (chipguy_lcd_dpi_panel_set_active_fb) never tears;
 * - each frame is fed as up to four regions -- a frozen top band, a window
 *   into a ring of rows (split in two where it wraps), a frozen bottom band --
 *   so text can scroll in hardware between a fixed header and footer
 *   (chipguy_lcd_dpi_panel_set_layout);
 * - the framebuffer may be taller than the panel (virtual_v_pixels), giving
 *   the ring spare rows to draw into before they scroll into view;
 * - a raster band: rows scanned chunk by chunk from 2 to 8 bounce buffers, with
 *   an interrupt as each chunk is read so it can be refilled
 *   (chipguy_lcd_dpi_panel_set_raster); which chunks are done is read from
 *   where the DMA is, so interrupts that arrive merged are caught up on; a
 *   chunk can be read straight from the framebuffer instead
 *   (chipguy_lcd_dpi_panel_raster_direct);
 * - any number of DMA blocks per frame (the bridge's multi-block counting is
 *   off; num_dma_blocks sizes the lists), rewritten only when they change;
 * - pausing the stream at a frame boundary, for light sleep
 *   (chipguy_lcd_dpi_panel_pause / _resume);
 * - frame and underrun counters, and vsync waiting.
 */

#ifdef ARDUINO_ESP32P4_DEV

#include <sys/param.h>
#include <string.h>
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
#include "hal/dw_gdma_ll.h"           // dw_gdma_link_list_item_t, to re-mark items valid
#include "soc/dw_gdma_struct.h"       // DW_GDMA, to read where the DMA is
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

// Copy the private bus structure definition from mipi_dsi_priv.h.  Only bus_id and
// hal are read; chipguy_lcd_new_panel_dpi() checks both against the HAL's register
// pointers, so a layout change in ESP-IDF fails there instead of driving garbage.
typedef struct esp_lcd_dsi_bus_t {
    int bus_id;
    mipi_dsi_hal_context_t hal;
    void *pm_lock;
} esp_lcd_dsi_bus_t;

static const char *TAG = "chipguy_dpi";

#define DPI_PANEL_MAX_FB_NUM 3
// Each frame is fed as a list of DMA blocks, one per contiguous run of rows:
// a layout's top band, window to the ring's end, window after the wrap and
// bottom band (at most four), or a raster band's chunks and the framebuffer
// above and below them.  The list's length may differ from frame to frame.
//
// That works because the DSI bridge is told multi-block transfer is OFF
// (mipi_dsi_brg_ll_set_multi_block_number(1)): it then ends a frame by pixel
// count alone and ignores where DMA blocks end.  With multi-block ON, it
// instead treats every N blocks (a 4-bit count, at most 15) as a frame, and a
// list that isn't a whole multiple of N wedges it on a cyan screen.  Measured
// 2026-11-13 on the 720x720 Waveshare P4-86 panel: 6, 16, 30 and 90 blocks per
// frame clean with counting off, and 720 (one per scanline) right but for the time it took to rebuild every
// frame -- which is why lists are now only rewritten when something changes.
//
// (The bridge can also be the flow controller, ending blocks itself.  Tried
// the same day: it only ever ended a block at the end of a frame, whatever
// blk_raw_num_total said.)
//
// n_items is the lists' capacity: 64 bytes of internal RAM per item, per
// framebuffer.
#define DPI_LLI_ITEMS_DEFAULT 8
#define DPI_RASTER_MAX_CHUNKS 1280   // chunks a raster band may have (one-row chunks on 1280 rows)
#define DPI_LLI_ITEMS_MAX     4096

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

    // Scan-out layout (see chipguy_lcd_dpi_panel_set_layout).  The pending
    // copy is applied at the next frame boundary, so a change never tears.
    chipguy_lcd_dpi_layout_t layout;
    chipguy_lcd_dpi_layout_t pending_layout;
    volatile bool layout_pending;
    portMUX_TYPE layout_lock;     // pending_layout / layout_pending, ISR vs task

    uint32_t n_items;             // list capacity: most DMA blocks a frame may use
    uint32_t cfg_gen;             // bumped whenever the layout or raster band changes
    uint32_t list_gen[DPI_PANEL_MAX_FB_NUM];   // cfg_gen each list was built from
    uint32_t list_len[DPI_PANEL_MAX_FB_NUM];   // items in use

    // Raster band (see chipguy_lcd_dpi_panel_set_raster); under layout_lock.
    chipguy_lcd_dpi_raster_t raster;
    chipguy_lcd_dpi_raster_t pending_raster;
    volatile bool raster_pending;
    uint32_t raster_seen;         // chunk interrupts so far this frame
    int      dma_ch_id;           // the DMA channel's number, to read where it is
    volatile uint32_t raster_merged;   // chunk interrupts that came merged with another
    uint32_t items_base[DPI_PANEL_MAX_FB_NUM];   // each list's first item, as the DMA addresses it
    uint8_t  scan_fb;             // the list the frame being read came from
    // Chunks read straight from the framebuffer instead of
    // their bounce buffer, a bit each (chipguy_lcd_dpi_panel_raster_direct)
    uint32_t raster_direct[(DPI_RASTER_MAX_CHUNKS + 31) / 32];

    // Custom block list (see chipguy_lcd_dpi_panel_set_blocks); under layout_lock.
    const chipguy_lcd_dpi_block_t *blocks, *pending_blocks;
    uint32_t n_blocks, pending_n_blocks;
    volatile bool blocks_pending;

    // Stream pause: set `paused` and the frame-done ISR stops re-arming the
    // DMA, then sets `stopped`.  Lets the chip light-sleep without the DMA
    // being cut off mid-frame (which hangs the bus).
    volatile bool paused;
    volatile bool stopped;
};

// The bounce buffer chunk c is read from (2 to 8 of them, in turn)
static inline const uint8_t *raster_buf(const chipguy_lcd_dpi_raster_t *r, uint32_t c)
{
    return (const uint8_t *)r->bufs[c % (r->nbufs ? r->nbufs : 2)];
}

// True when the raster band's last chunk is the frame's last block: its
// interrupt would coincide with the frame-done one, so the frame-done handler
// reports it instead.
static inline bool raster_reaches_bottom(const esp_lcd_dpi_panel_t *p, const chipguy_lcd_dpi_raster_t *r)
{
    return r->chunks && r->y0 + r->chunks * r->chunk_rows == p->v_pixels;
}

// How many blocks the raster band needs (its chunks, and the framebuffer
// above and below it).
static inline uint32_t raster_blocks(const esp_lcd_dpi_panel_t *p, const chipguy_lcd_dpi_raster_t *r)
{
    uint32_t y1 = r->y0 + r->chunks * r->chunk_rows;
    return r->chunks + (r->y0 ? 1 : 0) + (y1 < p->v_pixels ? 1 : 0);
}

// Write list item i: `bytes` bytes from src, with an interrupt when read if intr.
IRAM_ATTR
static void put_bytes(dw_gdma_link_list_handle_t list, uint32_t n_items, uint32_t i,
                      const uint8_t *src, uint32_t bytes, bool intr)
{
    dw_gdma_block_transfer_config_t cfg = {
        .src = { .addr = (uint32_t)src,
                 .burst_mode = DW_GDMA_BURST_MODE_INCREMENT,
                 .burst_items = DW_GDMA_BURST_ITEMS_512, .burst_len = 16,
                 .width = DW_GDMA_TRANS_WIDTH_64 },
        .dst = { .addr = MIPI_DSI_BRG_MEM_BASE,
                 .burst_mode = DW_GDMA_BURST_MODE_FIXED,
                 .burst_items = DW_GDMA_BURST_ITEMS_256, .burst_len = 16,
                 .width = DW_GDMA_TRANS_WIDTH_64 },
        .size = bytes / 8,
    };
    dw_gdma_lli_handle_t lli = dw_gdma_link_list_get_item(list, i);
    dw_gdma_lli_config_transfer(lli, &cfg);
    // Link to the next item.  The last item a frame uses is re-terminated by
    // configure_frame_llis(); the list's very last has no next to link to.
    if (i + 1 < n_items) dw_gdma_lli_set_next(lli, dw_gdma_link_list_get_item(list, i + 1));
    dw_gdma_lli_set_block_markers(lli, (dw_gdma_block_markers_t){
        .is_last = false, .is_valid = true, .en_trans_done_intr = intr });
}

// Write list item i: `rows` rows from src, with an interrupt when read if intr.
IRAM_ATTR
static void put_block(esp_lcd_dpi_panel_t *dpi_panel, dw_gdma_link_list_handle_t list, uint32_t i,
                      const uint8_t *src, uint32_t rows, bool intr)
{
    dw_gdma_block_transfer_config_t cfg = {
        .src = { .addr = (uint32_t)src,
                 .burst_mode = DW_GDMA_BURST_MODE_INCREMENT,
                 .burst_items = DW_GDMA_BURST_ITEMS_512, .burst_len = 16,
                 .width = DW_GDMA_TRANS_WIDTH_64 },
        .dst = { .addr = MIPI_DSI_BRG_MEM_BASE,
                 .burst_mode = DW_GDMA_BURST_MODE_FIXED,
                 .burst_items = DW_GDMA_BURST_ITEMS_256, .burst_len = 16,
                 .width = DW_GDMA_TRANS_WIDTH_64 },
        .size = rows * dpi_panel->bytes_per_row * 8 / 64,
    };
    dw_gdma_lli_handle_t lli = dw_gdma_link_list_get_item(list, i);
    dw_gdma_lli_config_transfer(lli, &cfg);
    // Link to the next item.  The last item a frame uses is re-terminated by
    // configure_frame_llis(); the list's very last has no next to link to.
    if (i + 1 < dpi_panel->n_items) dw_gdma_lli_set_next(lli, dw_gdma_link_list_get_item(list, i + 1));
    dw_gdma_lli_set_block_markers(lli, (dw_gdma_block_markers_t){
        .is_last = false, .is_valid = true, .en_trans_done_intr = intr });
}

// Build framebuffer `fb_index`'s descriptor list from the current layout or
// raster band.  Rows are 1600 bytes on this panel, so every block start and
// length is a multiple of the DMA's 8-byte (64-bit) transfer.
IRAM_ATTR
static void configure_frame_llis(esp_lcd_dpi_panel_t *dpi_panel, uint8_t fb_index)
{
    const chipguy_lcd_dpi_layout_t *l = &dpi_panel->layout;
    const chipguy_lcd_dpi_raster_t *r = &dpi_panel->raster;
    dw_gdma_link_list_handle_t list = dpi_panel->link_lists[fb_index];
    uint8_t *fb = dpi_panel->fbs[fb_index];
    size_t bpr = dpi_panel->bytes_per_row;
    uint32_t v = dpi_panel->v_pixels;
    uint32_t k = 0;

    if (dpi_panel->n_blocks) {
        // A custom block list: as given, whatever the framebuffer.
        for (uint32_t b = 0; b < dpi_panel->n_blocks; b++)
            put_bytes(list, dpi_panel->n_items, k++, (const uint8_t *)dpi_panel->blocks[b].src, dpi_panel->blocks[b].bytes, false);
    } else if (r->chunks) {
        // Raster band (the layout is plain): framebuffer, chunks, framebuffer.
        uint32_t y1 = r->y0 + r->chunks * r->chunk_rows;
        if (r->y0) put_block(dpi_panel, list, k++, fb, r->y0, false);
        for (uint32_t c = 0; c < r->chunks; c++) {
            bool last = (c == r->chunks - 1) && y1 == v;
            // A chunk marked direct is read from the framebuffer, its interrupt kept
            const uint8_t *src = (c < DPI_RASTER_MAX_CHUNKS && (dpi_panel->raster_direct[c >> 5] >> (c & 31) & 1))
                                     ? fb + (r->y0 + c * r->chunk_rows) * bpr
                                     : raster_buf(r, c);
            put_block(dpi_panel, list, k++, src, r->chunk_rows, !last);
        }
        if (y1 < v) put_block(dpi_panel, list, k++, fb + y1 * bpr, v - y1, false);
    } else {
        uint32_t bottom = v - l->top - l->window;
        if (l->top) put_block(dpi_panel, list, k++, fb, l->top, false);
        uint32_t first = l->ring - l->offset;               // window rows before the wrap
        if (first > l->window) first = l->window;
        put_block(dpi_panel, list, k++, fb + (l->top + l->offset) * bpr, first, false);
        if (l->window > first) put_block(dpi_panel, list, k++, fb + l->top * bpr, l->window - first, false);
        if (bottom) put_block(dpi_panel, list, k++, fb + (l->top + l->ring) * bpr, bottom, false);
    }

    // The last item ends the transfer: no next, and marked last.
    dw_gdma_lli_handle_t lli = dw_gdma_link_list_get_item(list, k - 1);
    lli->llp_lo.val = 0;                     // (dw_gdma_lli_set_next refuses NULL)
    lli->llp_hi.val = 0;
    lli->ctrl_hi.shadowreg_or_lli_last = 1;
    dpi_panel->list_len[fb_index] = k;
    dpi_panel->list_gen[fb_index] = dpi_panel->cfg_gen;
    if (k > 1) {    // item 0 links to item 1, an item on: item 0's address as the DMA has it
        dw_gdma_lli_handle_t i0 = dw_gdma_link_list_get_item(list, 0);
        dpi_panel->items_base[fb_index] = ((uint32_t)i0->llp_lo.loc0 << 6) - 64;
    }
}

// Make framebuffer `fb_index`'s list ready for the next frame: rebuilt if the
// layout, raster band or framebuffer changed since it was built, otherwise
// only marked valid again (the DMA may clear the mark as it consumes an item,
// and ESP-IDF's own driver re-marks every frame too).
IRAM_ATTR
static void prepare_frame_llis(esp_lcd_dpi_panel_t *dpi_panel, uint8_t fb_index)
{
    if (dpi_panel->list_gen[fb_index] != dpi_panel->cfg_gen) {
        configure_frame_llis(dpi_panel, fb_index);
        return;
    }
    dw_gdma_lli_handle_t items = dw_gdma_link_list_get_item(dpi_panel->link_lists[fb_index], 0);
    for (uint32_t i = 0; i < dpi_panel->list_len[fb_index]; i++) {
        items[i].ctrl_hi.shadowreg_or_lli_valid = 1;
    }
}

// ISR callback when a raster chunk has been read out of its bounce buffer
IRAM_ATTR
static bool dma_block_done_cb(dw_gdma_channel_handle_t chan, const dw_gdma_trans_done_event_data_t *event_data, void *user_data)
{
    esp_lcd_dpi_panel_t *dpi_panel = (esp_lcd_dpi_panel_t *)user_data;
    const chipguy_lcd_dpi_raster_t *r = &dpi_panel->raster;
    /* Which chunks are done, from where the DMA is, not from counting interrupts. when
     * this core's interrupts are held off for a while (a long cache operation, say), two chunks'
     * interrupts come as one; counted, every later chunk of the frame was reported early, and the
     * callback refilled buffers the panel was still reading (seen: whole chunks shown several chunks
     * over). the DMA's next list item is the chunk just done plus 2 (it has loaded the one after already:
     * measured, ~40,000 of 41,000 interrupts), 0 once the list's last item is loaded. every chunk up
     * to the last done is reported, and never one past it: a late interrupt finds its chunk reported
     * already and reports nothing.
     *
     * The list's last item is the band's last chunk when the band reaches the bottom (so the chunk
     * before it is done, and the last one is reported by the frame-done handler), otherwise the
     * framebuffer rows below the band (so the band's last chunk is done). */
    const uint32_t off = r->y0 ? 1 : 0;
    const int32_t max_last = (int32_t)r->chunks - (raster_reaches_bottom(dpi_panel, r) ? 2 : 1);
    const uint32_t llp = (uint32_t)dw_gdma_ll_channel_get_current_link_list_item_addr(&DW_GDMA, dpi_panel->dma_ch_id);
    int32_t last;                                  // the last chunk the DMA has finished
    if (!llp) {
        last = max_last;
    } else {
        const int32_t next = ((int32_t)llp - (int32_t)dpi_panel->items_base[dpi_panel->scan_fb]) / 64;
        last = next - 2 - (int32_t)off;
        if (next < 0 || next > (int32_t)dpi_panel->list_len[dpi_panel->scan_fb]) last = (int32_t)dpi_panel->raster_seen;   // not this list: the count
    }
    if (last > max_last) last = max_last;
    if (last > (int32_t)dpi_panel->raster_seen) dpi_panel->raster_merged += (uint32_t)(last - (int32_t)dpi_panel->raster_seen);
    bool yield = false;
    while ((int32_t)dpi_panel->raster_seen <= last) {
        uint32_t c = dpi_panel->raster_seen++;
        if (r->done && c < r->chunks && r->done(c, r->ctx)) yield = true;
    }
    return yield;
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

    // The frame just read: if its raster band ends at the bottom, its last
    // chunk had no interrupt of its own (see configure_frame_llis).
    chipguy_lcd_dpi_raster_t done_raster = dpi_panel->raster;
    const uint32_t seen_at_end = dpi_panel->raster_seen;
    dpi_panel->raster_seen = 0;

    // Apply a pending layout or raster band at the frame boundary
    portENTER_CRITICAL_ISR(&dpi_panel->layout_lock);
    if (dpi_panel->layout_pending) {
        dpi_panel->layout = dpi_panel->pending_layout;
        dpi_panel->layout_pending = false;
        dpi_panel->cfg_gen++;
    }
    if (dpi_panel->raster_pending) {
        dpi_panel->raster = dpi_panel->pending_raster;
        dpi_panel->raster_pending = false;
        dpi_panel->cfg_gen++;
    }
    if (dpi_panel->blocks_pending) {
        dpi_panel->blocks = dpi_panel->pending_blocks;
        dpi_panel->n_blocks = dpi_panel->pending_n_blocks;
        dpi_panel->blocks_pending = false;
        dpi_panel->cfg_gen++;
    }
    portEXIT_CRITICAL_ISR(&dpi_panel->layout_lock);

    // Paused: let the stream end at this frame boundary.  resume() readies
    // the list and restarts the DMA.
    if (dpi_panel->paused) {
        dpi_panel->stopped = true;
        return false;
    }

    // Ready the current framebuffer's list for the next frame and restart
    uint8_t fb_index = dpi_panel->cur_fb_index;
    dpi_panel->scan_fb = fb_index;
    prepare_frame_llis(dpi_panel, fb_index);
    dw_gdma_link_list_handle_t link_list = dpi_panel->link_lists[fb_index];
    dw_gdma_channel_use_link_list(chan, link_list);
    dw_gdma_channel_enable_ctrl(chan, true);

    if (done_raster.done && raster_reaches_bottom(dpi_panel, &done_raster)) {
        // Whatever of the frame was not reported yet (normally only its last chunk)
        for (uint32_t c = seen_at_end; c < done_raster.chunks; c++)
            if (done_raster.done(c, done_raster.ctx)) yield_needed = true;
    }

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
    dw_gdma_channel_get_id(dma_chan, &dpi_panel->dma_ch_id);

    // One list per framebuffer (configure_frame_llis)
    dw_gdma_link_list_config_t link_list_config = {
        .num_items = dpi_panel->n_items,
        .link_type = DW_GDMA_LINKED_LIST_TYPE_SINGLY,
    };
    for (int i = 0; i < dpi_panel->num_fbs; i++) {
        ESP_RETURN_ON_ERROR(dw_gdma_new_link_list(&link_list_config, &link_list), TAG, "create DMA link list failed");
        dpi_panel->link_lists[i] = link_list;
    }

    dw_gdma_event_callbacks_t dsi_dma_cbs = {
        .on_full_trans_done = dma_full_trans_done_cb,
        .on_block_trans_done = dma_block_done_cb,
    };
    ESP_RETURN_ON_ERROR(dw_gdma_channel_register_event_callbacks(dma_chan, &dsi_dma_cbs, dpi_panel), TAG, "register DMA callbacks failed");

    return ESP_OK;
}

esp_err_t chipguy_lcd_new_panel_dpi(esp_lcd_dsi_bus_handle_t bus, const chipguy_lcd_dpi_panel_config_t *panel_config, esp_lcd_panel_handle_t *ret_panel)
{
    ESP_LOGI(TAG, "Creating DPI panel (page flip + hardware scrolling)");

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

    // esp_lcd_dsi_bus_t (top of this file) is a copy of a private ESP-IDF struct, but
    // the bus object itself was built by IDF's esp_lcd_new_dsi_bus().  If IDF ever
    // reorders that struct, bus_id/hal read garbage and the panel would drive DSI
    // through bad register pointers.  Check them against what IDF's HAL init stores
    // (MIPI_DSI_LL_GET_HOST/GET_BRG of the bus id) and refuse the bus if they differ.
    ESP_RETURN_ON_FALSE(bus_id >= 0 && bus_id < MIPI_DSI_LL_NUM_BUS &&
                        hal->host == MIPI_DSI_LL_GET_HOST(bus_id) &&
                        hal->bridge == MIPI_DSI_LL_GET_BRG(bus_id),
                        ESP_ERR_INVALID_STATE, TAG,
                        "DSI bus object doesn't match this file's esp_lcd_dsi_bus_t copy "
                        "(ESP-IDF layout changed?) -- update the struct from mipi_dsi_priv.h");

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

    // Plain layout: the first v_pixels rows, top to bottom, no scrolling
    dpi_panel->layout = (chipguy_lcd_dpi_layout_t){ 0, display_height, display_height, 0 };
    dpi_panel->layout_pending = false;
    portMUX_INITIALIZE(&dpi_panel->layout_lock);

    dpi_panel->n_items = panel_config->num_dma_blocks ? panel_config->num_dma_blocks : DPI_LLI_ITEMS_DEFAULT;
    ESP_GOTO_ON_FALSE(dpi_panel->n_items >= 4 && dpi_panel->n_items <= DPI_LLI_ITEMS_MAX,
                      ESP_ERR_INVALID_ARG, err, TAG, "num_dma_blocks must be 4..4096");

    // Allocate framebuffer(s)
    dpi_panel->num_fbs = panel_config->num_fbs > 0 ? panel_config->num_fbs : 1;
    ESP_GOTO_ON_FALSE(dpi_panel->num_fbs <= DPI_PANEL_MAX_FB_NUM, ESP_ERR_INVALID_ARG, err, TAG, "too many framebuffers");

    for (int i = 0; i < dpi_panel->num_fbs; i++) {
        // On the P4's 128-byte cache line, so the PPA may write into it and a range of it
        // may be written back or invalidated without ESP_CACHE_MSYNC_FLAG_UNALIGNED
        uint8_t *frame_buffer = heap_caps_aligned_calloc(128, 1, fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
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
    // Multi-block OFF: the bridge frames by pixel count and ignores block ends,
    // so a frame may be any number of blocks (see DPI_LLI_ITEMS_DEFAULT).
    mipi_dsi_brg_ll_set_multi_block_number(hal->bridge, 1);
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

    // Descriptor lists for the initial (plain) layout
    for (int i = 0; i < dpi_panel->num_fbs; i++) {
        configure_frame_llis(dpi_panel, i);
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

esp_err_t chipguy_lcd_dpi_panel_set_layout(esp_lcd_panel_handle_t panel,
                                           const chipguy_lcd_dpi_layout_t *layout)
{
    ESP_RETURN_ON_FALSE(panel && layout, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    uint32_t v = dpi_panel->v_pixels;
    ESP_RETURN_ON_FALSE(layout->window >= 1 && layout->top + layout->window <= v &&
                        layout->ring >= layout->window && layout->offset < layout->ring &&
                        layout->top + layout->ring + (v - layout->top - layout->window)
                            <= dpi_panel->virtual_v_pixels,
                        ESP_ERR_INVALID_ARG, TAG, "layout does not fit the framebuffer");
    chipguy_lcd_dpi_layout_t plain = { 0, v, v, 0 };
    bool is_plain = memcmp(layout, &plain, sizeof plain) == 0;
    portENTER_CRITICAL(&dpi_panel->layout_lock);
    bool raster_on = dpi_panel->raster_pending ? dpi_panel->pending_raster.chunks
                                               : dpi_panel->raster.chunks;
    if (is_plain || !raster_on) {
        dpi_panel->pending_layout = *layout;
        dpi_panel->layout_pending = true;
    }
    portEXIT_CRITICAL(&dpi_panel->layout_lock);
    ESP_RETURN_ON_FALSE(is_plain || !raster_on, ESP_ERR_INVALID_STATE, TAG,
                        "turn the raster band off before changing the layout");
    return ESP_OK;
}

esp_err_t chipguy_lcd_dpi_panel_set_raster(esp_lcd_panel_handle_t panel,
                                           const chipguy_lcd_dpi_raster_t *raster)
{
    ESP_RETURN_ON_FALSE(panel && raster, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    uint32_t v = dpi_panel->v_pixels;
    const chipguy_lcd_dpi_raster_t *r = raster;
    if (r->chunks) {
        uint32_t y1 = r->y0 + r->chunks * r->chunk_rows;
        uint32_t pieces = raster_blocks(dpi_panel, r);
        const uint32_t nb = r->nbufs ? r->nbufs : 2;
        ESP_RETURN_ON_FALSE(r->chunk_rows && nb >= 2 && nb <= 8 && r->chunks % nb == 0 && y1 <= v, ESP_ERR_INVALID_ARG,
                            TAG, "raster band must be a multiple of its 2 to 8 buffers' chunks, on the panel");
        for (uint32_t i = 2; i < nb; i++)
            ESP_RETURN_ON_FALSE(r->bufs[i] && ((uintptr_t)r->bufs[i] & 7) == 0, ESP_ERR_INVALID_ARG, TAG,
                                "raster buffers must be 8-byte aligned");
        ESP_RETURN_ON_FALSE(pieces <= dpi_panel->n_items, ESP_ERR_INVALID_ARG, TAG,
                            "raster band needs %lu DMA blocks, the lists hold %lu (num_dma_blocks)",
                            pieces, dpi_panel->n_items);
        ESP_RETURN_ON_FALSE(r->bufs[0] && r->bufs[1] && ((uintptr_t)r->bufs[0] & 7) == 0 &&
                            ((uintptr_t)r->bufs[1] & 7) == 0, ESP_ERR_INVALID_ARG, TAG,
                            "raster buffers must be 8-byte aligned");
    }
    chipguy_lcd_dpi_layout_t plain = { 0, v, v, 0 };
    portENTER_CRITICAL(&dpi_panel->layout_lock);
    const chipguy_lcd_dpi_layout_t *l = dpi_panel->layout_pending ? &dpi_panel->pending_layout
                                                                   : &dpi_panel->layout;
    bool ok = !r->chunks || memcmp(l, &plain, sizeof plain) == 0;
    if (ok) {
        dpi_panel->pending_raster = *r;
        dpi_panel->raster_pending = true;
    }
    portEXIT_CRITICAL(&dpi_panel->layout_lock);
    ESP_RETURN_ON_FALSE(ok, ESP_ERR_INVALID_STATE, TAG, "raster band needs the plain layout");
    return ESP_OK;
}

/* Chunk `chunk` of the raster band read straight from the
 * framebuffer (direct) or from its bounce buffer again. Every framebuffer's list is patched in place
 * (only its item's source address), so it applies to the next time the DMA reaches that item: from
 * the raster callback for that chunk (called a chunk ahead of the DMA), this frame. Its done
 * interrupt still comes either way, and a chunk whose list is rebuilt keeps the setting. IRAM: it is
 * called from the raster task */
IRAM_ATTR
void chipguy_lcd_dpi_panel_raster_direct(esp_lcd_panel_handle_t panel, uint32_t chunk, bool direct)
{
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    const chipguy_lcd_dpi_raster_t *r = &dpi_panel->raster;
    /* the bit is kept even for a chunk past the band in use: at start the band is still pending (it is put
     * in at the first frame's end) while its first chunks are being filled. checked against the band in use,
     * those calls were dropped, the caller never asked again, and chunks 0-2 showed their bounce buffers'
     * leftovers for good. a pending band's lists are rebuilt from the bits when it is put in (cfg_gen) */
    if (chunk >= DPI_RASTER_MAX_CHUNKS) return;
    const uint32_t bit = 1u << (chunk & 31);
    if (!!(dpi_panel->raster_direct[chunk >> 5] & bit) == direct) return;
    if (direct) dpi_panel->raster_direct[chunk >> 5] |= bit;
    else dpi_panel->raster_direct[chunk >> 5] &= ~bit;
    if (chunk >= r->chunks) return;                    // no list has the band yet: nothing to patch
    const uint32_t k = chunk + (r->y0 ? 1 : 0);
    for (int i = 0; i < dpi_panel->num_fbs; i++) {
        if (dpi_panel->list_gen[i] != dpi_panel->cfg_gen || k >= dpi_panel->list_len[i]) continue;   // rebuilt with the bit anyway
        const uint8_t *src = direct ? dpi_panel->fbs[i] + (r->y0 + chunk * r->chunk_rows) * dpi_panel->bytes_per_row
                                    : raster_buf(r, chunk);
        dw_gdma_ll_lli_set_src_addr(dw_gdma_link_list_get_item(dpi_panel->link_lists[i], k), (uint32_t)src);
    }
}

esp_err_t chipguy_lcd_dpi_panel_set_blocks(esp_lcd_panel_handle_t panel,
                                           const chipguy_lcd_dpi_block_t *blocks, uint32_t n)
{
    ESP_RETURN_ON_FALSE(panel && (blocks || !n), ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    ESP_RETURN_ON_FALSE(n <= dpi_panel->n_items, ESP_ERR_INVALID_ARG, TAG,
                        "%lu blocks, the lists hold %lu (num_dma_blocks)", n, dpi_panel->n_items);
    uint64_t total = 0;
    for (uint32_t i = 0; i < n; i++) {
        ESP_RETURN_ON_FALSE(((uintptr_t)blocks[i].src & 7) == 0 && blocks[i].bytes &&
                            (blocks[i].bytes & 7) == 0, ESP_ERR_INVALID_ARG, TAG,
                            "block %lu: src and bytes must be multiples of 8", i);
        total += blocks[i].bytes;
    }
    ESP_RETURN_ON_FALSE(!n || total == (uint64_t)dpi_panel->v_pixels * dpi_panel->bytes_per_row,
                        ESP_ERR_INVALID_ARG, TAG, "blocks must add up to exactly one frame");
    portENTER_CRITICAL(&dpi_panel->layout_lock);
    bool raster_on = dpi_panel->raster_pending ? dpi_panel->pending_raster.chunks
                                               : dpi_panel->raster.chunks;
    if (!raster_on) {
        dpi_panel->pending_blocks = blocks;
        dpi_panel->pending_n_blocks = n;
        dpi_panel->blocks_pending = true;
    }
    portEXIT_CRITICAL(&dpi_panel->layout_lock);
    ESP_RETURN_ON_FALSE(!raster_on, ESP_ERR_INVALID_STATE, TAG, "not with a raster band");
    return ESP_OK;
}

bool chipguy_lcd_dpi_panel_is_layout_pending(esp_lcd_panel_handle_t panel)
{
    if (!panel) return false;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    return dpi_panel->layout_pending;
}

esp_err_t chipguy_lcd_dpi_panel_pause(esp_lcd_panel_handle_t panel, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    mipi_dsi_hal_context_t *hal = &dpi_panel->bus->hal;
    if (dpi_panel->paused) return ESP_OK;

    dpi_panel->stopped = false;
    dpi_panel->paused = true;
    uint32_t waited = 0;
    while (!dpi_panel->stopped && waited < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(1));
        waited++;
    }
    if (!dpi_panel->stopped) {
        dpi_panel->paused = false;
        ESP_LOGE(TAG, "pause: stream did not reach a frame boundary");
        return ESP_ERR_TIMEOUT;
    }

    // DMA is idle; take the bridge and the host out of video mode so nothing
    // expects pixels while the clocks are gated.
    mipi_dsi_brg_ll_enable_interrupt(hal->bridge, MIPI_DSI_BRG_LL_EVENT_UNDERRUN, false);
    mipi_dsi_brg_ll_enable_dpi_output(hal->bridge, false);
    mipi_dsi_brg_ll_update_dpi_config(hal->bridge);
    mipi_dsi_host_ll_enable_video_mode(hal->host, false);
    return ESP_OK;
}

esp_err_t chipguy_lcd_dpi_panel_resume(esp_lcd_panel_handle_t panel)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    mipi_dsi_hal_context_t *hal = &dpi_panel->bus->hal;
    if (!dpi_panel->paused) return ESP_OK;

    // Same order as dpi_panel_init(): arm the DMA, then video mode, then the
    // bridge's DPI output.  A layout or raster band set while paused is
    // applied here, as the frame-done handler would have.
    portENTER_CRITICAL(&dpi_panel->layout_lock);
    if (dpi_panel->layout_pending) {
        dpi_panel->layout = dpi_panel->pending_layout;
        dpi_panel->layout_pending = false;
        dpi_panel->cfg_gen++;
    }
    if (dpi_panel->raster_pending) {
        dpi_panel->raster = dpi_panel->pending_raster;
        dpi_panel->raster_pending = false;
        dpi_panel->cfg_gen++;
    }
    portEXIT_CRITICAL(&dpi_panel->layout_lock);
    dpi_panel->raster_seen = 0;
    uint8_t fb_index = dpi_panel->cur_fb_index;
    dpi_panel->scan_fb = fb_index;
    prepare_frame_llis(dpi_panel, fb_index);
    dpi_panel->paused = false;
    dpi_panel->stopped = false;
    dw_gdma_channel_use_link_list(dpi_panel->dma_chan, dpi_panel->link_lists[fb_index]);
    dw_gdma_channel_enable_ctrl(dpi_panel->dma_chan, true);

    mipi_dsi_host_ll_enable_video_mode(hal->host, true);
    mipi_dsi_brg_ll_enable_dpi_output(hal->bridge, true);
    mipi_dsi_brg_ll_update_dpi_config(hal->bridge);
    mipi_dsi_brg_ll_enable_interrupt(hal->bridge, MIPI_DSI_BRG_LL_EVENT_UNDERRUN, true);
    return ESP_OK;
}

uint32_t chipguy_lcd_dpi_panel_get_frame_count(void)
{
    return g_frame_count;
}

// Chunk interrupts that came merged with the one before (reported from where the DMA is)
uint32_t chipguy_lcd_dpi_panel_raster_merged(esp_lcd_panel_handle_t panel)
{
    if (!panel) return 0;
    esp_lcd_dpi_panel_t *dpi_panel = __containerof(panel, esp_lcd_dpi_panel_t, base);
    return dpi_panel->raster_merged;
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
