/*
 * jc8012_raster_copy.c - AXI GDMA memory-to-memory copier for the raster
 * prefetch.  See jc8012_raster_copy.h.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 */

#ifdef ARDUINO_ESP32P4_DEV

#include "jc8012_raster_copy.h"

#include <string.h>
#include "esp_private/gdma.h"
#include "esp_private/gdma_link.h"
#include "hal/gdma_ll.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "jc8012_rcopy";

// The receive lists kept, one per destination (a raster band has at most 8
// bounce buffers).
#define RC_RX_LISTS 8
// The most bytes one list item carries (the AXI GDMA's 12-bit length).
#define RC_ITEM_MAX 4095

typedef struct {
    gdma_link_list_handle_t list;
    void *dst;                 // what it was built for
    size_t dst_stride, len;
    int rows;
    int items;                 // items in use
} rc_rx_t;

struct jc8012_rcopy {
    gdma_channel_handle_t tx, rx;
    gdma_link_list_handle_t ltx;
    int max_rows, max_items;
    rc_rx_t rxl[RC_RX_LISTS];
    int rx_next;                       // the slot to build into next
    gdma_buffer_mount_config_t *cfg;   // max_items, for mounting
    volatile int busy;
    int64_t t_start;
    volatile uint32_t last_us, max_us;
    volatile uint32_t lost;
};

static IRAM_ATTR bool rc_eof(gdma_channel_handle_t ch, gdma_event_data_t *e, void *arg)
{
    (void)ch; (void)e;
    jc8012_rcopy_t *rc = (jc8012_rcopy_t *)arg;
    // (esp_timer, not the cycle counter: the copy is started on one core and
    // this interrupt may be taken on the other)
    uint32_t dt = (uint32_t)(esp_timer_get_time() - rc->t_start);
    rc->last_us = dt;
    if (dt > rc->max_us) rc->max_us = dt;
    rc->busy = 0;
    return false;
}

jc8012_rcopy_t *jc8012_rcopy_new(int max_rows)
{
    if (max_rows < 1) return NULL;
    jc8012_rcopy_t *rc = heap_caps_calloc(1, sizeof *rc, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!rc) return NULL;
    rc->max_rows = max_rows;
    // A packed side of max_rows rows of up to 1600 bytes is several items;
    // a strided side is one per row.
    rc->max_items = max_rows + 8;
    rc->cfg = heap_caps_calloc(rc->max_items, sizeof *rc->cfg, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!rc->cfg) goto fail;

    gdma_channel_alloc_config_t tc = { .direction = GDMA_CHANNEL_DIRECTION_TX, .flags.reserve_sibling = 1 };
    if (gdma_new_axi_channel(&tc, &rc->tx) != ESP_OK) goto fail;
    gdma_channel_alloc_config_t rcfg = { .sibling_chan = rc->tx, .direction = GDMA_CHANNEL_DIRECTION_RX };
    if (gdma_new_axi_channel(&rcfg, &rc->rx) != ESP_OK) goto fail;

    gdma_trigger_t trig = GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_M2M, 0);
    uint32_t mask = 0;
    gdma_get_free_m2m_trig_id_mask(rc->tx, &mask);
    if (!mask) goto fail;
    trig.instance_id = __builtin_ctz(mask);
    if (gdma_connect(rc->rx, trig) != ESP_OK || gdma_connect(rc->tx, trig) != ESP_OK) goto fail;

    gdma_strategy_config_t st = { .owner_check = true, .auto_update_desc = true, .eof_till_data_popped = true };
    gdma_apply_strategy(rc->tx, &st);
    gdma_apply_strategy(rc->rx, &st);
    // The burst size, measured with async memcpy: 16 -> 417 us a 25.6 KB
    // chunk, 64 -> 265, 128 -> 254.
    gdma_transfer_config_t xc = { .max_data_burst_size = 128, .access_ext_mem = true };
    if (gdma_config_transfer(rc->tx, &xc) != ESP_OK || gdma_config_transfer(rc->rx, &xc) != ESP_OK) goto fail;

    gdma_rx_event_callbacks_t cb = { .on_recv_eof = rc_eof };
    if (gdma_register_rx_event_callbacks(rc->rx, &cb, rc) != ESP_OK) goto fail;

    gdma_link_list_config_t lc = { .num_items = (uint32_t)rc->max_items,
                                   .item_alignment = GDMA_LL_AXI_DESC_ALIGNMENT,
                                   .flags = { .check_owner = 1 } };
    if (gdma_new_link_list(&lc, &rc->ltx) != ESP_OK) goto fail;
    ESP_LOGI(TAG, "AXI GDMA copier ready (%d rows a copy)", max_rows);
    return rc;

fail:
    ESP_LOGW(TAG, "no AXI GDMA copier: copies will be done by the CPU");
    if (rc->ltx) gdma_del_link_list(rc->ltx);
    if (rc->rx) { gdma_disconnect(rc->rx); gdma_del_channel(rc->rx); }
    if (rc->tx) { gdma_disconnect(rc->tx); gdma_del_channel(rc->tx); }
    heap_caps_free(rc->cfg);
    heap_caps_free(rc);
    return NULL;
}

// Fill rc->cfg for one side: one piece if packed (split into items of at
// most RC_ITEM_MAX), else a piece per row.  Returns the number of configs.
static IRAM_ATTR int rc_pieces(jc8012_rcopy_t *rc, void *base, size_t stride, size_t len, int rows, bool eof)
{
    int n = 0;
    if (stride == len) {
        size_t total = len * (size_t)rows, at = 0;
        while (at < total) {
            size_t k = total - at;
            // whole 8-byte multiples per item, so no item ends mid-word
            if (k > RC_ITEM_MAX) k = RC_ITEM_MAX & ~(size_t)7;
            rc->cfg[n++] = (gdma_buffer_mount_config_t){ .buffer = (uint8_t *)base + at, .buffer_alignment = 1,
                                                         .length = k };
            at += k;
        }
    } else {
        for (int r = 0; r < rows; r++)
            rc->cfg[n++] = (gdma_buffer_mount_config_t){ .buffer = (uint8_t *)base + (size_t)r * stride,
                                                         .buffer_alignment = 1, .length = len };
    }
    rc->cfg[n - 1].flags.mark_eof = eof;
    rc->cfg[n - 1].flags.mark_final = GDMA_FINAL_LINK_TO_NULL;
    return n;
}

// The receive list for this destination: built once, then only handed back
// to the DMA.
static IRAM_ATTR gdma_link_list_handle_t rc_rx_list(jc8012_rcopy_t *rc, void *dst, size_t dst_stride, size_t len, int rows)
{
    for (int i = 0; i < RC_RX_LISTS; i++) {
        rc_rx_t *x = &rc->rxl[i];
        if (x->list && x->dst == dst && x->dst_stride == dst_stride && x->len == len && x->rows == rows) {
            for (int k = 0; k < x->items; k++) gdma_link_set_owner(x->list, k, GDMA_LLI_OWNER_DMA);
            return x->list;
        }
    }
    rc_rx_t *x = &rc->rxl[rc->rx_next];
    rc->rx_next = (rc->rx_next + 1) % RC_RX_LISTS;
    if (!x->list) {
        gdma_link_list_config_t lc = { .num_items = (uint32_t)rc->max_items,
                                       .item_alignment = GDMA_LL_AXI_DESC_ALIGNMENT,
                                       .flags = { .check_owner = 1 } };
        if (gdma_new_link_list(&lc, &x->list) != ESP_OK) { x->list = NULL; return NULL; }
    }
    x->dst = NULL;
    int n = rc_pieces(rc, dst, dst_stride, len, rows, true);
    int end = 0;
    if (gdma_link_mount_buffers(x->list, 0, rc->cfg, n, &end) != ESP_OK) return NULL;
    x->dst = dst;
    x->dst_stride = dst_stride;
    x->len = len;
    x->rows = rows;
    x->items = end + 1;
    return x->list;
}

IRAM_ATTR bool jc8012_rcopy_start(jc8012_rcopy_t *rc, void *dst, size_t dst_stride,
                                  const void *src, size_t src_stride, size_t len, int rows)
{
    if (!rc || rc->busy || rows < 1 || rows > rc->max_rows || !len || len > 1600) return false;
    gdma_link_list_handle_t rx = rc_rx_list(rc, dst, dst_stride, len, rows);
    if (!rx) return false;
    int n = rc_pieces(rc, (void *)src, src_stride, len, rows, true);
    if (gdma_link_mount_buffers(rc->ltx, 0, rc->cfg, n, NULL) != ESP_OK) return false;
    __sync_synchronize();
    rc->busy = 1;
    rc->t_start = esp_timer_get_time();
    gdma_start(rc->rx, gdma_link_get_head_addr(rx));
    gdma_start(rc->tx, gdma_link_get_head_addr(rc->ltx));
    return true;
}

volatile int *jc8012_rcopy_busy(jc8012_rcopy_t *rc) { return &rc->busy; }

IRAM_ATTR bool jc8012_rcopy_wait(jc8012_rcopy_t *rc, uint32_t timeout_us)
{
    if (!rc || !rc->busy) return true;
    int64_t t0 = esp_timer_get_time();
    while (rc->busy) {
        if (esp_timer_get_time() - t0 > (int64_t)timeout_us) {
            gdma_reset(rc->tx);
            gdma_reset(rc->rx);
            // The receive lists may be left owned by the DMA mid-way: build them again.
            for (int i = 0; i < RC_RX_LISTS; i++) rc->rxl[i].dst = NULL;
            rc->busy = 0;
            rc->lost = rc->lost + 1;
            return false;
        }
    }
    return true;
}

uint32_t jc8012_rcopy_last_us(jc8012_rcopy_t *rc) { return rc ? rc->last_us : 0; }

uint32_t jc8012_rcopy_take_max_us(jc8012_rcopy_t *rc)
{
    if (!rc) return 0;
    uint32_t m = rc->max_us;
    rc->max_us = 0;
    return m;
}

uint32_t jc8012_rcopy_lost(jc8012_rcopy_t *rc) { return rc ? rc->lost : 0; }

#endif // ARDUINO_ESP32P4_DEV
