/*
 * jc8012_raster_copy.h - A memory-to-memory copier on the ESP32-P4's AXI GDMA,
 * for the display's raster prefetch (setRasterPrefetch in the display header).
 *
 * It copies `rows` rows of `len` bytes, each from src + r * src_stride to
 * dst + r * dst_stride, and sets a flag from its interrupt when the last byte
 * has landed.  One copy at a time.  A side that is packed (stride == len) is
 * mounted as one piece, the other as one piece per row; the receive side's
 * lists are kept per destination, so a bounce buffer's list is built once and
 * only handed back to the DMA after that.
 *
 * The same AXI GDMA set-up was measured on this board (2026-11-13/14): a
 * 25.6 KB chunk out of the PSRAM in ~150-180 us, at any 2-byte offset on
 * either side, with the burst at 128.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 */

#pragma once

#ifdef ARDUINO_ESP32P4_DEV

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct jc8012_rcopy jc8012_rcopy_t;

// A copier for up to max_rows rows a copy.  NULL if no AXI GDMA channel pair
// (or no memory) is free.  Its interrupt is taken on the calling core.
jc8012_rcopy_t *jc8012_rcopy_new(int max_rows);

// Start a copy.  False if one is still in flight, rows is out of range, or the
// lists could not be built (then nothing was started).
bool jc8012_rcopy_start(jc8012_rcopy_t *rc, void *dst, size_t dst_stride,
                        const void *src, size_t src_stride, size_t len, int rows);

// The flag that is nonzero while a copy is in flight (cleared from the
// interrupt), for a spinning reader that wants no function call.
volatile int *jc8012_rcopy_busy(jc8012_rcopy_t *rc);

// Wait for the copy in flight, if any.  False if it had not landed within
// timeout_us: the channels are then reset, and the copy is lost.
bool jc8012_rcopy_wait(jc8012_rcopy_t *rc, uint32_t timeout_us);

// Microseconds the last copy took, start to landing, and the longest since
// the last call to take_max (which resets it).  Copies lost to a reset, so far.
uint32_t jc8012_rcopy_last_us(jc8012_rcopy_t *rc);
uint32_t jc8012_rcopy_take_max_us(jc8012_rcopy_t *rc);
uint32_t jc8012_rcopy_lost(jc8012_rcopy_t *rc);

#ifdef __cplusplus
}
#endif

#endif // ARDUINO_ESP32P4_DEV
