# The raster band: drawing in step with the screen

A **raster band** is a run of screen rows that the panel doesn't read from a
framebuffer. Your function draws them instead, a few rows at a time, into small
buffers in internal RAM (two, or up to eight), just before the panel scans them. The rest of the screen
is the framebuffer as usual, so LVGL can still own a strip of it.

What you get for the work:

- **Every panel refresh is a frame** (52 per second) without a framebuffer for the
  band, and without the PSRAM traffic of rendering into one.
- **Nothing tears.** Each chunk is drawn shortly before it is scanned, so an animation
  that draws a whole frame from one snapshot of its state is never shown half old and
  half new.
- **Little memory:** two buffers of `chunkRows` rows by default. At the default 16 rows
  they take 51 KB of internal RAM in all; at 8 rows, 26 KB.

What it costs: your function has a deadline, the time the panel takes to scan one chunk.
**RasterBouncingBalls** is the worked example: 200 anti-aliased, see-through balls at
the full refresh rate, above an LVGL control strip. **RasterLandscapeBouncingBalls** does
the same in landscape, where every chunk also carries a piece of the LVGL strip, copied in
by DMA (["Part of every chunk from memory"](#part-of-every-chunk-from-memory-the-prefetch)).

This is the developer's file. [README-engineering.md](README-engineering.md#the-raster-band)
has how it works inside and the measurements behind the numbers here.

---

## Getting started

```cpp
#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"

#define CHUNK_ROWS 16

// Draw rows y .. y+rows-1 (800 RGB565 pixels each) into dst, row y first.
static void IRAM_ATTR drawBand(uint16_t *dst, int y, int rows, uint32_t frame, void *ctx) {
    for (int r = 0; r < rows; r++)
        for (int x = 0; x < 800; x++)
            dst[r * 800 + x] = ((x + y + r + frame) & 32) ? 0xFFFF : 0x001F;   // moving stripes
}

void setup() {
    display.setScanBlocks(1280 / CHUNK_ROWS + 2);  // before the display starts
    lv_setup.begin(0);                              // portrait
    // ... build an LVGL UI in rows 1120..1279 ...
    display.setRasterRows(0, 1120, drawBand, nullptr, 0, CHUNK_ROWS);
}
```

`setRasterRows(y0, y1, fn, ctx, core, chunkRows)` hands rows `y0` to `y1 - 1` to `fn`.
It draws the first two chunks itself before returning, so the band never shows
anything you didn't draw. `setRasterRows(0, 0, nullptr)` turns the band off again.

## The function you write

```cpp
void fn(uint16_t *dst, int y, int rows, uint32_t frame, void *ctx);
```

| Argument | What it is |
|---|---|
| `dst` | `rows` × 800 RGB565 pixels to fill, row `y` first. Fill every pixel: the buffer still holds the chunk drawn two chunks ago. |
| `y` | The first panel row of this chunk. |
| `rows` | The chunk height you chose. |
| `frame` | Counts panel frames. Every chunk of one frame has the same number, so a change means "first chunk of a new frame": the place to switch to the frame's snapshot of your state. |
| `ctx` | Whatever you passed to `setRasterRows`. |

It runs in a top-priority task pinned to `core` (0 unless you say otherwise), and it
has to finish within **one chunk's scan time**:

| `chunkRows` | Buffers (internal RAM) | Time per chunk |
|---|---|---|
| 8 | 26 KB | 117 µs |
| 16 (default) | 51 KB | 234 µs |
| 24 | 77 KB | 352 µs |
| 48 | 154 KB | 704 µs |

The panel scans a row every 14.7 µs. A chunk that isn't finished in time is **late**: the
panel shows whatever that buffer held, usually the chunk from two chunks earlier, as a
band of stale pixels.

Rules that keep it on time:

- **Keep the function and its data in internal RAM.** Mark the functions `IRAM_ATTR`,
  and keep tables in ordinary globals, not PSRAM. While the flash is being written, for
  example during an OTA update, code in flash can't run.
- **Plan per chunk, not per pixel.** Before the row loop, work out which of your objects
  cross this chunk at all. The thinner the chunk, the more often that planning runs,
  so its cost grows as `chunkRows` shrinks.
- **Use both cores for heavy drawing.** RasterBouncingBalls hands the bottom half of
  each chunk to a helper task on the other core. At 8 rows, splitting the work this way
  went from late at 50 balls to on time at 200.
- **Don't rely on seeing every chunk.** If the task falls a whole queue (24 chunks) behind,
  chunks are skipped and counted in `rasterDropped()`. Don't make the last chunk of a
  frame the only place where something essential happens, such as starting the next
  frame's physics.

## Choosing the chunk height

Bigger chunks give the function more time for the same per-chunk overhead. Smaller
chunks save internal RAM. Measured with RasterBouncingBalls: the longest chunk at 200
balls, drawing on both cores, plus the cache write-back:

| `chunkRows` | 200 opaque balls | 200 see-through balls | Late |
|---|---|---|---|
| 16 | 142 of 234 µs | 215 of 234 µs | none |

Only 16 rows has been measured on this board. At 200 see-through balls it uses 92 % of
its time, so at 8 rows, where each chunk carries the same planning in half the time,
this example would probably be late at 200 balls. (On the Waveshare P4-86 panel, whose rows are slower to scan, the same
example also ran at 8 rows, but only with each chunk split across both cores; on one core
it was late from 50 balls up.) 16 rows is the default. Try 24–48 if your drawing is
heavy, or fewer if internal RAM is tight and the drawing light.

**TODO (bench):** 8, 24 and 48 rows on this board.

## What it needs

- **The panel's own rows.** The band is in the panel's native portrait orientation,
  800 pixels across, row 0 at the top, whatever rotation LVGL uses: the graphics
  hardware turns only what LVGL draws. With LVGL at rotation 0 the two line up, as in
  RasterBouncingBalls. In landscape (rotation 3) the panel's rows are the picture's
  columns, right to left, as in RasterLandscapeBouncingBalls.
- **The plain scan layout**, so not together with the text terminal.
- **A whole number of chunks per buffer.** `y0` and `y1` are rounded outward to whole
  chunks, and then a chunk at a time until the count is a multiple of the buffers (even,
  with the default two).
- **Enough scan blocks.** Each chunk is one DMA block, plus one block each for the
  framebuffer rows above and below the band. Call `display.setScanBlocks(n)` before the
  display starts. `1280 / chunkRows + 2` always fits. Each block costs 64 bytes of
  internal RAM per framebuffer, so about 21 KB at 8 rows.

## Seeing how it's doing

| Call | Returns |
|---|---|
| `display.rasterLate()` | Chunks the panel began reading before they were drawn, since `setRasterRows`. |
| `display.rasterDropped()` | Chunks never drawn because the task was a queue (24 chunks) behind. |
| `display.rasterMerged()` | Chunk interrupts that came merged with another, which the driver caught up on ([why](README-engineering.md#the-raster-band)). |
| `display.rasterChunksRead()` | How far the panel has got: chunks read since `setRasterRows`, frames end to end. |
| `display.takeRasterMaxUs()` | The longest chunk so far (drawing plus the cache write-back), then resets it. |
| `display.rasterChunkPeriodUs()` | The time each chunk has, measured. |
| `display.rasterChunksDrawn()` | Chunks drawn so far. |
| `display.framePeriodUs()` | The panel's measured frame time (about 19,243 µs). |

RasterBouncingBalls prints these once a second on `Serial0`, and runs a benchmark
across ball counts when you type `b` in the Serial Monitor.

## More time per chunk: more buffers

```cpp
display.setRasterBuffers(4);   // before setRasterRows
```

With two buffers your function gets about one chunk's time. With N buffers (up to 8) it is
called for chunk c as soon as chunk c − (N − 1) has been read, so it has N − 2 chunk times:
two at 4 buffers (~470 µs at 16 rows). The cost is internal RAM, 25.6 KB per buffer at 16
rows, and the band must be a whole number of N-chunk groups (it is rounded out for you).
With 3 or more buffers there is also always one spare: while you draw chunk c, the buffer
for chunk c + 1 is already free (`rasterBuffer(c + 1)`). The prefetch below uses that.

## Part of every chunk from memory: the prefetch

Sometimes part of every row isn't yours to draw. The usual case is an LVGL strip along one
edge of a landscape screen: the panel scans the picture's columns, so every chunk crosses
the strip. The **prefetch** copies that part of each chunk in by DMA, one chunk ahead, and
your function draws the rest:

```cpp
// Where chunk (y, rows) of panel frame `frame` comes from: `rows` rows, each
// x1 - x0 pixels, one after another (or srcStride bytes apart).
static const void* IRAM_ATTR stripSource(int y, int rows, uint32_t frame, void* ctx);

display.setRasterBuffers(4);
display.setRasterPrefetch(640, 800, stripSource, nullptr);  // panel columns 640..799
display.setRasterPrefetchPause(1);                            // hold loop()'s core during each copy
display.setRasterRows(0, 1280, drawBalls, nullptr, 0, 16);    // drawBalls fills columns 0..639
```

- **When it happens.** The copy for chunk c + 1 runs while your function draws chunk c, so
  it costs the function nothing (with 2 buffers it runs just before, in the function's time).
  Your function must not write the copied columns. It may read them.
- **Changing the source.** `srcFn` is called once per chunk, in order. A frame's chunk 0 is
  asked for while the frame before is still being drawn. So if the source only changes when
  `frame` changes, a whole frame always comes from one source and nothing tears. After `srcFn`
  has been asked for a new frame, the old source is never read again. That is when you may
  write into it.
- **The source is read by DMA.** Anything the CPU wrote into it has to be written back from
  the cache first (`esp_cache_msync`). Things the PPA or another DMA wrote are fine as they are.
- **Columns** `x0` and `x1` must be multiples of 32 (64 bytes, one cache line), so the
  CPU's part of a row and the DMA's never share one.
- **The pause.** `setRasterPrefetchPause(core)` holds that core while each copy is in flight.
  A spinning task sits just under the raster task's priority, so anything below it waits,
  normally LVGL in `loop()`. The copy then doesn't share the PSRAM with LVGL's rendering.
  Tasks that are blocked (in `delay()`, waiting on a semaphore) aren't disturbed. Your
  helper task on that core must have a higher priority than the pause, or it will wait too.
  `-1` turns the pause off, even while the band runs.
- **No AXI GDMA free?** Then the CPU copies, much more slowly, and
  `rasterCopiesByCpu()` counts it.
- **Keep the interrupts off LVGL's core.** LVGL's flush and the PPA driver write the cache
  back with their core's interrupts held off, sometimes for hundreds of microseconds. If the
  panel's interrupts are on that core, chunks come late. Call
  `display.setInterruptCore(rasterCore)` before the display starts. The copier puts its own
  interrupt on the raster core by itself. For the same reason, a helper task on LVGL's core
  can't always be woken in time. RasterLandscapeBouncingBalls lets the raster task take over
  any half-chunk the helper hasn't started.
- **Keep LVGL on casual deadlines.** LVGL is there for the controls. Animation is your
  function's job. Cap LVGL's refresh (the example uses 50 ms) so it never competes with
  the band for the PSRAM.

**RasterLandscapeBouncingBalls** is the worked example: the balls in landscape, with
LVGL on a 160-row strip along the bottom. Its `lv_setup.hpp` shows how to make the source:
LVGL renders the strip into PSRAM, and the PPA turns it into a buffer laid out the way the
panel reads it, so each chunk's piece is one block of memory.

| Call | Returns |
|---|---|
| `display.takeRasterCopyMaxUs()` | The longest copy, from start to landing, then resets it. |
| `display.rasterCopyWaitUs()` | Total time your function's task waited for a copy to land. Near 0 when the copies keep ahead. |
| `display.rasterCopyPausedUs()` | Total time the pause core was held. |
| `display.rasterCopiesByCpu()` | Copies the CPU did instead of the DMA. |

## Chunks straight from the framebuffer

`display.setRasterDirect(chunk, true)`, called from your function for that chunk, makes the
panel read that chunk from the framebuffer instead of its bounce buffer, so you don't have to
fill it. `false` turns it back. Your function is still called for the chunk every frame. The
setting outlasts the band, so clear it when you're done.

---

## Where to go next

- Back to the [README](README.md).
- [README-engineering.md](README-engineering.md#the-raster-band): the DMA list behind
  the band, why there is no limit on blocks per frame, and the measurements.
