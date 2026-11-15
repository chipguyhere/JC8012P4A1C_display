# chipguy_JC8012P4A1C_display — engineering notes

How the display, touch and backlight are driven, and why, and how the text terminal and
the raster band work inside. The panel-side hardware facts
(FPC1 pin-out, the MP3202 circuit, the touch connector) are read out in the board
library's [schematic-description.md](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/schematic-description.md)
§1.5–1.7; the rest of the board is in the board library's
[README-engineering.md](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README-engineering.md). The
companion [480display](https://github.com/chipguyhere/JC8012P4A1C_480display/blob/main/README-engineering.md) shares
everything here except the PPA's scale factor and the canvas geometry, the DMA scan
list, the terminal and the raster band (its DPI driver is still the earlier one).

Contents: [MIPI-DSI bring-up](#mipi-dsi-bring-up) · [Panel reset](#panel-reset-and-the-retry-budget) ·
[Rotation](#rotation-the-ppa-pipeline) · [The scan list](#the-scan-list) ·
[The text terminal](#the-text-terminal) · [The raster band](#the-raster-band) ·
[Experimental: a frame from any blocks](#experimental-a-frame-from-any-blocks) ·
[Touch](#touch-gsl3680) · [Backlight](#backlight)

---

## MIPI-DSI bring-up

- **PHY power:** the P4's on-chip LDO channel 3 at **2.5 V** (`esp_ldo_acquire_channel`),
  feeding `VDD_MIPI_DPHY`. The same pin powers the CSI PHY, so a camera sketch gets it
  for free once the display is up — and must not release it.
- **Bus:** 2 data lanes (`JD9365_PANEL_BUS_DSI_2CH_CONFIG`), DBI command IO for the init
  sequence, then a DPI video-mode panel with **two framebuffers** by default
  (`chipguy_lcd_new_panel_dpi` in `esp_lcd_panel_dpi_bb.c`, a fork of ESP-IDF's DPI
  driver: page flipping at frame boundaries, the [scan list](#the-scan-list), a stream
  pause for light sleep, frame and underrun counters, and a vsync wait).
  `setFramebufferCount()` allows 1 or 3.
- **Panel:** the panel vendor's init file in Guition's package is titled
  `C-JD9365DA-H3_HKC10.1_IPS(QP101BS01-1)_800x1280_FP7721 … (2lane)` — an HKC
  QP101BS01-1 glass, JD9365DA-H3 driver, FP7721 gate driver. (The package's drawing says
  BOE; the glass supplier may vary.) The vendor table in `esp_lcd_jd9365.c` follows it.
- **Timing:** DPI clock 60 MHz; H 800 + 20 sync + 20 back + 40 front = 880; V 1280 + 4 +
  8 + 20 = 1312 — the same porches as the vendor file. Refresh = 60 MHz / (880 × 1312) ≈
  **52 Hz**.
- **Lane rate:** the driver runs each lane at **1500 Mbps**. The vendor file's
  MediaTek-style `PLL_CLOCK=416` implies about 830 Mbps; the pixel stream needs
  60 MHz × 16 bit / 2 lanes = 480 Mbps plus protocol overhead. 1500 works on the board
  (observed); the vendor figure would be the one to try if a panel batch misbehaved.
- **Pixel format:** RGB565 end to end (LVGL is forced to `LV_COLOR_FORMAT_RGB565` so it
  always matches the PPA's colour mode).
- **Init:** MADCTL 0xC0 (MY | MX) sets the panel's base scan direction, then the vendor
  table in `esp_lcd_jd9365.c`, `esp_lcd_panel_init()` to start video mode, then Sleep Out
  (120 ms) and Display On. Both framebuffers are cleared and written back from cache
  before the backlight comes on.

## Panel reset and the retry budget

The panel has **no power switch** — its logic is on the always-on 3.3 V rail and FPC1
carries only reset (GPIO27) — so a warm reboot (watchdog, `esp_restart`) never
power-cycles it. It keeps whatever state it was in when the DSI stream stopped mid-frame,
which is the known cause of a black screen after a watchdog reset. The reset line is the
only recovery lever firmware has, so:

- `LCD_RST` is **asserted before the DSI PHY is powered** and held through the whole
  host-side bring-up, with a floor of `PANEL_RESET_MIN_LOW_MS` = 50 ms, then released
  with a 200 ms settle before the first command. The panel never watches the lanes come
  alive underneath it.
- Any failed step goes through `panelInitFailed()`: backlight off, reset held 1 s, and
  `esp_restart()` — bounded at **3 consecutive attempts** by a counter in
  `RTC_NOINIT` memory (seeded by a magic value and cleared on power-on reset and on
  success), so a truly dead panel halts with a readable log instead of reboot-looping.
- A touch-controller failure is retried the same way while the budget lasts; once it's
  spent the display comes up **without touch** rather than dark.

## Rotation: the PPA pipeline

The JD9365 driver supports mirroring but not `swap_xy`, so 90°/270° can't be done in the
panel. Instead:

1. LVGL renders in **DIRECT** mode into one of two full-frame **draw buffers** sized to
   the logical orientation (800×1280 or 1280×800), in PSRAM, 128-byte aligned (the P4's
   L2 cache line — `esp_cache_msync()` requires start and size aligned to it).
2. On the last flush of a refresh, `flip()` writes the draw buffer back from cache and
   hands it to a task pinned to core 1 (priority 5).
3. That task runs `ppa_do_scale_rotate_mirror()` (blocking, scale 1.0, rotation 0/90/
   180/270) from the draw buffer straight into the **other** DSI framebuffer, invalidates
   the cache over it, and sets it active — the DPI driver switches at the next vsync.
4. The next `flip()` waits for the previous PPA job, so at most one is in flight.

**Memory:** each framebuffer and each draw buffer is 800 × 1280 × 2 = 2,048,000 bytes;
four of them is **≈ 7.8 MiB of PSRAM**, which is why PSRAM is required.

**Touch mapping:** the GSL3680 reports in native panel coordinates; `mapTouch()` applies
the inverse of the PPA rotation, so a touch lands where LVGL drew.

## The scan list

The display DMA (the DW-GDMA channel feeding the DSI bridge) reads each frame through a
linked list of blocks, one per contiguous run of source bytes. The driver keeps one list
per framebuffer and restarts the channel on the current one from the end-of-frame
interrupt, so any change to *where* a frame is read from takes effect at a frame
boundary and can't tear. A list is rewritten only when the layout, the raster band, the
custom block list or its own framebuffer changed since it was built; otherwise the
interrupt only sets each item's valid bit again, as ESP-IDF's own DPI driver does. The
list's length may change from frame to frame. `setScanBlocks()` (default 8, up to
4096) only sizes the lists: **64 bytes of internal RAM per item, per framebuffer.**

**Multi-block counting is off.** With the DMA as flow controller, the bridge has a
multi-block mode (`dma_multiblk_en`, with a 4-bit count) in which it expects that many
block ends per frame, and a list that isn't a whole multiple of the count wedges it on a
cyan screen. The driver sets the count to 1, which turns the mode off, before the first
frame; the bridge then ends each frame by pixel count and ignores where blocks end. That
was found and measured on the Waveshare P4-86 panel (`chipguy_P486Panel_display`, the
same driver), where 6, 16, 30, 90 and 720 blocks a frame all gave the right picture.
Two dead ends from the same tests: the bridge as flow controller never ends a block
mid-frame, and the DMA has no source-address wrap or mask, so a small buffer repeated
down the screen needs one item per repetition.

**Alignment.** Each block's source address and length must be multiples of the DMA's
64-bit transfer: **8 bytes, 4 RGB565 pixels**. A row is 1600 bytes, so any row boundary
works, and so does any 4-pixel column.

**A full list.** Each item used to be linked to item *i* + 1 as it was written, and the
frame's last item re-terminated afterwards. A list filled to exactly its capacity linked
its last item one past the end, and ESP-IDF's `dw_gdma_link_list_get_item` aborts on that
— from the frame-done interrupt, so the board boot-looped. Found with the 2560-block test
below (2026-11-13); the list's last item is no longer linked.

## The text terminal

`chipguy_JC8012P4A1C_terminal` ([README-terminal.md](README-terminal.md)) is the
`chipguy_ESP32-CYD-Resistive_display` terminal's front end (printing, `printf`, fonts,
API) over a back end that scrolls with the display hardware. It came here from
`chipguy_P486Panel_display`, which has the same driver. Measured on the bench unit,
2026-11-13, core 3.3.11.

**Cell size: 10 × 20.** A cell must divide every screen the terminal offers into whole
cells. Native portrait and landscape are 800 × 1280 and 1280 × 800, so a cell's width and
height must each divide gcd(800, 1280) = 160: widths 8, 10, 16, 20 …, heights 10, 16, 20,
32 …. The 10.1-inch panel is about 150 dpi (217 mm × 136 mm for 1280 × 800), against the P4-86
panel's 254, so 10 × 20 here is about the physical size of that board's 16 × 30. The
480 mode's long side was 768 (the 480display library's fullscreen canvas); 768 isn't a
multiple of 20, so the terminal's 480 mode is **480 × 760** instead, scaled 1.625×
(13/8, an exact PPA ratio) to **780 × 1235** at panel (10, 22). That gives 80 × 64,
128 × 40, 48 × 38 and 76 × 24 cells. The fonts were regenerated at 10 × 20, baseline 16
(`extras/fontgen/make_terminal_fonts.py`, which now takes `--cell-width`): Inconsolata
sized so its advance is 10 px (20 px type), symbols from DejaVu Sans Mono, LVGL's icons
from Font Awesome scaled into 10 × 12, box drawing and blocks drawn by the script. The
built-in font is 74,691 bytes of glyph data; the kana add-on 35,559. (A 16 × 32 version
was tried first and also divides every screen; it was dropped as too large on this
panel.)

### Direct mode: the framebuffer is the ring

At 800 resolution in portrait (rotation 0 or 180) the terminal asks the display for
framebuffers **1280 + 320 rows** tall (`setFramebufferRows()`; 16 spare lines × 20) and
no LVGL draw buffers (`setDrawBuffers(false)`), and lays each out as [top band][ring]
[bottom band]. The ring is the scrolling window plus the spare rows. Text is drawn
straight into it, the touched rows are written back from cache, and the frame task sets
the scan layout: top band, window from the ring at an offset (two blocks where it wraps),
bottom band — at most four list items, rewritten every frame while scrolling. No pixel
is copied to scroll. Rotation 180 is the same layout stored upside down, with the window
running through the ring backwards.

The ring rules are the P4-86 terminal's: a new line is cleared and drawn out of sight
before it scrolls in, the window never scrolls past rows not yet cleared, and a row is
only cleared once its alias one ring-length earlier is above the window the display is
**actually scanning** — which in direct mode lags the window asked for by one frame.

### PPA mode: landscape and the 480 resolution

The DMA can only move the picture along the panel's native rows, so a landscape or
scaled terminal needs the PPA. The terminal keeps an upright surface in PSRAM laid out
[top][ring][ring again][bottom]; the ring is stored twice so any window of it is one
contiguous block. Each update is at most three PPA blits (header, window, footer) into a
framebuffer that isn't on screen, then a flip. At 480 the frozen strips are rounded up to
multiples of 8, so 13/8 scaling lands them on whole pixels.

Measured PPA time per update (full output): **47 ms** landscape at 800 (2 MB rotated),
**23 ms** at 480 (portrait or landscape, 1.9 MB written, 0.7 MB read). Rotation costs
about twice what scaling does per byte written.

**The compose runs unlocked, into a third framebuffer.** The task takes the terminal's
lock only to step the scroll and snapshot the layout, then composes from the snapshot
without it. The PPA reads only the window at the snapshot's position; only the task
moves the window, and new lines are only ever cleared into the spare rows, so callers
keep printing while the PPA works. With three framebuffers (`setPpaFramebuffers()`,
default 3) a compose starts as soon as the last ends: a framebuffer is free once the
frame count has passed the count read right after the flip away from it (the frame-done
interrupt counts the frame before it reads which framebuffer to arm next, so a count
read after the flip is never too low). In PPA mode the task goes round again at once
after a compose rather than waiting for the next frame.

### Spare lines and the smooth-scroll limit

Each update scrolls a quarter of the backlog, so the backlog — bounded by the spare
lines — sets the burst rate. Measured with 16 × 32 cells during development, 300-line
bursts, smooth scrolling with no limit:

| Spare lines | Landscape 800 (PPA) | Portrait 480 (PPA) | Portrait 800 (direct) |
|---|---|---|---|
| 8, before the unlocked compose and third framebuffer | 28 lines/s | 21 lines/s | 88 lines/s |
| 8 | 33 | 34 | 88 |
| 16 | 60 | 63 | — |
| 32 | 96 | 109 | 265 |

The default is 16 spare lines (`setSpareLines()`, 1–64), and smooth scrolling is limited
to **100 lines/s** (`setSmoothScrollLimit()`; 0 = none) so text is still seen to move.
The limit is rows per elapsed time, so it holds whether the task goes round once a frame
or once a compose. Instant scrolling is never limited. With the 10 × 20 font, defaults:

| Mode | Smooth | Instant |
|---|---|---|
| 800 portrait (direct) | 117 lines/s | 422 lines/s |
| 800 landscape (PPA) | 66 | 152 |
| 480 portrait (PPA) | 102 | 304 |
| 480 landscape (PPA) | 98 | 302 |

Landscape at 800 can't reach the limit: 47 ms per update. The faster design would store
the landscape text pre-rotated, so the PPA only copies; not done.

The tuning used to be `#define`s "set before including". They never reached the
library's `.cpp`, and the stack size sized an array in the class, so overriding it would
have given the sketch and the library different layouts of one object. They are setters
now; the task stack is fixed at 4 KB, of which the terminal uses under 1 KB.

### Memory

| Mode | PSRAM |
|---|---|
| 800 portrait (direct) | 2 framebuffers × 1600 rows × 1600 B = 5.1 MB |
| 800 landscape (PPA) | surface (2 × 800 + 2 × 320 rows) × 2560 B = 5.7 MB, + 3 framebuffers × 2.05 MB = 11.9 MB |
| 480 (PPA) | surface about 2 MB, + 6.1 MB of framebuffers |

### The cursor

The whole screen is in memory, so the cursor saves the pixels it covers (up to a
double-width cell) and puts them back: it never damages old text.

## The raster band

`setRasterRows()` ([README-raster.md](README-raster.md)) makes rows of the panel come
not from a framebuffer but from small internal-RAM buffers (2 to 8), filled by an application
function just before the panel scans them. It came from the S3 RGB-panel libraries by
way of `chipguy_P486Panel_display`.

- **The list.** Rows `[y0, y0 + chunks·R)` become `chunks` list items of R rows each, from
  `bufs[0]`, `bufs[1]`, `bufs[0]`, …, with the framebuffer above and below as one item
  each. `chunks` must be even, so chunk 0 of every frame uses `bufs[0]`.
- **The interrupt.** Each chunk's item raises a block-done interrupt when the DMA has
  *read* it, a few rows before the panel shows its last row (the bridge FIFO holds
  1024 × 64 bits, a row 200 of them). Then `bufs[c & 1]` is free. A chunk at the panel's
  bottom would coincide with the frame-done interrupt, which reports it instead.
- **The fill.** The handler queues chunk c + 2, wrapping into the next frame, for a
  top-priority task pinned to the caller's core. The task calls the function, then
  writes the buffer back from the L1 cache (`esp_cache_msync`). Drawing through the cache
  and writing back beats the non-cached alias by about four times (measured on the
  P4-86).
- **Late and dropped.** When chunk c has been read, chunk c + 1 must be drawn, except at
  the frame's last chunk, whose successor is the next frame's chunk 0. A full 8-entry
  queue drops the job and counts it done.

The rows are the panel's native rows, whatever LVGL's rotation: LVGL's PPA flip writes
the whole framebuffer, including rows the band hides, which costs nothing visible.

Measured with RasterBouncingBalls (16-row chunks, 51 KB of buffers, both cores, the
cache write-back included), 2026-11-13. Panel period 19,243 µs (52.0 Hz), chunk period
234 µs:

| Balls | Opaque | See-through | Late |
|---|---|---|---|
| 20 | 63 µs | 75 µs | 0 |
| 100 | 104 µs | 134 µs | 0 |
| 200 | 142 µs | 215 µs | 0 |

51.8 frames/s shown, no underruns; 246 KB of internal RAM free with LVGL's 128 KB pool.
A row here is 14.7 µs against the P4-86's 19.6, with 800 pixels against 720, so a chunk
of the same height has 75 % of the time for 11 % more pixels.

### More buffers, merged interrupts, direct chunks (2026-11-15)

Developed on a whole-panel raster band, 2026-11-13/14:

- **2 to 8 buffers** (`nbufs`). The band is a multiple of N chunks so chunk 0 always uses
  `bufs[0]`. With N ≥ 3 the job for chunk c + N − 1 is queued when chunk c has been read.
  That buffer was freed a chunk earlier, so the function has N − 2 chunk times, and the
  buffer just freed (chunk c + N) is the next one's, free for a prefetch. With 2 the old
  c + 2 stays.
- **Which chunks are done, from the DMA's position.** When a core's interrupts are held
  off (a long `esp_cache_msync`: the PPA driver's, LVGL's flush), two block-done
  interrupts arrive as one. Counted, every later chunk of the frame was reported a chunk
  early and its buffer refilled while the panel still read it. The handler now reads the
  DW-GDMA channel's next-item pointer (`LLP`): it points two items past the chunk just
  done, so every chunk up to that one is reported and none beyond (`rasterMerged()`
  counts the catch-ups). The first version assumed the band ends at the panel's
  bottom. Here the list's last item may be the framebuffer below the band, so with `LLP`
  = 0 the band's last chunk is done too. Checked 2026-11-15 with RasterBouncingBalls (band
  0..1119, 2 buffers): 52 fps, 0 late or dropped, 200 see-through balls in 206 µs a chunk.
- **Direct chunks** (`raster_direct`): a chunk's item patched in every list to read the
  framebuffer instead of its buffer. The bit is kept even before the band is applied (early calls were lost otherwise).
- **Framebuffers on the 128-byte cache line** (`heap_caps_aligned_calloc`), so the PPA
  may write into them and ranges can be synced without `UNALIGNED`.
- **Late is exact**: a chunk counts when the DMA begins it before its job has finished.
  The old count also counted a second job waiting, which with 4 buffers is normal.

### The prefetch, and where the interrupts go (2026-11-15)

`setRasterPrefetch()` copies a column range of each chunk in by DMA before the function
runs. The copier (`jc8012_raster_copy.c`) is an AXI GDMA TX/RX pair on the
M2M trigger, burst 128, owner check on. The source side is mounted per copy. The
destination side, one piece per row when it isn't whole rows, is built once per bounce
buffer, and afterwards its items are only handed back to the DMA. The copy for chunk
c + 1 starts right after chunk c's has landed and its buffer's cache lines are dropped
(`M2C`; the previous fill ended in a write-back, so nothing dirty is lost). Columns are
whole 64-byte L1 lines (row starts are, 1600 = 25 × 64), so the CPU's and the DMA's
parts of a row never share a line.

The **pause** is a task, just under the raster task's priority, on the paused core. It is
notified as each copy starts and spins until the copy's interrupt clears its flag.
`vTaskSuspend()` on the LVGL task was rejected: suspending a task that is blocked takes it
off its event list, so a delay or semaphore wait would return early when it was resumed.
In RasterLandscapeBouncingBalls the balls' helper (higher priority, same core) usually
holds that core for the whole copy anyway, so the gate's own time (`rasterCopyPausedUs()`)
is small. Pause on or off made no measurable difference to lateness there. With LVGL
redrawing the whole strip every refresh, it cut the raster's wait for copies from ~480 to
~130 µs a second. That was before the interrupt fix below, and it was not re-measured
after it.

**Interrupts on the raster core.** Measured with the landscape example and LVGL redrawing
its whole strip every refresh (~30 flips/s, each with a 400 KB write-back and the PPA
driver's syncs). With the panel's and the copier's interrupts on LVGL's core: ~30 late
chunks/s, ~23 merged interrupts/s, copies "taking" ~440 µs (their landing interrupt
delayed). With both on the raster core (`setInterruptCore()`, and the copier made from
there): merged 0, copies ≤ 97 µs, still ~40 late/s. The rest was the balls' helper task on
LVGL's core, which can't be woken while that core's interrupts are off. Making its jobs
claimable by the raster task, slicing the flush's write-back into 8-row pieces, and giving
the PPA only the changed panel rows as its output picture took it to 0.

Measured 2026-11-15, RasterLandscapeBouncingBalls: whole panel, 80 chunks of 16 rows,
strip 160 columns (5 KB a copy), LVGL refresh capped at 50 ms, the board not touched.
With 4 buffers (102 KB) each chunk has 468 µs, and 194 KB of internal RAM is free. With 8
(205 KB) each chunk has 1404 µs, and 77.5 KB is free, the largest block 31.7 KB. The
example ships with 8. The buffer count must divide 80 when the band is the whole panel:
2, 4, 5 or 8.

| | 4 buffers: longest chunk (of 468 µs) | 8 buffers: longest chunk (of 1404 µs) | Copy | Late |
|---|---|---|---|---|
| 200 opaque balls | 169 µs | 236 µs | 42 µs | 0 |
| 400 opaque balls | — | 355 µs | 42 µs | 0 |
| 200 see-through balls | 191–205 µs | 205 µs | 42 µs | 0 |
| 300 see-through balls | 248 µs | 249 µs | 42 µs | 0 |
| 400 see-through balls (the slider's top) | 301 µs | 350 µs | 43 µs | 0 |
| 100 balls, LVGL redrawing the whole strip (20 flips/s) | 441 µs | 494 µs | ≤ 116 µs | 0 |

The longest chunk includes any wait for a stalled helper, so it grows with the slack
rather than meaning the drawing got slower. Touching the sliders during a run raised
it to ~830 µs, still 0 late with 8 buffers.

A full-strip PPA turn (1280 × 160) takes ~7.5 ms. The dirty-columns-only turn for a
slider or label is ~2 ms.

## Experimental: a frame from any blocks

`display.setScanList(blocks, n)` (`chipguy_lcd_dpi_panel_set_blocks`) reads each frame as
an arbitrary list of {source, bytes} blocks that together make one 800 × 1280 frame. A
block may end mid-row, so different parts of one row can come from different buffers.

**The 2560-block test** (2026-11-13): every row read as two 800-byte halves, the left
from one PSRAM buffer at that row, the right from another buffer upside down, then the
halves swapped, then switched every 250 ms. The user confirmed the picture.

| Phase | Frames/s | Underruns |
|---|---|---|
| Framebuffer alone | 52.0 | 0 |
| 2560 blocks, static | 51.8 | 0 |
| 2560 blocks, rebuilt 4 times a second | 48.0 | 0 |

- **Bandwidth is no problem:** fetching a 64-byte descriptor from internal RAM per 400
  pixels, with sources scattered in PSRAM, kept full rate.
- **Rebuilding costs a frame.** A change makes the frame-done interrupt rewrite all
  2560 items, which takes longer than the vertical blanking (32 rows, about 470 µs), so
  the next frame starts late — one frame lost per change, never an underrun.
- **Internal RAM decides the size.** A 2560-item list is 160 KB. Two (one per
  framebuffer) didn't fit — `dw_gdma_new_link_list` failed with "no mem" — so the test
  ran with `setFramebufferCount(1)`; free internal RAM went from 344 KB to 179 KB.

### Design note: a second framebuffer for part of the screen

What this was tested for, and is not implemented: a **rectangle of the screen read
from a buffer of its own**, scrolled frame-locked by the display hardware — a list or a
document view that moves smoothly at the panel's rate — while LVGL draws everything
outside it into the framebuffers as usual.

- **Blocks.** Each row that crosses the rectangle is three blocks: the framebuffer left
  of it, the region buffer, the framebuffer right of it (two at a screen edge). Rows above
  and below are one block each, since consecutive framebuffer rows are contiguous. A
  rectangle R rows tall costs 3R + 2 items. Its left and right edges must fall on 4-pixel
  boundaries (8 bytes); 8-pixel boundaries are comfortably inside that.
- **Scrolling.** The region buffer can be taller than the rectangle — a whole document —
  and scrolling is a change of which buffer row each region block starts at, wrapping if
  the buffer is a ring. Horizontal scrolling works too, on 4-pixel steps.
- **The cost to solve: changing the list every frame.** Rewriting every item through
  ESP-IDF's `dw_gdma_lli_config_transfer` takes longer than the vertical blanking at
  2560 items (above), so a scroll every frame would halve the frame rate. But a scroll
  changes only the region blocks' source addresses. Storing just those (one 32-bit
  `sar` write per region row into the idle list, a few microseconds for hundreds of rows),
  or building the next list in a task and swapping it in at the boundary, would fit.
- **Internal RAM.** A full-height region is 3 × 1280 + 2 = 3842 items, 240 KB per list.
  One list fits; two don't, and LVGL's flip uses two framebuffers. Options: since the
  region's rows are shared, one list could serve both framebuffers with only the
  framebuffer blocks' addresses switched on a flip; or a shorter region (a 600-row
  region is about 115 KB per list); or descriptors in PSRAM (untested: a descriptor fetch
  from PSRAM between 400-pixel blocks may not keep up).
- **LVGL.** LVGL keeps drawing the whole screen; the rows under the rectangle are simply
  not shown. Its PPA flip already rewrites the whole framebuffer each refresh, so nothing
  in LVGL changes. Touches inside the rectangle are the application's to route.
- **Tearing.** The region buffer is read while the panel scans, so a region drawn into
  while it is shown can tear within one frame. Drawing ahead of the scroll into rows not
  yet visible — as the terminal's ring does — avoids it.

A full-width region needs none of this: it is the terminal's [scan layout](#direct-mode-the-framebuffer-is-the-ring),
four blocks a frame.

## Touch: GSL3680

The controller sits on the panel's flex (FPC2): reset GPIO22, interrupt GPIO21, I²C at
0x40 on the board's **shared** bus (GPIO7/8). It has no flash of its own: every
`begin()` resets it, uploads its firmware and checks that it started (register 0xB0
reads `5A5A5A5A`). The driver, `gsl3680_touch.cpp`, is this library's own (MIT); the
register sequence is the one the vendor bring-up uses.

**The firmware lives in NVS, not here.** It is Silead's, from their GPL-2.0 gslX680
driver, so the library doesn't carry it: the
[firmware installer](https://github.com/chipguyhere/ESP32-P4-C6-firmware-installer/blob/main/README.md)'s
`InstallFirmware` writes it once to NVS namespace `jc8012`, key `gsl3680fw`. The
image is the controller's RAM page by page — page number, then 32 words, little-endian;
139 pages, 18,348 bytes — stored zlib-compressed (9,878 bytes), because the P4's default
NVS partition (20 KB, one page reserved) has no room for it raw. `begin()` inflates it
with the miniz in the P4's ROM, checks its SHA-256 against the constant in
`gsl3680_touch.cpp` (`650876e3…94006`) and only then uploads it (~4,450 four-byte
register writes). `status()` reports `NO_FIRMWARE` / `BAD_FIRMWARE` when NVS lacks it,
and the display's `begin()` then comes up without touch instead of spending its
reboot-and-retry budget, which only `NO_RESPONSE` / `NOT_RUNNING` can benefit from.

**No host algorithm.** Silead's driver feeds each report through a host-side tracker
(`gsl_point_id.c`, also GPL) that assigns finger IDs, filters, edge-corrects and
occasionally retunes the scan frequency. Measured against it on the bench (2026-11-15,
`JC8012_GSL3680_Probe`, 26 held targets over two runs plus 2.5k drag frames):

- Raw reports map linearly onto the panel: native x = Y·800/896, native y =
  1280 − X·1280/1664 (fits: 0.879–0.890 and −0.771…−0.778, r > 0.9997). 896 and 1664 are
  the 14 × 26 electrode grid in the controller's config at 64 units per electrode
  (inferred; it matches the fits). Error at the targets: raw 9.7 / 8.5 px RMS, the
  algorithm 10.7 / 9.2 — finger placement, not the controller. Contacts reach all edges.
- The algorithm smooths and lags: it trails raw by 2–3 reads (~40 ms); frame-to-frame
  acceleration median 2 px against raw's 6.
- Report layout at 0x80: byte 0 = contacts (up to 5 records fit; 4 seen), bytes 1–3 =
  a frame counter that advances once per scan (19 % of 16 ms polls repeated the previous
  scan), then 4 bytes per contact: Y (16 bits; a value with top bits set, `0x4xxx`, is a
  flag seen at touch-down and lift, not a position — the algorithm drops those) and X
  (12 bits, top 4 bits an ID that this firmware always leaves 0).
- Slot order is not stable: two fingers swapped slots 24 times in 1,546 frames.
- The frequency retune fired once per run. Its absence is the one effect not measured:
  **TODO (bench):** touch in an electrically noisy setting (cheap charger, motor).

**The bus.** `gsl3680_touch` talks through `board.Wire.readRegister()` /
`writeRegister()` from the board library — see its
[engineering notes](https://github.com/chipguyhere/JC8012P4A1C_board/blob/main/README-engineering.md#the-shared-i2c-bus).
The display object therefore takes a `JC8012P4A1C_Board&`, and `begin()` starts the
board (idempotently) before touch. Verified on hardware (2026-09-30): touch tracks the
finger through the shared bus, and the codec and RTC answer on it alongside.

**Filtering** (in `gsl3680_touch.cpp`, with the measurements that set each threshold in
the comments there), in native pixels. The holds are timed, not counted in reads:
counted, a caller polling slowly (LVGL behind a heavy frame — measured with
`LvglBouncingBalls` at a high ball count) stretched a tap into a press long enough to
swallow the next tap as a drag. Always on, because none adds lag:

1. A read that repeats the last frame counter returns the last result (if the counter
   hasn't moved for 150 ms it counts as a dropout, so a stalled controller can't hold a
   press forever; it normally advances every ~20 ms).
2. Flagged records are skipped. Failed or empty reads are ridden out for 65 ms (release
   debounce).
3. One finger is followed: the contact nearest the previous sample, not slot 0.

Off by default, on with `display.setTouchFilter(true)` (the same switch as
`chipguy_P486_or_JC8012_display`), for panels where a twitch does harm — a volume slider:

4. A step over 350 px is held for up to 100 ms, then the touch is dropped — the panel
   emits phantom contacts that park near an axis end for ~150 ms.
5. A 7-sample sliding median (~50 ms of lag at a 16 ms poll).

## Backlight

The LED string is driven by an **MP3202** boost converter running from the IP5306's 5 V
rail, not from 3.3 V. Its current is set by R13 ‖ R14 = **1.1 Ω** from `LEDK` to ground:
I_LED = V_FB / 1.1 Ω. With the MP3202's feedback reference (**TODO (datasheet)**; parts
of this class use ~100–200 mV) that is roughly **0.1–0.2 A** through the string.

Its **EN pin** is driven from GPIO23 through R15 0 Ω, with **R16 10 kΩ to ground**:
the backlight is off whenever the P4 isn't driving the pin — at reset, in deep sleep,
before `begin()`. `setBacklight()` dims it by PWM on EN with `analogWrite()` (the core's
LEDC default, 8-bit). **TODO (datasheet/bench):** the MP3202's permitted EN-PWM frequency
range, and whether low duty cycles flicker or whine.

Because the backlight is on the same 5 V rail as the speaker amplifier and is by far the
largest load, it dominates battery life — see the
[power library](https://github.com/chipguyhere/JC8012P4A1C_power/blob/main/README.md#power-consumption).
