/*
 * balls.h - Bouncing balls drawn by the display's raster band, in landscape.
 *
 * balls_raster() is a display raster-band function (see setRasterRows() in
 * the display header).  The panel scans its native rows, which in this
 * landscape orientation are the picture's COLUMNS, right to left; each panel
 * row runs down one column, top to bottom.  So a chunk of 16 panel rows is a
 * 16-pixel-wide vertical slice of the picture, and every slice crosses both
 * the balls and the LVGL strip at the bottom.  balls_raster() draws the ball
 * part of each row (panel columns 0 .. height-1); the strip's part has already
 * been copied in by the display's raster prefetch.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 */

#pragma once

#include <stdint.h>

#define BALLS_MAX 400

// Set up the ball area: width x height pixels of the landscape picture, from
// its top-left corner (width 1280 here, height the rows above the strip).
// framePeriodUs is the panel's frame time; each frame advances the physics
// by exactly that much, whether or not anything was late.
void balls_init(int width, int height, uint32_t framePeriodUs);

// Draw half of every chunk on another core (the display's raster task draws
// the other half), and the physics there too.  Call once before setRasterRows().
void balls_start_helper(int core);

// The raster-band function to hand to display.setRasterRows().
void balls_raster(uint16_t* dst, int y, int rows, uint32_t frame, void* ctx);

// Controls, safe to set from any task; read once per frame.
void balls_set_count(int n);         // 1..BALLS_MAX
void balls_set_speed(int v);         // 1..100; 100 is the LVGL demo's speed
void balls_set_see_through(bool on); // 75 % opaque balls

// Frames drawn so far (one per panel refresh, unless one was missed).
uint32_t balls_frames(void);
