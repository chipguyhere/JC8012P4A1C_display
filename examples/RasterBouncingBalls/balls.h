/*
 * balls.h - Bouncing balls drawn by the display's bounce fill, not by LVGL.
 *
 * balls_raster() is a display raster-band function (see setRasterRows() in
 * the display header): the driver calls it for each chunk (16 rows) of the ball
 * area just before the panel scans it, and it draws those rows straight into
 * an internal-RAM bounce buffer.  Nothing is drawn into PSRAM, and every frame of the
 * panel is a frame of the animation.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License
 */

#pragma once

#include <stdint.h>

#define BALLS_MAX 200

// Set up the ball area: width x height pixels, from panel row 0.
// framePeriodUs is the panel's frame time; each frame advances the physics
// by exactly that much, whether or not anything was late.
void balls_init(int width, int height, uint32_t framePeriodUs);

// Draw the bottom half of every chunk on another core (the display's raster
// task draws the top half).  Call once before setRasterRows().
void balls_start_helper(int core);

// The raster-band function to hand to display.setRasterRows().
void balls_raster(uint16_t* dst, int y, int rows, uint32_t frame, void* ctx);

// Controls, safe to set from any task; read once per frame.
void balls_set_count(int n);        // 1..BALLS_MAX
void balls_set_speed(int v);        // 1..100; 100 is the LVGL demo's speed
void balls_set_see_through(bool on); // 50 % transparent balls

// Frames drawn so far (one per panel refresh, unless one was missed).
uint32_t balls_frames(void);
