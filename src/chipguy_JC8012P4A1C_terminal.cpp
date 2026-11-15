/*
 * Console-style text terminal with hardware scrolling for the Guition
 * JC8012P4A1C.  See chipguy_JC8012P4A1C_terminal.h for the API and
 * README-terminal.md / README-engineering.md for how it works.
 *
 * Storage.  The scrolling area is a RING of _R pixel rows, _SP rows longer
 * than the window shown (_VSA rows), plus a frozen top band (_fixT rows) and
 * bottom band (_fixB rows):
 *   DIRECT mode -- the ring and bands ARE the DMA framebuffer (rows
 *     [top][ring][bottom], reversed for rotation 180), and the display's
 *     scan layout picks the window out of the ring each frame.
 *   PPA mode -- the ring and bands live in an upright surface of their own,
 *     with the ring stored TWICE back to back, so any window of it is one
 *     contiguous block.  The PPA composes header, window and footer into the
 *     back framebuffer, rotating and scaling, and the frame is flipped in.
 *
 * Positions in the scrolling area are virtual pixel rows; row v lives in ring
 * row v mod _R.  A row is cleared before it can scroll into view
 * (_clearedTo), and only once its alias one ring-length earlier has scrolled
 * off (v < _vtop + _R) -- that is where print() waits when it is too far
 * ahead.
 *
 * Locking.  _mutex (recursive) serializes callers.  _draw guards the storage
 * and the shared state between a caller and the task; a caller that has to
 * wait for scrolling releases _draw (never _mutex) while it waits.  _busy
 * keeps the task's cursor blink off a caller's half-finished work.
 *
 * Copyright (c) 2026 chipguyhere
 * MIT License -- see LICENSE
 */

#include "chipguy_JC8012P4A1C_terminal.h"

#ifdef ARDUINO_ESP32P4_DEV

#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "JC8012term";

#define CW JC8012T_CELL_W
static constexpr TickType_t BLINK_TICKS = pdMS_TO_TICKS(530);
static constexpr int BAR_W = 3;         // JC8012T_CURSOR_BAR width, pixels
static constexpr int UL_H = 4;          // JC8012T_CURSOR_UNDERLINE height, pixels
static constexpr int32_t MIN_STEP = 4;  // smooth scrolling: rows per frame, at least

static inline int32_t pmod(int32_t a, int32_t m) {
    int32_t r = a % m;
    return r < 0 ? r + m : r;
}

int32_t chipguy_JC8012P4A1C_terminal::_lineFloor(int32_t v) const {
    return v - pmod(v - _grid, _CH);
}

// ── Fonts ──────────────────────────────────────────────────────────────────

const uint8_t *chipguy_JC8012P4A1C_terminal::_glyph(uint16_t cp, uint32_t &len,
                                                  bool &wide) const {
    for (int pass = 0; pass < 2; pass++) {
        for (const jc8012t_font_t *f = _font; f; f = f->fallback) {
            int lo = 0, hi = (int)f->count - 1;
            while (lo <= hi) {
                int mid = (lo + hi) >> 1;
                uint16_t c = f->codepoints[mid];
                if (c == cp) {
                    uint32_t a = f->offsets[mid], b = f->offsets[mid + 1];
                    wide = (a & 0x80000000u) != 0;
                    a &= 0x7FFFFFFFu;
                    b &= 0x7FFFFFFFu;
                    len = b - a;
                    return f->data + a;
                }
                if (c < cp) lo = mid + 1; else hi = mid - 1;
            }
        }
        cp = 0xFFFD;        // missing: the replacement character
    }
    len = 0;
    wide = false;
    return nullptr;
}

bool chipguy_JC8012P4A1C_terminal::_isWide(uint16_t cp) const {
    uint32_t len; bool wide;
    _glyph(cp, len, wide);
    return wide;
}

// Decode a whole glyph (n = w x h pixels) through the 16-entry palette.  The
// token format is make_terminal_fonts.py's: runs of background / foreground,
// and literal runs of 4-bit levels; what the data doesn't cover is background.
static void decodeGlyph(const uint8_t *p, const uint8_t *end, int n,
                        const uint16_t *pal, uint16_t *out) {
    int i = 0;
    while (i < n && p < end) {
        uint8_t t = *p++;
        if (t < 0x80) {
            int len = (t & 0x3F) + 1;
            uint16_t c = pal[t < 0x40 ? 0 : 15];
            while (len-- && i < n) out[i++] = c;
        } else {
            int len = (t & 0x7F) + 1;
            const uint8_t *q = p;
            p += (len + 1) >> 1;
            for (int j = 0; j < len && i < n; j++) {
                uint8_t b = q[j >> 1];
                out[i++] = pal[(j & 1) ? (b & 0x0F) : (b >> 4)];
            }
        }
    }
    while (i < n) out[i++] = pal[0];
}

// 16 blends from bg (0) to fg (15), in the framebuffer's native RGB565.
void chipguy_JC8012P4A1C_terminal::_palette(uint16_t fg, uint16_t bg) {
    if (fg == _palFg && bg == _palBg) return;
    int fr = fg >> 11, fgn = (fg >> 5) & 0x3F, fb = fg & 0x1F;
    int br = bg >> 11, bgn = (bg >> 5) & 0x3F, bb = bg & 0x1F;
    for (int a = 0; a < 16; a++) {
        int r = (br * (15 - a) + fr * a + 7) / 15;
        int g = (bgn * (15 - a) + fgn * a + 7) / 15;
        int b = (bb * (15 - a) + fb * a + 7) / 15;
        _pal[a] = (uint16_t)((r << 11) | (g << 5) | b);
    }
    _palFg = fg;
    _palBg = bg;
}

// ── Storage ────────────────────────────────────────────────────────────────

uint16_t *chipguy_JC8012P4A1C_terminal::_rowPtr(Band b, int32_t idx, int copy, bool &rev) const {
    int32_t r;
    rev = false;
    if (_ppa) {
        // Surface: [top][ring][ring again][bottom], upright.
        switch (b) {
            case TOP:  r = idx; break;
            case RING: r = _fixT + copy * _R + idx; break;
            default:   r = _fixT + 2 * _R + idx; break;
        }
    } else if (_rot == 2) {
        // Framebuffer upside down: [bottom][ring][top], every row and every
        // pixel in reverse.  The DMA reads it top to bottom as usual.
        rev = true;
        switch (b) {
            case TOP:  r = _fixB + _R + (_fixT - 1 - idx); break;
            case RING: r = _fixB + (_R - 1 - idx); break;
            default:   r = _fixB - 1 - idx; break;
        }
    } else {
        switch (b) {
            case TOP:  r = idx; break;
            case RING: r = _fixT + idx; break;
            default:   r = _fixT + _R + idx; break;
        }
    }
    return _store + (size_t)r * _stride;
}

void chipguy_JC8012P4A1C_terminal::_markDirty(int region, uint8_t *p, size_t bytes) {
    if (!_dlo[region] || p < _dlo[region]) _dlo[region] = p;
    if (!_dhi[region] || p + bytes > _dhi[region]) _dhi[region] = p + bytes;
}

// Write the CPU's changes back to PSRAM, where the display DMA and the PPA
// read them.
void chipguy_JC8012P4A1C_terminal::_commit() {
    bool any = false;
    for (int i = 0; i < 4; i++) {
        if (_dlo[i] && _dhi[i] > _dlo[i]) {
            esp_cache_msync(_dlo[i], _dhi[i] - _dlo[i],
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
            any = true;
        }
        _dlo[i] = _dhi[i] = nullptr;
    }
    if (any) _dirty = true;
}

// One horizontal run of pixels: px[0..n), or n pixels of `color` when px is
// null.  x is a screen column; the ring is written in both copies (PPA).
void chipguy_JC8012P4A1C_terminal::_span(Band b, int32_t idx, int x, int n,
                                       const uint16_t *px, uint16_t color) {
    if (n <= 0) return;
    int copies = (_ppa && b == RING) ? 2 : 1;
    for (int c = 0; c < copies; c++) {
        bool rev;
        uint16_t *row = _rowPtr(b, idx, c, rev);
        uint16_t *d;
        if (!rev) {
            d = row + x;
            if (px) memcpy(d, px, n * sizeof(uint16_t));
            else for (int i = 0; i < n; i++) d[i] = color;
        } else {
            d = row + (_W - x - n);
            if (px) for (int i = 0; i < n; i++) d[n - 1 - i] = px[i];
            else for (int i = 0; i < n; i++) d[i] = color;
        }
        _markDirty(b == TOP ? 0 : b == BOTTOM ? 3 : 1 + c, (uint8_t *)d, n * sizeof(uint16_t));
    }
}

void chipguy_JC8012P4A1C_terminal::_readSpan(Band b, int32_t idx, int x, int n, uint16_t *out) const {
    bool rev;
    const uint16_t *row = _rowPtr(b, idx, 0, rev);
    if (!rev) memcpy(out, row + x, n * sizeof(uint16_t));
    else {
        const uint16_t *s = row + (_W - x - n);
        for (int i = 0; i < n; i++) out[i] = s[n - 1 - i];
    }
}

// Which stored row is on screen row y, with the window at virtual row vtop.
void chipguy_JC8012P4A1C_terminal::_screenRow(int y, int32_t vtop, Band &b, int32_t &idx) const {
    if (y < _fixT)              { b = TOP;    idx = y; }
    else if (y >= _fixT + _VSA) { b = BOTTOM; idx = y - _fixT - _VSA; }
    else                        { b = RING;   idx = pmod(vtop + (y - _fixT), _R); }
}

void chipguy_JC8012P4A1C_terminal::_layout(int16_t T, int16_t B) {
    _fixT = T;
    _fixB = B;
    _VSA = _H - T - B;
    _R = _VSA + _SP;
}

// ── begin ──────────────────────────────────────────────────────────────────

bool chipguy_JC8012P4A1C_terminal::begin(chipguy_JC8012P4A1C_display &display, uint16_t rotation,
                                       uint16_t resolution, const jc8012t_font_t *font) {
    if (_task) return true;
    if (!font || font->cellW != CW || font->cellH < 8 || font->cellH > JC8012T_MAX_CELL_H) {
        ESP_LOGE(TAG, "font cells must be %d wide and at most %d tall", CW, JC8012T_MAX_CELL_H);
        return false;
    }
    if (rotation >= 1 && rotation <= 3) rotation *= 90;
    if (rotation % 90 || rotation >= 360) rotation = 0;
    _rot = rotation / 90;
    // The upright screen: native 800 x 1280 (portrait at 0 / 180, landscape
    // 1280 x 800 at 90 / 270), or 480 pixels across the panel's short side,
    // scaled 1.625x (26/16, exact for the PPA) into a centred 780 x 1235.
    // 760 rather than 768 for the long side, so 20-pixel lines fit it exactly.
    bool small = (resolution == 480);
    int16_t shortSide = small ? 480 : JC8012_NATIVE_WIDTH;
    int16_t longSide  = small ? 760 : JC8012_NATIVE_HEIGHT;
    _W = (_rot & 1) ? longSide : shortSide;
    _H = (_rot & 1) ? shortSide : longSide;
    _num = small ? 13 : 1;
    _den = small ? 8 : 1;
    _rw = shortSide * _num / _den;              // its area on the panel
    _rh = longSide * _num / _den;
    _ox = (JC8012_NATIVE_WIDTH - _rw) / 2;
    _oy = (JC8012_NATIVE_HEIGHT - _rh) / 2;
    _ppa = small || (_rot & 1);
    _font = font;
    _CH = font->cellH;
    _cols = _W / CW;
    _SP = _spareLines * _CH;
    _disp = &display;

    // Direct mode scrolls inside the framebuffer, which needs the spare rows.
    // The display stays at rotation 0 either way: the terminal rotates.
    if (!_ppa) display.setFramebufferRows(JC8012_NATIVE_HEIGHT + _SP);
    display.setDrawBuffers(false);              // no LVGL: the terminal draws
    if (_ppa) display.setFramebufferCount(_ppaFbs);
    if (!display.begin(0)) return false;
    if (!_ppa && display.framebufferRows() < JC8012_NATIVE_HEIGHT + _SP) return false;

    _layout(0, 0);
    if (_ppa) {
        size_t rows = 2 * (size_t)_H + 2 * (size_t)_SP;    // worst case of T + 2R + B
        _store = (uint16_t *)heap_caps_aligned_alloc(128, rows * _W * sizeof(uint16_t),
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!_store) {
            ESP_LOGE(TAG, "no PSRAM for the %dx%d surface", _W, (int)rows);
            return false;
        }
        _stride = _W;
        ppa_client_config_t pc = {};
        pc.oper_type = PPA_OPERATION_SRM;
        if (ppa_register_client(&pc, &_ppaClient) != ESP_OK) {
            ESP_LOGE(TAG, "PPA client registration failed");
            return false;
        }
    } else {
        _store = display.getFramebuffer(0);
        _stride = JC8012_NATIVE_WIDTH;
        if (!_store) return false;
    }

    _mutex = xSemaphoreCreateRecursiveMutexStatic(&_mutexBuf);
    _draw  = xSemaphoreCreateMutexStatic(&_drawBuf);
    _prog  = xSemaphoreCreateBinaryStatic(&_progBuf);
    _palFg = _palBg = 1;
    _clearAll(true, JC8012T_BLACK);
    _commit();
    _dirty = true;

    _task = xTaskCreateStaticPinnedToCore(_taskEntry, "jc8012term",
                                          JC8012T_STACK_BYTES / sizeof(StackType_t), this,
                                          _taskPrio, _stack, &_tcb, _taskCore);
    ESP_LOGI(TAG, "%dx%d, rotation %d, %s mode, %d x %d cells", _W, _H, _rot * 90,
             _ppa ? "PPA" : "direct", _cols, (int)(_VSA / _CH));
    return _task != nullptr;
}

bool chipguy_JC8012P4A1C_terminal::setFont(const jc8012t_font_t *font) {
    if (!font || font->cellW != CW || font->cellH != _CH) return false;
    if (!_task) { _font = font; return true; }
    xSemaphoreTakeRecursive(_mutex, portMAX_DELAY);
    _font = font;
    xSemaphoreGiveRecursive(_mutex);
    return true;
}

uint32_t chipguy_JC8012P4A1C_terminal::stackHighWater() const {
    return _task ? uxTaskGetStackHighWaterMark(_task) * sizeof(StackType_t) : 0;
}

// ── Locking and waiting ────────────────────────────────────────────────────

void chipguy_JC8012P4A1C_terminal::_lock() {
    xSemaphoreTakeRecursive(_mutex, portMAX_DELAY);
    xSemaphoreTake(_draw, portMAX_DELAY);
    _busy = true;
    _liftCursor();
}

void chipguy_JC8012P4A1C_terminal::_unlock() {
    _showCursor();
    _commit();
    _busy = false;
    xSemaphoreGive(_draw);
    xSemaphoreGiveRecursive(_mutex);
}

// Called holding _draw: let the task run one frame, then take it back.
void chipguy_JC8012P4A1C_terminal::_waitFrame() {
    _commit();
    xSemaphoreGive(_draw);
    xSemaphoreTake(_prog, pdMS_TO_TICKS(100));
    xSemaphoreTake(_draw, portMAX_DELAY);
}

void chipguy_JC8012P4A1C_terminal::_flushLocked() {
    _commit();
    while (_vtop != _vtopT || _dirty || _composing) _waitFrame();
}

void chipguy_JC8012P4A1C_terminal::flush() {
    if (!_task) return;
    _lock();
    _flushLocked();
    _unlock();
}

// Make sure every row that can come into view, or that the cursor's line
// covers, has been cleared -- waiting, if print() is too far ahead, until the
// row's alias one ring-length earlier has scrolled off the top.  "Off the
// top" is judged against the window the display is scanning NOW: in direct
// mode a new scroll offset only takes effect at the next frame boundary, and
// clearing against the newer one blanked rows still on screen for a frame.
void chipguy_JC8012P4A1C_terminal::_ensureReady() {
    int32_t end = max(_vtopT + (int32_t)_VSA, _curTop + (int32_t)_CH);
    while (_clearedTo < end) {
        int32_t lim = min(end, min(_vtop, _vtopLive) + _R);
        if (_clearedTo >= lim) { _waitFrame(); continue; }
        for (int32_t v = _clearedTo; v < lim; v++)
            _span(RING, pmod(v, _R), 0, _W, nullptr, _screenBg);
        _clearedTo = lim;
    }
}

// ── Text ───────────────────────────────────────────────────────────────────

size_t chipguy_JC8012P4A1C_terminal::write(uint8_t b) {
    return write(&b, 1);
}

size_t chipguy_JC8012P4A1C_terminal::write(const uint8_t *buf, size_t n) {
    if (!_task) return 0;
    _lock();
    for (size_t i = 0; i < n; i++) _feedByte(buf[i]);
    _unlock();
    return n;
}

void chipguy_JC8012P4A1C_terminal::putCodepoint(uint32_t cp) {
    if (!_task) return;
    _lock();
    _u8need = 0;
    _putCp(cp);
    _unlock();
}

// UTF-8 decoder.  A malformed sequence prints U+FFFD.
void chipguy_JC8012P4A1C_terminal::_feedByte(uint8_t b) {
    if (_u8need) {
        if ((b & 0xC0) == 0x80) {
            _u8cp = (_u8cp << 6) | (b & 0x3F);
            if (--_u8need == 0) _putCp(_u8cp);
            return;
        }
        _u8need = 0;
        _putCp(0xFFFD);           // truncated sequence; b starts afresh
    }
    if (b < 0x80)                { _putCp(b); }
    else if ((b & 0xE0) == 0xC0) { _u8cp = b & 0x1F; _u8need = 1; }
    else if ((b & 0xF0) == 0xE0) { _u8cp = b & 0x0F; _u8need = 2; }
    else if ((b & 0xF8) == 0xF0) { _u8cp = b & 0x07; _u8need = 3; }
    else                         { _putCp(0xFFFD); }
}

void chipguy_JC8012P4A1C_terminal::_putCp(uint32_t cp) {
    switch (cp) {
        case '\n': _newline(); return;
        case '\r': _moveCursor(_curTop, 0); return;
        case '\b':
            if (_curCol > 0) _moveCursor(_curTop, min<int16_t>(_curCol, _cols) - 1);
            return;
        case '\t':
            // Destructive: spaces to the next multiple of 4, stopping at a wrap.
            if (_curCol >= _cols) { _newline(); return; }   // parked: column 0 is a stop
            do {
                _putGlyph(' ');
            } while (_curCol % 4 != 0 && _curCol < _cols);
            return;
        case '\v': _clearEol(); return;
        case '\f': _clearAll(false, _bg); return;
    }
    if (cp < 0x20 || cp == 0x7F) return;
    if (cp > 0xFFFD) cp = 0xFFFD;
    _putGlyph((uint16_t)cp);
}

// Draw character cp in cell (lineTop, col).  w returns its width in pixels.
void chipguy_JC8012P4A1C_terminal::_drawCell(int32_t lineTop, int col, uint16_t cp,
                                           uint16_t fg, uint16_t bg, int &w) {
    uint32_t len;
    bool wide;
    const uint8_t *g = _glyph(cp, len, wide);
    w = wide ? 2 * CW : CW;
    if (col * CW + w > _W) w = _W - col * CW;
    _palette(fg, bg);
    int gw = wide ? 2 * CW : CW;
    if (g) decodeGlyph(g, g + len, gw * _CH, _pal, _glyphBuf);
    else for (int i = 0; i < gw * _CH; i++) _glyphBuf[i] = _pal[0];
    for (int py = 0; py < _CH; py++)
        _span(RING, pmod(lineTop + py, _R), col * CW, w, _glyphBuf + py * gw, 0);
}

void chipguy_JC8012P4A1C_terminal::_putGlyph(uint16_t cp) {
    bool wide = _isWide(cp) && _cols >= 2;
    int need = wide ? 2 : 1;
    if (_curCol >= _cols) _newline();                // parked past the edge
    if (_curCol + need > _cols) _newline();          // a wide glyph that won't fit

    int c = _curCol, w;
    // Don't leave half of a wide character behind on either side.
    if (_cells[c] == CELL_WIDE_R && c > 0) {
        _cells[c - 1] = ' ';
        _drawCell(_curTop, c - 1, ' ', _fg, _bg, w);
    }
    if (c + need < _cols && _cells[c + need] == CELL_WIDE_R) {
        _cells[c + need] = ' ';
        _drawCell(_curTop, c + need, ' ', _fg, _bg, w);
    }
    _cells[c] = cp;
    if (wide) _cells[c + 1] = CELL_WIDE_R;
    _drawCell(_curTop, c, cp, _fg, _bg, w);

    int16_t next = c + need;
    if (next >= _cols) {
        // Last cell filled: park just past the edge, so a '\n' that follows
        // doesn't leave a blank line.  Or wrap now, if asked to.
        if (_immediateWrap) _newline();
        else _moveCursor(_curTop, _cols);
    } else {
        _moveCursor(_curTop, next);
    }
}

void chipguy_JC8012P4A1C_terminal::_newline() {
    int32_t top = _curTop + _CH;
    if (top + _CH > _vtopT + _VSA) _vtopT = top + _CH - _VSA;
    _moveCursor(top, 0);
}

void chipguy_JC8012P4A1C_terminal::_moveCursor(int32_t top, int16_t col) {
    if (top != _curTop)
        for (int c = 0; c < _cols; c++) _cells[c] = CELL_KEEP;   // not this line's
    _curTop = top;
    _curCol = col;
    _blinkOn = true;
    _blinkAt = xTaskGetTickCount() + BLINK_TICKS;
    _ensureReady();
}

void chipguy_JC8012P4A1C_terminal::_clearEol() {
    if (_curCol >= _cols) return;
    int w;
    // A wide character whose right half is about to be cleared loses its left half too.
    if (_curCol > 0 && _cells[_curCol] == CELL_WIDE_R) {
        _cells[_curCol - 1] = ' ';
        _drawCell(_curTop, _curCol - 1, ' ', _fg, _bg, w);
    }
    for (int c = _curCol; c < _cols; c++) _cells[c] = ' ';
    for (int py = 0; py < _CH; py++)
        _span(RING, pmod(_curTop + py, _R), _curCol * CW, (_cols - _curCol) * CW, nullptr, _bg);
}

void chipguy_JC8012P4A1C_terminal::clearToEndOfLine() {
    if (!_task) return;
    _lock();
    _clearEol();
    _unlock();
}

void chipguy_JC8012P4A1C_terminal::_clearAll(bool includeFrozen, uint16_t color) {
    _screenBg = color;
    for (int32_t i = 0; i < _R; i++) _span(RING, i, 0, _W, nullptr, color);
    if (includeFrozen) {
        for (int i = 0; i < _fixT; i++) _span(TOP, i, 0, _W, nullptr, color);
        for (int i = 0; i < _fixB; i++) _span(BOTTOM, i, 0, _W, nullptr, color);
    }
    _vtop = _vtopT = _vtopLive = 0;   // the window jumps back to the ring's start
    _clearedTo = _R;
    _grid = 0;
    _curTop = 0;
    _curCol = 0;
    for (int c = 0; c < _cols; c++) _cells[c] = ' ';
    _blinkOn = true;
    _blinkAt = xTaskGetTickCount() + BLINK_TICKS;
    _dirty = true;
}

void chipguy_JC8012P4A1C_terminal::clear(bool includeFrozen) {
    if (!_task) return;
    _lock();
    _clearAll(includeFrozen, _bg);
    _unlock();
}

// ── Cursor ─────────────────────────────────────────────────────────────────

int16_t chipguy_JC8012P4A1C_terminal::rows() const {
    if (!_CH) return 0;
    int32_t r0 = _lineFloor(_vtopT);
    return (int16_t)((_vtopT + _VSA - r0 + _CH - 1) / _CH);
}

int16_t chipguy_JC8012P4A1C_terminal::cursorRow() const {
    return (int16_t)((_curTop - _lineFloor(_vtopT)) / _CH);
}

void chipguy_JC8012P4A1C_terminal::setCursor(int16_t col, int16_t row) {
    if (!_task) return;
    _lock();
    int16_t nrows = rows();
    if (row < 0) row = 0;
    if (row >= nrows) row = nrows - 1;
    if (col < 0) col = 0;
    if (col >= _cols) col = _cols - 1;
    _moveCursor(_lineFloor(_vtopT) + (int32_t)row * _CH, col);
    _unlock();
}

void chipguy_JC8012P4A1C_terminal::setCursorStyle(jc8012t_cursor_t style) {
    if (!_task) { _cursorStyle = style; return; }
    _lock();                          // lifts the old one...
    _cursorStyle = style;
    _blinkOn = true;
    _blinkAt = xTaskGetTickCount() + BLINK_TICKS;
    _unlock();                        // ...and shows the new
}

// Save the pixels the cursor will cover, then draw it.
void chipguy_JC8012P4A1C_terminal::_showCursor() {
    if (_csShown || _cursorStyle == JC8012T_CURSOR_OFF || !_blinkOn || _curCol >= _cols) return;
    int c = _curCol;
    bool wide = c + 1 < _cols && _cells[c] != CELL_KEEP && _cells[c] != CELL_WIDE_R &&
                _cells[c + 1] == CELL_WIDE_R;
    int x = c * CW, w, y0 = 0, h = _CH;
    switch (_cursorStyle) {
        case JC8012T_CURSOR_BAR:       w = BAR_W; break;
        case JC8012T_CURSOR_UNDERLINE: w = CW; y0 = _CH - UL_H; h = UL_H; break;
        default:                     w = wide ? 2 * CW : CW; break;
    }
    for (int r = 0; r < h; r++) {
        int32_t idx = pmod(_curTop + y0 + r, _R);
        _readSpan(RING, idx, x, w, _curSave + r * w);
        _span(RING, idx, x, w, nullptr, _fg);
    }
    _csShown = true;
    _csTop = _curTop; _csX = x; _csY = y0; _csW = w; _csH = h;
}

void chipguy_JC8012P4A1C_terminal::_liftCursor() {
    if (!_csShown) return;
    for (int r = 0; r < _csH; r++)
        _span(RING, pmod(_csTop + _csY + r, _R), _csX, _csW, _curSave + r * _csW, 0);
    _csShown = false;
}

void chipguy_JC8012P4A1C_terminal::evictPartialLine(bool backward) {
    if (!_task) return;
    _lock();
    if (!backward) {
        int32_t p = pmod(_vtopT - _grid, _CH);
        if (p) {
            _vtopT += _CH - p;
            if (_curTop < _vtopT) _moveCursor(_vtopT, min<int16_t>(_curCol, _cols - 1));
            else _ensureReady();
        }
    } else {
        _flushLocked();
        int32_t q = pmod(_vtopT + _VSA - _grid, _CH);
        if (q) {
            // Rows scrolled in at the top come in blank.
            for (int32_t v = _vtopT - q; v < _vtopT; v++)
                _span(RING, pmod(v, _R), 0, _W, nullptr, _screenBg);
            _vtopT -= q;
            if (_curTop + _CH > _vtopT + _VSA)          // the cursor's line went off the bottom
                _moveCursor(_curTop - _CH, min<int16_t>(_curCol, _cols - 1));
        }
    }
    _unlock();
}

// ── Freeze ─────────────────────────────────────────────────────────────────

bool chipguy_JC8012P4A1C_terminal::freeze(uint16_t topPixels, uint16_t bottomPixels) {
    if (!_task) return false;
    int T2 = topPixels, B2 = bottomPixels;
    if (_den != 1) { T2 = (T2 + 7) & ~7; B2 = (B2 + 7) & ~7; }  // whole pixels at 1.625x
    if (T2 + B2 + _CH > _H) return false;
    uint16_t *snap = (uint16_t *)heap_caps_malloc((size_t)_W * _H * sizeof(uint16_t),
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!snap) return false;

    _lock();
    _flushLocked();
    // Keep the screen exactly as it is: copy it out, change the layout, put it back.
    Band b; int32_t idx;
    for (int y = 0; y < _H; y++) {
        _screenRow(y, _vtop, b, idx);
        _readSpan(b, idx, 0, _W, snap + (size_t)y * _W);
    }
    int32_t yc = _fixT + (_curTop - _vtop);               // cursor line, on screen

    _layout(T2, B2);
    _vtop = _vtopT = _vtopLive = 0;
    for (int y = 0; y < _H; y++) {
        _screenRow(y, 0, b, idx);
        _span(b, idx, 0, _W, snap + (size_t)y * _W, 0);
    }
    for (int32_t v = _VSA; v < _R; v++) _span(RING, v, 0, _W, nullptr, _screenBg);
    _clearedTo = _R;

    if (yc < _fixT) yc = _fixT;
    if (yc > _fixT + _VSA - _CH) yc = _fixT + _VSA - _CH;
    _curTop = yc - _fixT;
    _grid = pmod(_curTop, _CH);
    if (_curCol > _cols - 1) _curCol = _cols - 1;
    for (int c = 0; c < _cols; c++) _cells[c] = CELL_KEEP;
    _dirty = true;
    _unlock();
    free(snap);
    return true;
}

// ── Bitmaps ────────────────────────────────────────────────────────────────

void chipguy_JC8012P4A1C_terminal::drawBitmap(int16_t x, int16_t y, int16_t w, int16_t h,
                                            const uint16_t *pixels) {
    if (!_task || !pixels || w <= 0 || h <= 0) return;
    int x0 = max<int>(x, 0), y0 = max<int>(y, 0);
    int x1 = min<int>(x + w, _W), y1 = min<int>(y + h, _H);
    if (x0 >= x1 || y0 >= y1) return;
    _lock();
    if (y0 < _fixT + _VSA && y1 > _fixT) _flushLocked();   // land where (x, y) is now
    Band b; int32_t idx;
    for (int yy = y0; yy < y1; yy++) {
        _screenRow(yy, _vtopT, b, idx);
        _span(b, idx, x0, x1 - x0, pixels + (size_t)(yy - y) * w + (x0 - x), 0);
    }
    _unlock();
}

// ── Touch ──────────────────────────────────────────────────────────────────

bool chipguy_JC8012P4A1C_terminal::pollTouch(int16_t &x, int16_t &y, jc8012t_touch_units_t units) {
    if (!_task || !_disp->touchAvailable()) return false;
    uint16_t tx, ty;
    if (!_disp->getTouchDriver().getTouch(&tx, &ty)) return false;
    // Native panel coordinates -> the screen's area on the panel -> the
    // upright screen, then its resolution.
    int32_t rx = (int32_t)tx - _ox, ry = (int32_t)ty - _oy;
    int32_t rw = _rw, rh = _rh;
    if (rx < 0 || ry < 0 || rx >= rw || ry >= rh) return false;   // in the 480 mode's margin
    int32_t lx, ly;
    switch (_rot) {
        case 1:  lx = ry;          ly = rw - 1 - rx; break;
        case 2:  lx = rw - 1 - rx; ly = rh - 1 - ry; break;
        case 3:  lx = rh - 1 - ry; ly = rx;          break;
        default: lx = rx;          ly = ry;          break;
    }
    lx = lx * _den / _num;
    ly = ly * _den / _num;
    if (units == JC8012T_TOUCH_PIXELS) {
        x = lx;
        y = ly;
        return true;
    }
    if (ly < _fixT || ly >= _fixT + _VSA) return false;
    int32_t vt = _vtopT;
    int32_t v = vt + (ly - _fixT);
    x = min<int16_t>(lx / CW, _cols - 1);
    y = (int16_t)((v - _lineFloor(vt)) / _CH);
    return true;
}

// ── printf ─────────────────────────────────────────────────────────────────

size_t chipguy_JC8012P4A1C_terminal::printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    size_t n = vprintf(fmt, ap);
    va_end(ap);
    return n;
}

size_t chipguy_JC8012P4A1C_terminal::vprintf(const char *fmt, va_list ap) {
    if (!_task || !fmt) return 0;
    _lock();
    using Self = chipguy_JC8012P4A1C_terminal;
    jc8012t_fmt_sink sink = {
        [](void *c, const char *s, size_t n) {
            for (size_t i = 0; i < n; i++) ((Self *)c)->_feedByte((uint8_t)s[i]);
        },
        [](void *c, uint32_t cp) { ((Self *)c)->_putCp(cp); },
        [](void *c, uint16_t cp) { return ((Self *)c)->_isWide(cp); },
        this,
    };
    size_t total = jc8012t_term_vformat(sink, fmt, ap);
    _unlock();
    return total;
}

// Screen cells taken by UTF-8 text, reading at most maxBytes and stopping
// before a sequence that would be cut by that limit.  Returns the byte count
// used through `bytes`.
static int utf8Cells(const char *s, int maxBytes, int &bytes, const jc8012t_fmt_sink &sink) {
    int i = 0, cells = 0;
    while ((maxBytes < 0 || i < maxBytes) && s[i]) {
        uint8_t b = (uint8_t)s[i];
        int k = b < 0x80 ? 1 : (b & 0xE0) == 0xC0 ? 2 : (b & 0xF0) == 0xE0 ? 3
              : (b & 0xF8) == 0xF0 ? 4 : 1;
        if (maxBytes >= 0 && i + k > maxBytes) break;
        uint32_t cp = b;
        if (k > 1) {
            cp = b & (0x7F >> k);
            for (int j = 1; j < k; j++) {
                if (!s[i + j]) { k = j; break; }
                cp = (cp << 6) | ((uint8_t)s[i + j] & 0x3F);
            }
        }
        if (cp >= 0x20) cells += (cp <= 0xFFFF && sink.wide(sink.ctx, (uint16_t)cp)) ? 2 : 1;
        i += k;
    }
    bytes = i;
    return cells;
}

// The formatter behind printf().  Strings and characters go to the sink as
// they are; each number is rendered by snprintf, without its width, into a
// small buffer on this stack, and padded here.
size_t jc8012t_term_vformat(const jc8012t_fmt_sink &sink, const char *fmt, va_list ap) {
    size_t total = 0;
    auto emit = [&](const char *s, size_t n) { sink.bytes(sink.ctx, s, n); total += n; };
    auto pad = [&](int n, char c) {
        for (int i = 0; i < n; i++) sink.bytes(sink.ctx, &c, 1);
        if (n > 0) total += n;
    };

    while (*fmt) {
        if (*fmt != '%') {
            const char *s = fmt;
            while (*fmt && *fmt != '%') fmt++;
            emit(s, fmt - s);
            continue;
        }
        const char *spec = fmt++;
        if (*fmt == '%') { emit("%", 1); fmt++; continue; }

        // flags, width, precision, length
        bool left = false, zero = false;
        char flags[5]; int nf = 0;
        for (;; fmt++) {
            char c = *fmt;
            if (c == '-') left = true;
            else if (c == '0') zero = true;
            else if (c == '+' || c == ' ' || c == '#') { if (nf < 4) flags[nf++] = c; }
            else break;
        }
        flags[nf] = 0;
        int width = 0;
        if (*fmt == '*') {
            width = va_arg(ap, int);
            if (width < 0) { left = true; width = -width; }
            fmt++;
        } else {
            while (*fmt >= '0' && *fmt <= '9') width = width * 10 + (*fmt++ - '0');
        }
        int prec = -1;
        if (*fmt == '.') {
            fmt++;
            prec = 0;
            if (*fmt == '*') { prec = va_arg(ap, int); if (prec < 0) prec = -1; fmt++; }
            else while (*fmt >= '0' && *fmt <= '9') prec = prec * 10 + (*fmt++ - '0');
        }
        char len[3] = {0, 0, 0};
        if (*fmt == 'h' || *fmt == 'l') {
            len[0] = *fmt++;
            if (*fmt == len[0]) len[1] = *fmt++;
        } else if (*fmt == 'j' || *fmt == 'z' || *fmt == 't' || *fmt == 'L') {
            len[0] = *fmt++;
        }
        char conv = *fmt;
        if (!conv) break;               // a '%' cut off by the end: dropped, as newlib does
        fmt++;

        if (conv == 's') {
            if (len[0] == 'l') {
                const wchar_t *ws = va_arg(ap, const wchar_t *);
                if (!ws) ws = L"(null)";
                int n = 0, cells = 0;
                while (ws[n] && (prec < 0 || n < prec)) {
                    uint32_t cp = (uint32_t)ws[n++];
                    cells += (cp <= 0xFFFF && sink.wide(sink.ctx, (uint16_t)cp)) ? 2 : 1;
                }
                if (!left) pad(width - cells, ' ');
                for (int i = 0; i < n; i++) { sink.codepoint(sink.ctx, (uint32_t)ws[i]); total++; }
                if (left) pad(width - cells, ' ');
            } else {
                const char *s = va_arg(ap, const char *);
                if (!s) s = "(null)";
                int bytes, cells = 0;
                if (width > 0 || prec >= 0) cells = utf8Cells(s, prec, bytes, sink);
                else bytes = (int)strlen(s);
                if (!left) pad(width - cells, ' ');
                emit(s, bytes);
                if (left) pad(width - cells, ' ');
            }
            continue;
        }
        if (conv == '%') {              // "%5%": a padded '%', as newlib does
            if (!left) pad(width - 1, ' ');
            emit("%", 1);
            if (left) pad(width - 1, ' ');
            continue;
        }
        if (conv == 'c') {
            int c = va_arg(ap, int);
            bool w2 = len[0] == 'l' && (uint32_t)c <= 0xFFFF && sink.wide(sink.ctx, (uint16_t)c);
            int cells = w2 ? 2 : 1;
            if (!left) pad(width - cells, ' ');
            if (len[0] == 'l') { sink.codepoint(sink.ctx, (uint32_t)c); total++; }
            else { char ch = (char)c; emit(&ch, 1); }
            if (left) pad(width - cells, ' ');
            continue;
        }
        if (conv == 'n') {             // accepted and ignored
            (void)va_arg(ap, void *);
            continue;
        }

        // Numbers: snprintf one value, without the width, then pad here.
        char sub[16];
        int k = 0;
        sub[k++] = '%';
        for (int i = 0; flags[i]; i++) sub[k++] = flags[i];
        if (prec >= 0) k += snprintf(sub + k, sizeof(sub) - k - 4, ".%d", prec > 99 ? 99 : prec);
        for (int i = 0; len[i]; i++) sub[k++] = len[i];
        sub[k++] = conv;
        sub[k] = 0;

        char nb[JC8012T_PRINTF_NUMBUF];
        int n = -1;
        bool isInt = false;
        switch (conv) {
            case 'd': case 'i':
                isInt = true;
                if (len[0] == 'l' && len[1] == 'l') n = snprintf(nb, sizeof nb, sub, va_arg(ap, long long));
                else if (len[0] == 'l')             n = snprintf(nb, sizeof nb, sub, va_arg(ap, long));
                else if (len[0] == 'j')             n = snprintf(nb, sizeof nb, sub, va_arg(ap, intmax_t));
                else if (len[0] == 'z')             n = snprintf(nb, sizeof nb, sub, va_arg(ap, ssize_t));
                else if (len[0] == 't')             n = snprintf(nb, sizeof nb, sub, va_arg(ap, ptrdiff_t));
                else                                n = snprintf(nb, sizeof nb, sub, va_arg(ap, int));
                break;
            case 'u': case 'o': case 'x': case 'X':
                isInt = true;
                if (len[0] == 'l' && len[1] == 'l') n = snprintf(nb, sizeof nb, sub, va_arg(ap, unsigned long long));
                else if (len[0] == 'l')             n = snprintf(nb, sizeof nb, sub, va_arg(ap, unsigned long));
                else if (len[0] == 'j')             n = snprintf(nb, sizeof nb, sub, va_arg(ap, uintmax_t));
                else if (len[0] == 'z')             n = snprintf(nb, sizeof nb, sub, va_arg(ap, size_t));
                else if (len[0] == 't')             n = snprintf(nb, sizeof nb, sub, va_arg(ap, ptrdiff_t));
                else                                n = snprintf(nb, sizeof nb, sub, va_arg(ap, unsigned int));
                break;
            case 'f': case 'F': case 'e': case 'E': case 'g': case 'G': case 'a': case 'A':
                if (len[0] == 'L') n = snprintf(nb, sizeof nb, sub, va_arg(ap, long double));
                else               n = snprintf(nb, sizeof nb, sub, va_arg(ap, double));
                break;
            case 'p':
                n = snprintf(nb, sizeof nb, sub, va_arg(ap, void *));
                break;
            default:                    // unknown: print the spec as written
                emit(spec, fmt - spec);
                continue;
        }
        if (n < 0) continue;
        if (n > (int)sizeof nb - 1) n = sizeof nb - 1;   // cut short; see JC8012T_PRINTF_NUMBUF

        int padn = width - n;
        if (padn <= 0) { emit(nb, n); continue; }
        if (left) { emit(nb, n); pad(padn, ' '); continue; }
        // Zero padding goes after any sign and 0x prefix, and not for
        // integers with a precision (as C does) or for inf / nan.
        int pre = 0;
        if (nb[0] == '+' || nb[0] == '-' || nb[0] == ' ') pre = 1;
        if (nb[pre] == '0' && (nb[pre + 1] == 'x' || nb[pre + 1] == 'X')) pre += 2;
        char c0 = nb[pre];
        bool special = c0 == 'i' || c0 == 'I' || c0 == 'n' || c0 == 'N';   // inf, nan
        if (zero && !special && !(isInt && prec >= 0)) {
            emit(nb, pre);
            pad(padn, '0');
            emit(nb + pre, n - pre);
        } else {
            pad(padn, ' ');
            emit(nb, n);
        }
    }
    return total;
}


// ═══════════════════════════════════════════════════════════════════════════
// Task side: once per frame, scroll a step, blink the cursor, and put the
// result on screen -- a new scan layout (direct) or a PPA compose (PPA).
// ═══════════════════════════════════════════════════════════════════════════

void chipguy_JC8012P4A1C_terminal::_taskEntry(void *arg) {
    ((chipguy_JC8012P4A1C_terminal *)arg)->_taskLoop();
}

void chipguy_JC8012P4A1C_terminal::_taskLoop() {
    bool composed = false;
    for (;;) {
        // Direct mode, and PPA mode with nothing composed last time round:
        // once a frame.  Right after a PPA compose, go again at once -- the
        // compose took longer than a frame, and _compose() itself waits if
        // no framebuffer is free yet.
        if (!composed) _disp->waitVsync(100);
        xSemaphoreTake(_draw, portMAX_DELAY);
        // The layout set last time round has been applied at the frame
        // boundary just passed (PPA mode: composed from it already).
        _vtopLive = _vtop;

        // Scroll toward the target -- but never past rows not yet cleared.
        // Smooth mode moves a quarter of the way each time round (at least
        // MIN_STEP rows), but no faster than _smoothLimit lines a second, so
        // text pouring in is still seen to scroll.  The time since the last
        // time round is a frame in direct mode, a compose in PPA mode.
        int64_t nowUs = esp_timer_get_time();
        int64_t dtUs = nowUs - _lastLoopUs;
        _lastLoopUs = nowUs;
        if (dtUs > 100000) dtUs = 100000;
        bool moved = false;
        int32_t target = min(_vtopT, _clearedTo - (int32_t)_VSA);
        if (_vtop != target) {
            int32_t d = target - _vtop, a = d < 0 ? -d : d;
            int32_t step = a;
            if (_scrollMode != JC8012T_SCROLL_INSTANT) {
                step = max(MIN_STEP, a / 4);
                if (_smoothLimit) {
                    int32_t lim = (int32_t)((int64_t)_smoothLimit * _CH * dtUs / 1000000);
                    step = min(step, max(MIN_STEP, lim));
                }
            }
            if (step > a) step = a;
            _vtop += d < 0 ? -step : step;
            _scrolled += step;
            moved = true;
        }

        if (!_busy && _cursorStyle != JC8012T_CURSOR_OFF &&
            (int32_t)(xTaskGetTickCount() - _blinkAt) >= 0) {
            _blinkAt = xTaskGetTickCount() + BLINK_TICKS;
            if (_csShown) { _liftCursor(); _blinkOn = false; }
            else          { _blinkOn = true; _showCursor(); }
            _commit();
        }

        // PPA mode: the compose runs WITHOUT _draw, from a snapshot of the
        // layout, so callers keep printing while the PPA works.  That is
        // safe because the PPA reads only the window at _vtop, which only
        // this task moves, and _ensureReady() never clears a row of it.  A
        // caller drawing into the window meanwhile sets _dirty again, and
        // the next compose picks it up.
        composed = false;
        ComposeSnap snap;
        if (moved || _dirty) {
            if (_ppa) {
                snap = { _vtop, _R, _fixT, _fixB, _VSA };
                _composing = true;
                composed = true;
            } else {
                _applyScan();
            }
            _dirty = false;
        }

        xSemaphoreGive(_draw);
        if (composed) {
            _compose(snap);
            xSemaphoreTake(_draw, portMAX_DELAY);
            _composing = false;
            xSemaphoreGive(_draw);
        }
        xSemaphoreGive(_prog);
    }
}

// Direct mode: tell the display DMA where the window is.  Applied at the next
// frame boundary.
void chipguy_JC8012P4A1C_terminal::_applyScan() {
    if (_rot == 2) {
        // Stored upside down: the native top band is the screen's bottom one,
        // and the window runs through the ring backwards.
        _disp->setScanLayout(_fixB, _R, _VSA, pmod(-_vtop - _VSA, _R));
    } else {
        _disp->setScanLayout(_fixT, _R, _VSA, pmod(_vtop, _R));
    }
}

// PPA mode: header, window and footer from the surface into a framebuffer
// that isn't on screen, rotated and scaled; then flip to it.
//
// A framebuffer that was on screen is scanned until the next frame boundary
// after the flip away from it, so it is free once the frame count has moved
// past the count taken at that flip (_retiredAt).  With three framebuffers
// one is nearly always free, so a compose starts as soon as the last ends.
void chipguy_JC8012P4A1C_terminal::_compose(const ComposeSnap &v) {
    int64_t t0 = esp_timer_get_time();
    uint8_t n = _disp->getNumFramebuffers(), back = 0xFF;
    for (;;) {
        uint32_t now = _disp->getFrameCount();
        for (uint8_t i = 0; i < n; i++) {
            if (i == _front || (int32_t)(now - _retiredAt[i]) <= 0) continue;
            if (back == 0xFF || (int32_t)(_retiredAt[i] - _retiredAt[back]) < 0) back = i;
        }
        if (back != 0xFF) break;
        _disp->waitVsync(50);
    }
    uint16_t *fb = _disp->getFramebuffer(back);
    if (!fb) return;
    ppa_srm_rotation_angle_t angle;
    switch (_rot) {                       // clockwise in, PPA counter-clockwise
        case 1:  angle = PPA_SRM_ROTATION_ANGLE_270; break;
        case 2:  angle = PPA_SRM_ROTATION_ANGLE_180; break;
        case 3:  angle = PPA_SRM_ROTATION_ANGLE_90;  break;
        default: angle = PPA_SRM_ROTATION_ANGLE_0;   break;
    }
    const float s = (float)_num / _den;
    const uint32_t surfRows = 2 * (uint32_t)_H + 2 * (uint32_t)_SP;
    // The screen's area on the panel: all of it, or 780 x 1235 centred.
    const int32_t rw = _rw, rh = _rh;

    auto blit = [&](int32_t srcRow, int32_t h, int32_t y) {
        if (h <= 0) return;
        int32_t y0 = y * _num / _den, hs = h * _num / _den, ox, oy;
        switch (_rot) {                   // where that band lands on the panel
            case 1:  ox = rw - y0 - hs; oy = 0; break;
            case 2:  ox = 0; oy = rh - y0 - hs; break;
            case 3:  ox = y0; oy = 0; break;
            default: ox = 0; oy = y0; break;
        }
        ox += _ox;
        oy += _oy;
        ppa_srm_oper_config_t c = {};
        c.in.buffer = _store;
        c.in.pic_w = _W;
        c.in.pic_h = surfRows;
        c.in.block_w = _W;
        c.in.block_h = h;
        c.in.block_offset_x = 0;
        c.in.block_offset_y = srcRow;
        c.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
        c.out.buffer = fb;
        c.out.buffer_size = (size_t)JC8012_NATIVE_WIDTH * JC8012_NATIVE_HEIGHT * sizeof(uint16_t);
        c.out.pic_w = JC8012_NATIVE_WIDTH;
        c.out.pic_h = JC8012_NATIVE_HEIGHT;
        c.out.block_offset_x = ox;
        c.out.block_offset_y = oy;
        c.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
        c.rotation_angle = angle;
        c.scale_x = s;
        c.scale_y = s;
        c.mode = PPA_TRANS_MODE_BLOCKING;
        esp_err_t e = ppa_do_scale_rotate_mirror(_ppaClient, &c);
        if (e != ESP_OK) ESP_LOGE(TAG, "PPA: %s", esp_err_to_name(e));
    };
    blit(0, v.fixT, 0);
    blit(v.fixT + pmod(v.vtop, v.R), v.VSA, v.fixT);      // contiguous: the ring is stored twice
    blit(v.fixT + 2 * v.R, v.fixB, v.fixT + v.VSA);

    // Flip, THEN read the frame count: the frame-done interrupt counts the
    // frame before it reads which framebuffer to show next, so the count
    // taken here is never too low.
    _disp->setActiveFramebuffer(back, false);
    _retiredAt[_front] = _disp->getFrameCount();
    _front = back;
    _composeUs = (uint32_t)(esp_timer_get_time() - t0);
    _composeN++;
}

#endif // ARDUINO_ESP32P4_DEV
