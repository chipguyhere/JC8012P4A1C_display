# extras

Tools and license files that support the library but are not compiled into it.

- **`fontgen/`** — `make_terminal_fonts.py`, which builds the text terminal's fonts
  (`src/jc8012t_font_mono10.cpp` and the opt-in `jc8012t_font_mono10_*.h` headers) from
  Inconsolata, DejaVu Sans Mono, Font Awesome Free and Noto Sans CJK, together with
  those fonts' license texts. The fonts in `src/` are rendered from them and stay under
  their licenses. The script comes from `chipguy_ESP32-CYD-Resistive_display` by way of
  `chipguy_P486Panel_display`, with this library's `jc8012t_` names and a 10 × 20 cell
  (`--cell-width 10 --cell-height 20 --baseline 16`, the defaults here). Chinese,
  Japanese-kanji and Korean fonts can be built with it too.
- **`bitmaps/`** — `make_header_footer.py`, which renders the TerminalHeaderFooter
  example's header and footer images (`examples/TerminalHeaderFooter/bitmaps.h`). Needs
  Pillow and the DejaVu Sans font files.
