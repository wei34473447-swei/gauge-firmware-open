# Local LVGL source changes

Base: LVGL 8.4.0. These two expression corrections are already applied in `src/font/lv_font_fmt_txt.c`; the patch is recorded at `patches/0001-font-decoder-bounds-and-mask.patch`. This is a local firmware-export patch, not a claim that the upstream release contains the fix.

- `get_bits`: read a second byte only when the requested bit range crosses the boundary, rather than also when it ends exactly on the boundary. This removes an unnecessary final-byte overread.
- `bits_write`: clear only the target bit mask before inserting the new value. This avoids left-shifting a negative promoted complement and preserves the non-target bits of a prefilled destination byte.

Original file SHA-256: 310ec9757ce6e5196a9246b4aa7d8e6a0209c89dc8c07c34f6209efdc7443a72.
Patched file SHA-256: 2653a673bca7e7bca987f0d0853d082c3395e699ec411c63d6b18735d84d40d2.

The original LVGL MIT license and source headers remain unchanged. The public fonts retain 4bpp coverage, metrics and decoded pixels; lossless compression changes storage representation only. The native decoder uses shared decompression memory, so compression is not advertised as zero RAM or zero CPU overhead. Host sanitizer and raster checks do not replace hardware performance acceptance.
