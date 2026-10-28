# Firmware source subset of LVGL 8.4.0

This is a redistributable firmware-focused source subset, not a full upstream LVGL checkout. Core source, public API headers, ESP-IDF build support and the upstream MIT license are retained. Examples and demos are disabled in the public firmware configuration; their implementation and nonessential photographs, screenshots, raw fonts, scripts, tests, CMSIS packs and other-platform resources are omitted. The firmware does not call `lv_demo_*` or `lv_example_*`. Example/demo declaration headers remain for include compatibility; enabling those optional examples or demos is unsupported in this subset.

The unused SimSun, DejaVu and Unscii font implementations are excluded and their options remain disabled. Built-in Montserrat fonts and the Font Awesome glyphs embedded into those fonts are retained. The included font-source metadata identifies Montserrat 7.200 and Font Awesome 5.9.0; their original copyright/license notices are in `LICENSES/`. Those font glyphs remain under their font licenses, rather than being relicensed under the LVGL MIT license or the firmware GPL license. Font Awesome brand glyphs do not imply endorsement and must not be used as unrelated product logos.

Upstream source: https://github.com/lvgl/lvgl/tree/v8.4.0
Component version: 8.4.0. Original registry component hash: d7c1ac037ae6e85d94897f807d6e7ba0946a83e720074fc95a4f6241da9f9f53.

The component's original hash/checksum metadata remains attribution metadata; it does not assert that this pruned and license-supplemented tree is byte-identical to the complete registry download. Exact public-export hashes are recorded separately by the project release review. No source was downloaded or installed automatically to make this subset.

This subset is a local project component at `components/lvgl__lvgl`, explicitly required by the application. It is not a Component Manager registry dependency; the project lock retains only the other pinned registry dependencies and ESP-IDF. No machine-specific absolute local path is distributed in the lock.

Two local decoder expression corrections are documented in `PUBLIC_PATCHES.md`; original LVGL licensing remains in force.
