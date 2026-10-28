# Notices and third-party licenses

The instrument firmware is a derivative of steveEcode/obd_brz_gauge, which acknowledges zhaizhaitao/open_obd_dsp. Keep the original author and copyright notices in the source. The project GPLv3 text is LICENSE. This notice does not replace individual file licenses.

## SDK and board support

ESP-IDF v5.5.3 is installed separately. Espressif's own SDK code is primarily Apache-2.0 and contains separately licensed third-party components. Follow its LICENSE and COPYRIGHT inventory; do not assume every SDK file has one license. The local private_bt folder contains SDK source replacements and their original notices; its name does not denote a private key store. The local HTTP server component retains its Apache license and provenance.

## LVGL and fonts

components/lvgl__lvgl is a clearly marked local subset of LVGL v8.4.0. Its code license, retained headers and PUBLIC_SUBSET documentation apply. It excludes unused demo photos, fonts, and packs. The retained Montserrat and Font Awesome font data have separate notices under that component's LICENSES folder.

The bundled Adobe Source Han Sans original font and the generated bitmap subsets use SIL OFL 1.1. Their license is tools/generate_public_assets/fonts/OFL.txt. The new neutral artwork and asset generator are GPL-3.0-or-later. See tools/generate_public_assets/NOTICE.md for source URLs, scope and regeneration inputs.

Legacy C symbol names are retained to preserve interfaces; their payloads have been replaced. Those names do not grant rights to the old fonts, logos, social accounts or game artwork. No product trademark rights are granted by the software license.

## Application and service boundary

This repository contains the v1.0.44 firmware-side communication and update implementations. Independent mobile applications, Mini Programs, backend business services, customer/device databases, account credentials and signing private keys are not included. A user can build and install the firmware independently.

## Vehicle and map data

Only data already embedded in this firmware is part of the source snapshot. This repository does not redistribute separately collected Car Scanner APKs, private vehicle captures, VIN logs, bulk community archives, or national map packages. Any external data or assets added later need their own source and license inventory.

Local LVGL 8.4.0 font-decoder corrections are already applied and recorded in `components/lvgl__lvgl/PUBLIC_PATCHES.md` with the minimal patch. Original MIT copyright/license notices are retained.
