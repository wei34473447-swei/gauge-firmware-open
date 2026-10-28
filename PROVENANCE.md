# Source provenance and public-variant changes

This repository is a maintained derivative, not an independently rewritten firmware.

- Immediate upstream: https://github.com/steveEcode/obd_brz_gauge
- Upstream project acknowledged there: https://github.com/zhaizhaitao/open_obd_dsp
- Board and SDK: Waveshare ESP32-S3-Touch-LCD-1.85 V1; Espressif ESP-IDF v5.5.3.
- GUI library: LVGL v8.4.0, with the local subset and font notices in `components/lvgl__lvgl/`.

The functional source baseline is v1.0.44, imu-console-r2, with vehicle profiles limited to the upstream configuration set listed in docs/VEHICLES.md. Product artwork and non-redistributable fonts are replaced with neutral, reproducible assets. Standalone build/install documentation and a locally declared LVGL component are included.

Original source copyright and SPDX notices are retained. Third-party code and font data keep their own licenses. The project license does not relicense separately licensed dependencies or grant product trademark rights.

No private repository history, original product artwork, client applications, backend business service, customer database, signing private key, device Key, production account configuration, device backup or private test log is included.

The presence of a vehicle reader is not proof of physical acceptance on every model year or adapter.
