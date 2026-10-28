# 车型配置

沿用上游 `steveEcode/obd_brz_gauge` 的 17 个配置项（基线 `fcf4e4a8f73c59cffa67ff97124e15563e5617f6`）。包含通用配置和同车型的不同读取路径，不等于 17 款均已实车验证的车型。

| 配置 | 范围 |
|---|---|
| OBD2 Generic | 通用标准 OBD-II |
| ZN/C6 CAN | 丰田 GT86 / 斯巴鲁 BRZ 一代，CAN 与 OBD |
| ZN/C6 PID | 同平台，PID 读取 |
| ZD8 OBD | 斯巴鲁 BRZ 二代，标准 OBD |
| ZD8 | 同平台，标准 OBD |
| MX-5 ND | 马自达 MX-5 ND |
| BMW F/G | 宝马 F/G 平台 |
| Supra A90 | 丰田 Supra A90 |
| BMW G OBD | 宝马 G 平台 OBD 配置 |
| BMW E | 宝马 E 平台 |
| JCW F56 | MINI JCW F56 |
| MINI R55 | MINI R55 |
| POS 997.2 | 保时捷 987.2 / 997.2 |
| POS 997.1 | 保时捷 987.1 / 997.1 |
| GIULIA 2.0T | 阿尔法·罗密欧 Giulia 2.0T |
| jeep | Jeep 通用配置 |
| Honda Integra | 本田形格 / Integra 配置 |

具体请求、地址、公式和适用限制以 `main/app_obd_dsp/vehicle_profiles.c`、`vehicle_custom_config.h` 与 `main/bsp_obd_dsp/elm327_ble_client.c` 为准。配置中部分轮胎、齿比和年款适用范围仍为占位或待验证值；不能据此承诺整个品牌或平台的字段可读。
