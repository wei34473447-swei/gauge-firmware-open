# Community Gauge Firmware

面向 Waveshare ESP32-S3-Touch-LCD-1.85 V1（360×360、16 MB Flash、8 MB PSRAM）的仪表固件源码。

基于 **v1.0.44 r2** 的仪表固件源码，保留上游作者署名，车型配置范围见[车型清单](docs/VEHICLES.md)。品牌图片、二维码和未获公开再分发许可的字体使用中性素材替换。

## 包含内容

- 显示、触摸、背光、IMU 和板级驱动。
- BLE ELM327 连接、通用 OBD-II、上游车型读取与字段设置。
- 数字、指针、趋势、四象限、雷达演示等仪表页面、设置与多表通信。
- Wi-Fi 本地固件更新、回退及设备识别。
- 构建配置、分区表、所需组件、中性素材及其生成脚本、修改版 USB 安装脚本。

## 不包含内容

- 独立 App、小程序和后台业务服务的源码。
- 原产品 Logo、商标素材、二维码、商业字体或游戏角色素材。
- 真实设备 Key、签名私钥、客户或设备数据库、账号凭据和生产配置。
- 全国地图数据、真实车辆原始记录、用户位置、设备备份或开发机日志。

## 构建与安装

使用 **ESP-IDF v5.5.3** 和 ESP32-S3 工具链，见 [BUILD.md](BUILD.md)。开源版本为 `v1.0.44`。

车辆字段是否可读取，取决于实际车型、年款和适配器；已有读取实现不代表所有车辆均完成实测。

## 来源与许可证

本项目基于 [steveEcode/obd_brz_gauge](https://github.com/steveEcode/obd_brz_gauge)，其上游包括 [zhaizhaitao/open_obd_dsp](https://github.com/zhaizhaitao/open_obd_dsp)。原有版权声明及独立组件许可证保留，见 [LICENSE](LICENSE)、[NOTICE.md](NOTICE.md) 和 [PROVENANCE.md](PROVENANCE.md)。

本项目按 GPLv3 开放，欢迎使用、修改、构建和分发，可自行编译安装。配套线上服务及用户数据独立管理，不随源码提供。

单独许可的 SDK 组件、字体等继续遵循各自许可证。
