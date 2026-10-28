# 构建、修改与安装

## 环境

- Waveshare ESP32-S3-Touch-LCD-1.85 **V1**。B/C 或其他修订的引脚不能直接照用。
- ESP-IDF **v5.5.3**；按 Espressif 官方说明安装 ESP32-S3 工具链并加载 `export.sh`。
- 所需固件组件源码已随仓库提供。LVGL 为明确标识的本地裁剪副本；不包含未用演示中的字体、照片等素材。

正常构建不需要商业字体、私有服务、真实设备 Key、App 或私有仓库。

```sh
idf.py set-target esp32s3
idf.py build
```

应用输出为 `build/obd_brz_gauge.bin`。构建时须核实应用未超过 `partitions.csv` 中的 OTA 应用槽；不要仅因为编译完成就向现有设备写入。

## 安装自己的修改版

这不需要官方小程序或官方服务授权。保留现有设备的备份与设置；先核实芯片、板型、当前分区、MAC 和活动应用已 VALID。

`tools/install_app.py` 是可审阅的独立 USB 安装脚本，依赖 `esptool==5.4.0`。它只在当前分区表与本次构建引用表一致时写非活动应用槽和另一个 OTA selector 扇区，并校验安装器未改 NVS。相同 selector 序号、序号耗尽或未确认的活动应用会拒绝安装。脚本与备份留在你本地；备份可能含你自己的设备数据，不要上传。

```sh
python3 -m pip install esptool==5.4.0
python3 tools/install_app.py \
  --port YOUR_USB_PORT \
  --expected-mac YOUR_DEVICE_MAC \
  --firmware build/obd_brz_gauge.bin \
  --sha256 YOUR_BUILT_APP_SHA256 \
  --partition-table build/partition_table/partition-table.bin \
  --output-dir local-backups/your-install
```

该脚本不是空白新板初始化器。只有确认是空白开发板、分区和电气配置正确时，才使用 ESP-IDF 的完整初始化流程。现有产品升级不能默认用整片擦除或完整刷写代替。

写入校验、首次启动日志和应用 VALID 分别核验；结果未知时先检查原记录，不反复刷写或复位。实屏触摸、SD 热插拔、手机热点、BLE 及车辆验收不由构建成功推导。

## 素材与依赖

中性图片、启动流和字体子集的可编辑输入及重生方法见 [tools/generate_public_assets/README.md](tools/generate_public_assets/README.md)。正常构建使用已提交的生成源码，无需 Node 或字体生成环境。

修改构建依赖时核对每个组件的许可证及锁文件，不将本地裁剪的组件冒充为 registry 上逐字相同的完整包。
