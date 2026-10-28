# 44 中性素材与字体生成

仅替换真实44的资源，保留44业务代码与四字段phase1_asset_t。原始几何由本工具绘制，无品牌Logo、二维码或游戏截图。phase1/preview5输出44的5字节RLE；boot保持grid240×240、15fps、37帧、2467ms、delta_varint_rgb565_black_v1。

正常固件构建使用已提交的资源，不运行此工具。离线重生需Pillow==12.2.0、fonttools==4.62.1；字体转换器使用package.json固定lv_font_conv==1.5.3。所用Source Han原始字体及完整OFL位于fonts/。

- `python3 tools/generate_public_assets/generate.py --rle-and-boot-only`：只生成两RLE资源和44规格启动流。
- `python3 tools/generate_public_assets/generate.py --assets-only`：另生成六份中性LVGL图片。
- `python3 tools/generate_public_assets/generate.py --fonts-only`：只重生font-replacements.json列出的11份OFL替代字体。
- 默认：生成素材和这11份替代字体。不会重生其余20份44原OFL字库。

font-subsets.json保留全部31份真实44字库的CMap范围；font-replacements.json仅选择原WenYue及FontTypoder的11份。旧C符号用于ABI，不代表使用旧商业字体。保留20份44原OFL字库的字形/描述符不变，生成头仅移除开发机路径。替换字体的形状和部分指标会改变，需独立检查44页面排版。

素材与生成代码GPL-3.0-or-later；Adobe字体及生成子集OFL-1.1；Zen Old Mincho与LVGL内置字体遵循各自保留许可。此工具不生成设备Key，不访问网络或私人字体，不包含73的UI、地图、授权或业务模块。
