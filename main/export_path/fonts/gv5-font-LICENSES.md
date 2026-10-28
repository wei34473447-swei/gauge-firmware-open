# 44 社区版字体与许可

固件保留真实44r2的31份字体符号、字号和字符范围。字体数据使用SIL Open Font License 1.1；该许可与固件代码GPLv3分别适用，不授予原商业字体的再分发权。

- 保留44的20份OFL字库：中文/中文补充/单位来自Adobe Source Han Sans；日文/日文补充/数字来自Zen Old Mincho（部分单位由Source Han补充）。对应完整许可为fonts/gv5-SourceHanSans-LICENSE.txt和fonts/gv5-ZenOldMincho-OFL.txt。字形、描述符与原44一致，只清理生成头中的开发机路径。
- 原WenYue英文和FontTypoder字体的11份C字库，替换为Adobe Source Han Sans SC Medium的OFL中性子集。具体文件见tools/generate_public_assets/font-replacements.json；真实44字符范围见font-subsets.json。旧C符号只保留ABI，不代表仍使用旧字体。
- Adobe源字体版权2014–2025 Adobe，完整原始字体及OFL位于tools/generate_public_assets/fonts/。本包不包含WenYue或Typodermic原始字体。
- LVGL内置Montserrat/Font Awesome保留原始独立许可。此清单不重新许可第三方组件。

商业字体替换会改变字形与部分指标，不能宣称与旧字体像素一致。不要将73新增字符、字体选择器或UI模块并入44。
