# frame-read-pipeline-20261103

**离线整机实验版本，未通过整机时序验收，未下载到板子。**

- 顶层 `system_task4_top`，360 点雷达、MPU6500/6050、图传、光流、导航与网口状态上传。
- 全点雷达实现保留在 `source/`；本冻结构建选择 360 点。
- 工具 setup 统计 550，hold 0；MPW -0.844 ns，PR1014 仍存在。
- 冻结源码完整顶层仿真通过；仿真强制 PHY 初始化并注入视频，不代表实板启动通过。
- 本整机有电机命令接收入口，输出并非静态固定 GND；仿真检查了未 ARM 时输出低。本次没有发送控制命令或进行下载。

独立 Gowin V1.9.12.04 工程：`sensor.gprj`；构建脚本：`build.tcl`，只构建不下载。
原位流、时序详报、综合网表分别无损压缩为 `firmware.fs.gz`、`sensor.tr.gz`、`sensor.vg.gz`。
可用 Python `gzip.decompress` 解压，原文件哈希见 `artifacts.json`，包内文件哈希见 `SHA256.json`。
原位流 SHA256：`4136ca0bdae7aab54f010c21470890ce47fb3f5dd3ec7cfdd406d80fab831d3f`。重新构建可能获得不同版图，必须重新验收。

`source/` 是当时实际编译的完整输入。目录内 `.gitattributes` 禁止 Git 换行转换，保持哈希一致。
`fixtures/` 保存冻结仿真台所需输入和模型；原台中的 `build/lidar_raw_ethernet_20261102` 相对路径保留。
工作树中的格式清理（如有）单独记录，不修改冻结编译输入。

稳定传感器恢复版本：`lidar360-mpu6500-20261102`。它只提供 360 点雷达和状态采集，不含图传/光流。
详细改动、已验证范围和残余问题见 `REVIEW.md`。
