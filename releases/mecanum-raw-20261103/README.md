# 已上板验证的整机 SRAM 位流

文件：`sensor.fs`；器件：GW5AT-LV138PG484AC1/I0，Device Version B。
构建：20261103T013936Z，顶层 system_mecanum_top。
包含麦轮 1kHz 控制、路由器 UDP 控制与遥测、雷达原始全量点、相机及光流。

使用 Gowin Programmer 的 SRAM Program 下载。此文件对应已验证的实验版本，不包含后续开发中的麦轮自主导航；本次上传不重新构建或下载，不提供 Flash 固化验收。

Setup/Hold 违例均为 0，但完整时序签核未通过：保留 camera_pclk_d 的 PR1014 普通时钟布线警告及 eth125 最小脉宽裕量 -0.849 ns。不能标为完整 STA 通过。

SHA256 见 manifest.json。上板及构建证据见 ../../docs/evidence/mecanum_1khz_20261103/，雷达恢复证据见 ../../docs/evidence/nano_lidar_fix_20261103/。
