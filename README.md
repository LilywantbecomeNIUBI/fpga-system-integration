# fpga-system-integration

两轮差速驱动 + 万向轮的 FPGA 系统集成工程。阶段目标是完成系统动态验证；不依赖车体自平衡。

当前接线采用ADC接口复用版：D157B用P7-21~27及36/37。先拔下旧ADC数字连接、释放载板雷达TTL接口；旧无刷电机和UVW线可保留，所有旧驱动EN/PWM/CAL由新顶层固定低，建议断开旧驱动动力支路。此前复用P7-1~4/13~16的接线表已作废。详见[当前引脚表](docs/pin_plan_v0.md)。

底盘：两台 WHEELTEC MG513Xp28-12V 有刷减速电机、AT8236 D157B 双路驱动（带稳压、电机 6Pin 插座）、AB 相编码器。用户于 2026-10-29 确认驱动板为 D157B；电机完整规格及实际 PCB 修订版待确认。FPGA 暂按现有 ACG720 / GW5AT-LV138PG484AC1/I0（GW5AT-138B）。

第一阶段统一顶层已接入D157B双轮控制、摄像头/YOLO反馈/LK双目标图传及1kHz IMU。Nano网关独占FPGA串口并接STL-27L，PC状态台提供显式遥控和感知观测。音频留待第二阶段。本车动态验收尚未进行；详细离线结论见[系统验证记录](docs/system_validation.md)。

默认 `start_console.cmd` 打开离线页面；`python tools/test_system.py` 执行可移植回归，`python tools/test_full_top.py` 执行本机完整数字顶层仿真，`python tools/build_system.py` 构建统一工程，`python tools/prepare_nano.py` 只准备代码包。这些命令不会部署Nano、下载板卡或启动电机。实机配置与启动见[系统运行说明](docs/system_runbook.md)。

- [引脚分配与接线条件](docs/pin_plan_v0.md)
- [完整引脚清单](hardware/pins.csv)
- [系统集成与动态验收计划](docs/integration_and_acceptance.md)
- [资料依据与待确认项](docs/sources.md)
- [电机FPGA实现与串口协议](docs/motor_v1.md)
- [系统运行说明与动态验收](docs/system_runbook.md)
- `fpga/constraints/chassis_candidate.cst`：底盘约束片段，满足接线条件后供后续顶层使用，当前不可直接下载。

运行 `python tools/audit_pins.py` 检查引脚唯一性、P7 对应关系、候选 CST 和预留接口冲突；此检查不能证明电气兼容、时序通过或动态验收。

本仓库独立维护系统顶层、接口约定、集成快照与验收证据。既有 fpga-robot、fpga-vision、fpga-sensor、fpga-audio 和 fpga-balance 继续作为来源参考；原无刷工程保留。后续移入模块应记录实际文件哈希、来源提交和工作树差异。
