# fpga-system-integration

**2026-10-30 感知合并：IMU、YOLO/多点LK、独立金字塔光流/避障观测已按模块边界接入。架构与复用接口见 [本轮合并说明](docs/PERCEPTION_MERGE_20261030.md)。轮速控制保留显式ARM/心跳；感知暂不触发自动运动。既有电机与IMU上板记录见 [当前交接](docs/MOTOR_HANDOFF_20261030.md) 和 [IMU记录](docs/MOTOR_IMU_20261030.md)。**

2026-10-31已抓到环境LK的IDLE/busy矛盾及WAIT_RAM(3)→IDLE(0)异常记录，恢复候选可返回无效完成并保留故障锁存。实板五分钟并发收帧29.99fps，光流持续推进；页面原约5fps和Nano积压已改善，检测画面改为独立连续流、约24.6fps。异常跳转的最初原因仍未查明，**整体感知验收未通过**。最新实验候选Setup/Hold=0/0、最差Setup 0.000ns；MPW −0.688ns未决，整体时序未签核。见[本轮诊断及证据](docs/FLOW_VIDEO_DIAG_20261031.md)；[首次上板记录](docs/BOARD_SENSING_20261031.md)保留为历史。本轮未测试控制部分。

两轮差速驱动 + 万向轮的 FPGA 系统集成工程。阶段目标是完成系统动态验证；不依赖车体自平衡。

当前接线采用已验证的编码器配置：D157B电机控制用P7-21~24，右编码器用P7-27/36，左编码器用P7-31/32（F16/E17），输入内部上拉。先拔下旧ADC数字连接、释放载板雷达TTL接口；旧无刷电机和UVW线可保留，所有旧驱动EN/PWM/CAL由新顶层固定低，建议断开旧驱动动力支路。此前复用P7-1~4/13~16及左轮P7-25/26的接线表已作废。详见[当前引脚表](docs/pin_plan_v0.md)。

底盘：两台 WHEELTEC MG513Xp28-12V 有刷减速电机、AT8236 D157B 双路驱动（带稳压、电机 6Pin 插座）、AB 相编码器。用户于 2026-10-29 确认驱动板为 D157B；电机完整规格及实际 PCB 修订版待确认。FPGA 暂按现有 ACG720 / GW5AT-LV138PG484AC1/I0（GW5AT-138B）。

第一阶段统一顶层已接入D157B双轮控制、摄像头/YOLO反馈/LK双目标图传、独立光流/避障观测及1kHz IMU。Nano网关独占FPGA串口并接STL-27L，PC状态台提供显式遥控和感知观测。音频留待第二阶段。独立电机带载前后转停已由用户确认；合并候选的感知并发动态验收待做；详细离线结论见[系统验证记录](docs/system_validation.md)。

默认 `start_console.cmd` 打开离线页面；`python tools/test_system.py` 执行可移植回归，`python tools/test_perception.py` 检查环境光流/事件合并，`python tools/test_full_top.py` 执行本机完整数字顶层仿真，`python tools/build_system.py` 构建统一工程，`python tools/prepare_nano.py` 只准备代码包。这些命令不会部署Nano、下载板卡或启动电机。实机配置与启动见[系统运行说明](docs/system_runbook.md)。

- [引脚分配与接线条件](docs/pin_plan_v0.md)
- [完整引脚清单](hardware/pins.csv)
- [系统集成与动态验收计划](docs/integration_and_acceptance.md)
- [资料依据与待确认项](docs/sources.md)
- [电机FPGA实现与串口协议](docs/motor_v1.md)
- [系统运行说明与动态验收](docs/system_runbook.md)
- `fpga/constraints/chassis_candidate.cst`：底盘约束片段，满足接线条件后供后续顶层使用，当前不可直接下载。

运行 `python tools/audit_pins.py` 检查引脚唯一性、P7 对应关系、候选 CST 和预留接口冲突；此检查不能证明电气兼容、时序通过或动态验收。

本仓库独立维护系统顶层、接口约定、集成快照与验收证据。既有 fpga-robot、fpga-vision、fpga-sensor、fpga-audio 和 fpga-balance 继续作为来源参考；原无刷工程保留。后续移入模块应记录实际文件哈希、来源提交和工作树差异。

### 有刷闭环专用调参（2026-10-29）

当前有刷独立/集成电机路径仅支持轮速 PI，已取消 `estop_n` / P7-37 外部许可依赖，不接 P7-29→37 跳线。保留上电锁定、显式 STOP、250ms FPGA 看门狗、200ms 网关失联停车、限幅和编码器保护。

运行 `python host/motor_tuner.py` 可打开默认离线的 `http://127.0.0.1:8874/`。实际连接时加 `--live --token-file <本机session_token.json>`，复用唯一网关而不打开 COM20。先更新/重启网关，再连接专用调参台；原视觉/IMU状态台保留并增加光流观测。用法见 `docs/motor_v1.md`。新固件下载与实际转动分开执行，独立电机/IMU的上板结果不能代替合并候选验收。
