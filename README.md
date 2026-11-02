# fpga-system-integration

2026-11-03 路由器控制开发：麦轮命令新增 UDP6206，电脑支持有线/Wi-Fi；使用 `start_mecanum_router.cmd`。需配套新固件及 PC 端点配置，Nano 图传端点保留，详见 [接线、状态端点与验证边界](docs/MECANUM_ROUTER_CONTROL_20261103.md)。本段为本分支开发状态，下方阶段记录保留原验收结论。

## 2026-11-03 阶段性主线

当前主线合并了**全点雷达麦轮整机、队友完整 IO 电气约束，以及 Nano 实时图传修复**。
FPGA Setup/Hold 为 0/0；IMU、串口控制、看门狗和架空短时前进已验证。Nano 默认保持只读的 `nano-video-20261103` 运行基线。
这是阶段性联调成果，仍保留 PCLK MPW/PR1014、运动期间 4 次可恢复 I²C NACK、雷达未接暂缓和落地运动精度未验等余项。

- [阶段性版本与验证边界](docs/MILESTONE_MECANUM_NANO_20261103.md)
- [麦轮实板记录与当前位流](docs/MECANUM_BOARD_20261103.md)
- [Nano 固定运行版本](docs/NANO_BASELINE.md) · [图传修复与交付帧率](docs/VIDEO_FIX_20261103.md)

整机构建入口：`python tools/task4/build_ethernet.py --system --mecanum --raw-points --clock-cleanup`。
历史双轮、360 点和全点源码/检查点均保留；下列旧日期记录作为历史说明。

**2026-11-02 调试代码快照：雷达72扇区双缓冲RAM版本已通过静态上板接收；IMU已按STM32参考工程补充MPU6500（WHO=0x70）支持，原转接板接线仍在地址阶段NACK。新增P7-31/F16、P7-32/E17直连静态诊断源码快照，仿真及布线通过，尚未下载或确认直连接线。具体改动、验证边界和接线见[STM32对照与直连说明](docs/task4/IMU_STM32_COMPARE_20261102.md)。本次保存也包含当前工作区的1kHz轮速、雷达全点网口和上位机修改，不代表整车动态验收通过。**

**2026-11-02 雷达显示/网口：已升级为解析器全点上传，记录回归每圈约2175点；界面默认全量，可选720/360显示上限和自动范围。七类状态经UDP6205上传，控制命令保留串口。12000点逐点回归及静态诊断版PnR通过（Setup/Hold=0/0，Logic12%、寄存器14%、BSRAM不到1%）。按用户要求仅离线交付，未下载；完整融合版仍待独立PnR和实板验收。见[全点方案与验证](docs/task4/LIDAR_RAW_ETHERNET_20261102.md)；[旧360点记录](docs/task4/LIDAR_ETHERNET_360_20261102.md)保留。**

**2026-11-02 MPU-6050：用户确认 RevB（99x99mm）、模块自带 AD0 下拉、外形不碰相邻元件，按原座八针直插，无需飞线。SCL/SDA 保持 F19/F20，地址 0x68；驱动改为最大400kHz、1kHz、14字节六轴/温度突发读取，网关同步适配身份和换算。数字验证及重新布局布线完成；IMU静态检查版时序通过，整机候选Setup/Hold=0/0但仍保留原eth125脉宽和时钟布线告警，尚未下载。见 [更换及上板测试说明](docs/IMU_MPU6050_20261102.md)。**

**2026-11-01：光流视频底图叠加与M2雷达扫描上传已完成代码、数字回归及综合；完整布线本轮超时，尚无新位流；新显示协议、M2网关配置、只读接收验证工具及实机边界见[扫描与光流显示说明](docs/task4/M2扫描与光流显示_20261101.md)。本轮用户选择离线，未下载或实机接收验证。**

**2026-10-31 最新：目标LK、环境光流/避障及帧RAM所有权已迁到内部control50。数字回归通过，保留Place 2（Setup/Hold=0/0、control50裕量+4.944ns）；SRAM上板300.817秒并发窗口未触发环境LK故障锁存，真实YOLO/目标LK带载、约30fps。4次原始不可用状态均正确停车屏蔽，电机未使能。MPW仍未决，动态整车和长期验证未签核。见[50MHz迁移与证据](docs/LK_CONTROL50_MIGRATION_20261031.md)。**

任务四的RPP/VFH+导航、雷达前端、M2接线及配套诊断台见[任务四说明](docs/task4/README.md)。该候选使用独立顶层与M2引脚映射，保持独立构建；当前尚未完成新接线的整车PnR、实车标定及动态验收。

**2026-10-31 上一轮LK专项（历史失败）：单份主状态、Gray关键边及寄存译码缓冲已实现，主状态最大直接扇出降至19；数字回归通过，但实板仍捕获27→18异常，300秒健康验收未通过。当时保持故障锁存和停车建议屏蔽。详见[本轮记录](docs/LK_FSM_HARDENING_20261031.md)。整体时序仍未签核。**

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
