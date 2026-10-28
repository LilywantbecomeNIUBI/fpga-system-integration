# fpga-system-integration

两轮差速驱动 + 万向轮的 FPGA 系统集成工程。阶段目标是完成系统动态验证；不依赖车体自平衡。

底盘：两台 WHEELTEC MG513Xp28-12V 有刷减速电机、AT8236 D157B 双路驱动（带稳压、电机 6Pin 插座）、AB 相编码器。用户于 2026-10-29 确认驱动板为 D157B；电机完整规格及实际 PCB 修订版待确认。FPGA 暂按现有 ACG720 / GW5AT-LV138PG484AC1/I0（GW5AT-138B）。

已增加独立的D157B双路有刷电机RTL、串口调试工具、仿真与Gowin构建入口。使用说明及验证边界见[电机V1](docs/motor_v1.md)和[验证记录](docs/motor_validation.md)。本车动态验收尚未进行，感知模块尚未接入该电机顶层。

- [引脚分配与接线条件](docs/pin_plan_v0.md)
- [完整引脚清单](hardware/pins.csv)
- [系统集成与动态验收计划](docs/integration_and_acceptance.md)
- [资料依据与待确认项](docs/sources.md)
- [电机FPGA实现与串口协议](docs/motor_v1.md)
- `fpga/constraints/chassis_candidate.cst`：底盘约束片段，满足接线条件后供后续顶层使用，当前不可直接下载。

运行 `python tools/audit_pins.py` 检查引脚唯一性、P7 对应关系、候选 CST 和预留接口冲突；此检查不能证明电气兼容、时序通过或动态验收。

本仓库独立维护系统顶层、接口约定、集成快照与验收证据。既有 fpga-robot、fpga-vision、fpga-sensor、fpga-audio 和 fpga-balance 继续作为来源参考；原无刷工程保留。后续移入模块应记录实际文件哈希、来源提交和工作树差异。
