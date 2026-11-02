# 帧缓存 Bank 读出流水与代码复核

本次只修改 `vision_frame_stream.v` 的读数据流水及对应控制延迟，保留此前的 Bank 写入、槽位分配、帧发布时序和包格式。没有下载新位流，也没有改写 Flash。

## 改造

旧路径：BSRAM 同步读出 → 跨 Bank 选择 → `cached_word`。

新路径：BSRAM 同步读出 → 各 Bank 的 `read_pixel_pipe[15:0]` → 跨 Bank 选择 → `cached_word`。

每个 Bank 新增 16 位保留寄存器，默认四组共 64 位。`read_bank_pipe` 同步延迟 Bank 选择；`fill_valid` 从两级扩为三级，新增 `fill_index2` 对齐行 RAM 写地址。`DRAIN` 继续等整个有效流水排空，才允许开始发包。载荷寄存器不加复位，复位后的旧数据由有效流水屏蔽。

每个有效图像行的预填充多一拍（125MHz 下 8ns），UDP 字节接口、分辨率、行头和像素内容保持不变。综合/复制带来的整机寄存器差异以最终物理报告为准，不能只用 RTL 新增 64 位代替整机统计。

## 功能验证

- `python tools/test_perception.py --events-only` 通过；复用哈希一致的光流数值参考，重跑图像、事件、UDP/FCS/IP 校验。
- 双槽位排队、队满整帧丢弃、指针回绕、槽位复用及字节停顿通过。
- 4×2、最小 2×1，以及 6×3 非二次幂图像通过，写快读慢的时钟组合也通过；6×3 情况下，Bank 边界位于一行中间，专门验证新增 Bank 选择延迟。
- 冻结源码完整顶层仿真通过：最近回波 360 点、视频/事件并发、七类 UDP 状态、未 ARM 时电机输出低、UART 空闲。

覆盖边界：完整顶层台强制 `camera.phy_init=1` 并注入视频源，验证的是整机传输并发，不是实物 PHY/相机初始化成功。实际帧缓存像素、跨 Bank 读出及行包内容由独立的 `tb_frame_payload`、`tb_frame_banks`、`tb_stream_udp` 覆盖。上述仿真不能代替上板后的 PHY 初始化、相机 DVP 和网口数据检查。

首次冻结目录 `build/lidar_ethernet_20261102/system/20261102T155202Z` 完成了完整顶层仿真；其布局布线为补充定向报告而停止，不能作为最终时序结果。正式物理试验目录为 `build/lidar_ethernet_20261102/system/20261102T155742Z`；两者 RTL 哈希完全一致，仅增加报告命令，因此后者复用前者的完整顶层仿真，记录于 `test_result.json`。

定向报告分别覆盖 64 个 Bank 输出寄存器 D 端和 16 个 `cached_word` D 端，由 `audit_frame_read_timing.py` 核查覆盖率和 setup 裕量。这些命令只增加报告，不增加 false path、multicycle 或放宽周期。语法依据：[Gowin 设计时序约束 SUG940](https://cdn.gowinsemi.com.cn/SUG940E.pdf)。

## 代码复核发现

### 1. 已复现：无补边模式错误

当 `CANVAS_HEIGHT == ACTIVE_HEIGHT`，`TOP=0`，后续行判断使用 `row >= TOP-1`。`row` 为无符号量，减一后的比较导致有效行被判成补边。4×2、画布高 2 的用例中，第 1 行首个像素应为 `0x1404`，实际低字节为补边色 `0x738e` 的 `0x8e`。

同一个用例在优化前后的源码中均复现，证据为正式试验目录的 `no_padding_bug_reproduction.json`。生产参数 320×180 放入 320×320 画布时 TOP=70，不触发这个问题。本次没有把该独立边界修复混入读流水改造；建议后续用“下一行 >= TOP 且 < TOP+ACTIVE_HEIGHT”的无减一条件统一判断。

复现命令（PowerShell 中应保留参数引号）：

```powershell
iverilog -g2012 -s tb_frame_banks '-Ptb_frame_banks.W=4' '-Ptb_frame_banks.H=2' '-Ptb_frame_banks.C=2' -o build/tests/frame_no_padding.vvp tests/vision/tb_frame_banks.v fpga/vision/src/vision/vision_frame_stream.v
vvp build/tests/frame_no_padding.vvp
```

这是已知失败用例，没有纳入普通通过回归；普通回归并不代表该参数组合已支持。

### 2. 约束缺口：帧槽位 CDC 尚缺专门预算

模块中 `wgray → ws0/ws1`、`rgray → rs0/rs1` 使用两级同步；`descriptors` 在帧完成后保持到槽位释放，依赖发布指针同步后的稳定窗口取数。代码结构符合“先写数据，后发布所有权”的意图，但生成的 SDC 对 cam_pclk/eth125 使用异步分组，没有为这里的 Gray 总线和描述符路径配置独立最大延时预算，也没有对这些同步器配置保留/异步寄存器属性。

另一个 `frames_to_control50/fifo` 已有 Gray 路径预算与物理审计，不能把那个模块的通过结果等同于本模块也通过 CDC 审计。这里应补充适合实际时钟周期、发布等待和接收窗口的独立约束及布局后审计；目前是待核实风险，不能据此认定曾发生亚稳态或导致网口失联。

### 3. 接口限制：帧边界与像素并拍需谨慎

`frame_commit` 与帧末检查使用当前 `written`，同拍 `pixel_valid` 的加一在下一拍才生效。因此 `frame_end` 若与最后一个有效像素同拍，会因少计一个像素而拒绝帧。当前上游 `vision_dvp_capture` 在 VSYNC 结束帧，像素和帧结束控制经过同深度流水，与当前测试分开结束的用法一致；修改上游接口时不能默认支持“最后像素即 EOF”的并拍协议。

### 4. 本次数据流水不解决的物理问题

先前整机报告仍有 eth125 高脉宽违规及 PCLK PR1014，并包含图像地址计数、桥接年龄计数和网络逻辑的 setup 违规。新增 Bank 输出寄存器只拆分读数据路径，不能把这一项的通过宣称为整机通过，也不能据此断言网口实板故障已修复。

## 本次物理结果

Gowin 正常完成布局布线及位流输出，耗时约 1159 秒。定向审计通过，**目标读出 setup 违规已消除**：

| 路径 | 覆盖端点 | 最差数据延时 | 最差 setup 裕量 |
|---|---:|---:|---:|
| 改造前 BSRAM → cached_word | — | 9.804 ns | −1.811 ns |
| 改造后 BSRAM → Bank read_pixel_pipe | 64/64 | 7.055 ns | +0.827 ns |
| 改造后 Bank read_pixel_pipe → cached_word | 16/16 | 4.019 ns | +3.671 ns |

两级数据延时应分别满足各自的 8ns 周期，不能将 7.055 与 4.019 相加后再与单周期比较。定向报告同时保留时钟偏斜、setup 要求和逐端点路径。证据为正式试验目录的 `frame_read_timing_audit.json`、`frame_read_netlist_audit.json` 及 `impl/pnr/sensor.tr`。

整机仍未通过时序验收：

| 指标 | 改造前 | 本次 |
|---|---:|---:|
| 全局 setup / hold 违规端点 | 881 / 0 | 550 / 0 |
| eth125 setup 违规端点 | 828 | 508 |
| 全局最差 setup 裕量 | −1.811 ns | −1.009 ns |
| cam_pclk / control50 setup 违规端点 | 0 / 0 | 0 / 0 |
| 最小脉宽裕量 | −0.849 ns | −0.844 ns |
| 寄存器 | 38915 | 38994 |
| BSRAM | 236/340 | 236/340 |

本次整机寄存器净增 79 个，约占器件寄存器总量 0.057 个百分点；逻辑数 48952→48955，DSP 不变。其差异包括新增数据/选择/有效/行索引流水及综合复制变化，不能仅归结为 64 位数据寄存器。

当前最差路径已转移到 `camera/runtime_network.net/action_3_s0/Q → send_state_0_s6/D`，数据延时 8.947 ns，裕量 −1.009 ns。eth125 最差 50 条抽样中，网络发送状态逻辑 22 条、flow_bridge 15 条、bridge 6 条、语义网络逻辑 4 条、stream 3 条；这不代表全部 508 个违规端点的完整分布。图像缓存内仍有其他路径需要检查，不应把本次读出目标通过等同于整个 stream 模块通过。

PR1014 和 eth125 RAM 高脉宽违规没有消除。新位流 SHA256 为 `4136ca0bdae7aab54f010c21470890ce47fb3f5dd3ec7cfdd406d80fab831d3f`，本次没有下载。

复核命令：

```powershell
python tools/task4/audit_frame_read_timing.py build/lidar_ethernet_20261102/system/20261102T155742Z
python tools/task4/report_route_trial.py build/lidar_ethernet_20261102/system/20261102T155742Z --baseline build/lidar_ethernet_20261102/system/20261102T134609Z
```
