# 整机 360 点离线布线优化试验

试验目录：`build/lidar_ethernet_20261102/system/20261102T134609Z`。
对照目录：`build/lidar_ethernet_20261102/system/20261102T123714Z`。
两者都是带相机、光流、导航、IMU、网口遥测的整机顶层；不是已上板的静态传感器版本。
比较两份 manifest 的源码哈希，唯一不同的 RTL 文件为 `vision_frame_stream.v`；其余为上述构建参数、物理约束和报告配置变化。
离线试验结束后，按用户后续指令将本次位流作为实验版下载到 SRAM；没有改写 Flash，没有改变已发布的两个 360 点标签，也没有删除全点雷达实现。

## 本次改动

- `build_ethernet.py` 默认启用资源复制，提供 `--replicate-resources 0/1`；布局/布线使用 `place=2, route=0`。
- `vision_frame_stream.v` 将双帧 RAM 按地址分成 4 个 Bank，每组独立寄存写使能、地址、像素。两级写入命令流水保持三者对齐；分配指针立即更新，跨域发布指针延迟至最后一次 RAM 写完成后。读 Bank 选择与同步读数据对齐，保持原来的两拍读取接口。
- `--clock-cleanup` 为试验选项：使用 `LOCAL_CLOCK` 排除有稳定 RTL 名字的复位/使能控制网络；用 `BUFG = CLK` 指定五条真实时钟。没有采用未经核对的 `syn_global_buffers` 属性，也没有硬编码综合生成的 `n8531_18` 等节点名。
- 补充无过滤的最差 setup 路径及五个时钟域的详细路径。保留已有 CDC 最大延时和一、二级同步器审计，不放宽时钟周期或最小脉宽。
- 修正构建结果对 CDC 时序审计的判断：必须检查 JSON 中的 `passed`，不能仅以脚本退出码为依据。
- `report_route_trial.py` 输出每域违规数、资源、时钟树分配和逐路径源/目的/延时。路径组统计明确标记为最差路径抽样，不能把 50 条抽样冒充全部 926 个违规端点。

`LOCAL_CLOCK` / `BUFG = CLK` 的语法依据：[Gowin 物理约束指南 SUG1018](https://cdn.gowinsemi.com.cn/SUG1018E.pdf)。
相机 PCLK 的 B1 输入在旧报告中已使用 PRIMARY，但仍有 PR1014，因此不能把释放时钟树资源等同于解决输入端专用走线限制。

## 验证

`python tools/test_perception.py --events-only` 通过；复用哈希一致的已验证光流数值参考，重跑事件、截止时间、帧所有权、图像字节、补边、背压、UDP/FCS/IP 校验测试。
新增 `tb_frame_banks.v` 验证两个排队帧、队满整帧丢弃、指针回绕、槽位复用及发送停顿；4×2 和最小 2×1 两种帧尺寸均通过。

综合后确认四组独立写使能均保留，每组直接连接 32 个 SDPB 的 CEA，总计 128 个图像 SDPB，与原来图像缓存的容量一致。详见试验目录的 `frame_bank_netlist_audit.json`。
控制时钟域及光流状态机综合网表审计均通过。布线延时必须以最终报告为准，未承诺降至 2ns。
完整冻结源码顶层仿真通过：原生雷达输入含错误 CRC，输出与独立最近回波参考一致，同时验证视频/事件并发、七类状态、电机未使能和 UART 空闲。证据为试验目录中的 `test_result.json`、`sector_simulation.log`。

## 复现

```powershell
python tools/task4/build_ethernet.py --system --sector360 --place 2 --route-option 0 --replicate-resources 1 --clock-cleanup --timeout-seconds 2400
python tools/task4/test_sector_system.py <本次构建目录>
python tools/task4/report_route_trial.py <本次构建目录> --baseline build/lidar_ethernet_20261102/system/20261102T123714Z
```

## 实测结果

Gowin 正常退出，综合、布局、布线、位流输出全部完成，共约 605 秒。时序验收仍为 **失败**；后续经用户明确要求下载实验版，不改变此验收结论：

| 指标 | 对照 | 本次 |
|---|---:|---:|
| 全局 setup / hold 违规端点 | 1095 / 0 | 881 / 0 |
| cam_pclk setup 违规 | 14 | 0 |
| cam_pclk 同域最差裕量 | −2.600 ns | +2.898 ns |
| control50 同域最差裕量 | +0.169 ns | +2.877 ns |
| sys50 setup 违规 | 152 | 49 |
| eth_rx125 setup 违规 | 3 | 4 |
| eth125 setup 违规 | 926 | 828 |
| PRIMARY / LW | 7/8、8/8 | 5/8、5/8 |
| 最小脉宽裕量 | −0.844 ns | −0.849 ns |
| BSRAM | 236/340 | 236/340 |
| LUT/ALU 逻辑数 | 48962 | 48952 |
| 寄存器数 | 38627 | 38915 |

新增 288 个寄存器，约占器件寄存器总量 0.21 个百分点；BRAM、DSP 用量不变。资源代价很小，但读出端及其他逻辑的时序仍需继续处理。

相机域最差路径现在为 Bank 0 的写使能寄存器到 CEA，数据延时 8.980 ns，其中布线 8.635 ns；虽然通过了 11.905 ns 的周期要求，仍没有达到“布线降到 2 ns”这个预期。PRIMARY 的五条网络全部是真时钟；部分指定 `LOCAL_CLOCK` 的复位网络仍列在 LW，不能声称控制网络已经全部退出时钟资源。

本次无过滤最差 setup 路径裕量 −1.811 ns：图像 RAM 输出到 `cached_word[1]`，数据延时 9.804 ns，属于 eth125 域。
eth125 最差 50 条路径中，目的模块分布为：

| 目的模块 | 本次最差 50 条中的数量 |
|---|---:|
| camera/flow_bridge | 25 |
| camera/bridge | 15 |
| camera/stream | 4 |
| camera/runtime_network.net | 4 |
| camera/runtime_network.semantics | 2 |

因此剩余问题并非仅在雷达或网络 FIFO：图像读出、桥接逻辑的年龄计数路径及网络逻辑均需要检查。以上是最差路径抽样，不是全部 828 个违规端点的完整分布。详细源、目的、延时及每域报告在 `routing_diagnosis.json` 和 `impl/pnr/sensor.tr` 中。

PR1014 仍指向 `camera_pclk_d`。最小脉宽违规仍来自 **eth125 的图像 RAM CLKB**，STA 计算高脉宽 0.051 ns，小于要求的 0.900 ns；释放 PRIMARY/LW 没有解决它。PCLK 输入告警与 eth125 的脉宽违规属于不同的时钟路径，不能直接认定前者导致后者。

CDC 物理延时审计通过，Gray 路径最大 1.513 ns，小于 10 ns 预算；总时序审计 `passed=false` 的原因是仍有同步 setup 违规。此次工具正常完成，不能把先前超时断言为“CDC 导致布线死循环”。

建议后续保留本次有效的写入 Bank 流水，针对 eth125 的 RAM 读出/地址、桥接计数逻辑继续分级，并单独解决 PLL 到 RAM 读时钟的脉宽及 PCLK 输入走线问题。不能靠放宽周期、忽略告警或删除全点代码作为通过依据。

## 用户要求的实验版下载

2026-11-02 22:05:40（北京时间），Programmer 确认 SRAM 下载完成。位流 SHA256：`761696945e914e3685855dabc92544467e5f933eefae7305113223be1eeef2ff`。
静态稳定版下载工具的验收门槛保持不变，实验下载由独立的 `program_experimental_system.py --experimental` 执行，保留未过时序、恢复版本与前一板上版本的记录。

实板功能尚未确认：下载后 PC 有线网卡为 Disconnected，FPGA NCFG 查询和 Nano HTTP/SSH 均未成功。因此只能确认 JTAG 下载成功，不能据此声称图传、光流、雷达或 IMU 已在本实验版实板验收通过。接回有线网络后还需配置 Nano 接收端，并重启其 `robot-web` 接收器以处理 FPGA 序列号重置。

恢复入口：已发布稳定标签 `lidar360-mpu6500-20261102`；本机原已验证构建为 `build/lidar360_mpu6500_20261102T131932Z`。本次没有执行恢复或发送电机控制命令。
