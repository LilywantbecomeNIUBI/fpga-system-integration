# 麦轮路由器控制（2026-11-03）

从 `main` 阶段提交 `dbbb4e3` 开发，分支 `codex/mecanum-router-control-20261103`。
当前整机的运动命令改走 UDP 6206，保留 V3 的 24 字节内容及 CRC16。
FPGA 通过板载 RJ45 接路由器 LAN 口；电脑既可插 LAN 网线，也可连同一路由器 Wi-Fi。
无线指电脑到路由器这一段，FPGA 板本身没有因此增加 Wi-Fi 硬件。
车上路由器若需要无线回程，须由路由器自己的桥接功能提供同一二层 LAN。

## 网络与启动

沿用板端静态地址 `192.168.50.30/24`。路由器 LAN 配置为该网段，电脑选一个不冲突的地址
（本次只读核对有线接口为 `192.168.50.10/24`），有线和 Wi-Fi 均可。关闭访客/AP 客户端隔离。
当前实现使用同网段 ARP，不提供 DHCP、跨子网网关转发或 VLAN 帧解析。
电脑只启用实际使用的网络路径，或用 `--telemetry-bind` 明确选择接口地址。

新固件的整机顶层固定使用网络控制，不同时混入 UART 命令；串口引脚保留但不驱动底盘。
`--transport serial --port COM15` 只用于配合旧串口固件，不能让新固件重新启用 UART。

先按下文构建、检查并下载新固件，随后双击仓库根目录 `start_mecanum_router.cmd`，或运行：

```powershell
python host/mecanum/gui.py --telemetry-bind 192.168.50.10
python host/mecanum/console.py --action monitor --telemetry-bind 192.168.50.10
python host/mecanum/console.py --action stop --telemetry-bind 192.168.50.10
```

GUI 默认离线，点击连接仅监测；执行按钮才开始 STOP → CLEAR → ARM → POSE 握手。
单个 24 字节命令对应一个 UDP 数据报，不自动重发运动命令；通过 M4/P4 序号和状态确认执行。
同一时刻只运行一个运动控制端，GUI 和 CLI 不同时占用 UDP6205。
允许电脑接收 UDP6205，发送到 FPGA UDP6206；现有网络配置入口仍为 UDP6200。

## 保留 Nano 图传，同时让 PC 收到回执

麦轮新固件会把同一份 UDP6205 遥测分别发给已启用并完成 ARP 的 PC、Nano 两个现有端点。
PC 槽用于控制台回执，Nano 槽继续用于图传、识别结果、光流和原状态接收。
只配置 PC 槽，无需把 Nano 的第二端点改到电脑：

```powershell
# 只读查询两个槽：
python tools/task4/telemetry_endpoint.py --bind 192.168.50.10 --slot pc
python tools/task4/telemetry_endpoint.py --bind 192.168.50.10 --slot nano
# 显式配置 PC 槽：
python tools/task4/telemetry_endpoint.py --bind 192.168.50.10 --slot pc --target 192.168.50.10 --video-port 6102
```

Nano 槽应保持 `192.168.50.20:6102`。如需恢复该槽，显式运行：

```powershell
python tools/task4/telemetry_endpoint.py --bind 192.168.50.10 --slot nano --target 192.168.50.20 --video-port 6102
```

配置 PC 槽也启用原有的第一路视频副本，发送到 PC 的 `--video-port`；运动 GUI 只消费 UDP6205 状态。
PC 不需要转发数据给 Nano。两份遥测在源端只读取一次，按现有调度分别发送，内容和 TLM1 序号完全一致。
一端停用、断开或 ARP 失效不会要求另一端停止接收；没有配置 PC 槽时 GUI 无法确认命令，会超时停止。
连接 GUI/CLI 本身不会自动改端点。板端重置/重新下载后需重新查询并配置。
本次没有修改 Nano 冻结服务，也没有自动切换实际板端端点。

## FPGA 接入与保护

`network_runtime_dual` 新增默认关闭的 `MOTOR_ENABLE`，仅麦轮整机开启；历史相机、双轮、静态诊断入口行为保持。
入口要求本机 MAC、IPv4 地址、同网段源地址、UDP6206、固定长度、不分片、正确 IP/UDP 校验及 Ethernet FCS。
合法 IPv4 的 UDP 零校验和仍支持，内层 V3 CRC16 始终校验。
完整命令通过请求/应答双触发器邮箱从 eth125 到 sys50，数据保持到全部 24 字节消费完成。
邮箱忙时丢弃新命令，不累积运动队列；网络重置同时清空邮箱和解码器的半帧状态。
网络尚未就绪时解码器保持复位，避免将等待初始化的每个时钟计作帧错误。

原位置闭环、四轮引脚、电气约束、IMU 保持策略、PWM 限制和 250ms FPGA 看门狗保留。
网络 ARM 额外要求序号比最近接受的命令新（8 位半区间判断），避免上一会话的重复 ARM 在 STOP 后重启。
POSE/KEEPALIVE 继续由原核心拒绝重复或倒序；断线后不自动清故障或恢复运动。
主机用新鲜 M4 序号开始新动作；运动过程中每 50ms 发 KEEPALIVE，反馈超过 300ms 即结束并尝试 STOP。
看门狗由 FPGA 独立执行，不依赖电脑成功发出 STOP。
控制入口按现有可信局域网使用，没有新增互联网接入或身份认证。

## 验证与构建

```powershell
python tools/mecanum/test_router.py
python tools/mecanum/test_mirror.py
python tools/mecanum/test.py
python tools/task4/test_network_cycle.py
python tools/task4/build_ethernet.py --system --mecanum --raw-points --clock-cleanup --route-option 1
# 将上述命令返回的冻结目录作为参数：
python tools/mecanum/test_system.py <冻结目录>
```

新增回归使用真实预导码/FCS/IP/UDP 字节，三组不同周期的时钟，经过生产网口解析器、CDC、V3 解析器和麦轮核心。
覆盖错误 FCS/IP/UDP、分片、异网段、MAC/端口/长度、内层 CRC、重复 POSE、旧 ARM、STOP、看门狗和复位。
双端点专项验证逐包检查相同状态在 PC/Nano 的 IP/MAC/端口、内容及 FCS，并验证停用任一端后另一端继续接收。
运动模型仅强制有效航向并执行零位移，不能替代 IMU 或实车动态验证。
构建目录、综合布线结果、回归证据以本次运行记录为准，旧阶段位流不能接收 UDP6206。
本次不自动下载、写 Flash、启动电机或切换 Nano 服务；新无线链路的延迟、断线及落地控制仍需实板验证。

## 最终离线结果

最终冻结目录：`build/lidar_raw_ethernet_20261102/mecanum/20261102T215739Z`（Place 2 / Route 1）。

- 已生成 `impl/pnr/sensor.fs`，SHA256：`22797c7779a044874aa65f45f28bcf4f12d0483c672f760984b265a79ca3aced`。
- Setup/Hold：0/0；控制域、LK 状态、CDC、帧 RAM 读出审计通过。
- 27 组 HDL 仿真、28 项上游 Python 测试及 11 项集成 Python 测试通过。
- 最终冻结源码整机仿真通过：12000 原始雷达点、并发视频、MPU6500 校准、M4/P4 CRC、未使能输出。
- PC/Nano 双端点专项通过：相同遥测副本、并发视频副本、源视频只读取一次、停用任一端后另一端继续接收。
- 命令专项通过：错误报文拒绝、旧序号/重复目标与 ARM 拦截、STOP、250ms 看门狗、网络启动不产生伪帧错误。

**完整时序签核仍未通过**：原 MPW -0.849ns 和 PR1014 告警保留；构建记录 `passed=false` 明确保留此边界。
本次没有下载 SRAM/Flash，也没有进行实际有线/Wi-Fi 运动或落地精度验收。
本次只读网络核对结果为 PC `.10`、板端 `.30`，Nano 第二端点 `.20:6102` 正常；Nano 默认冻结服务未修改。

代码和验证证据在 `docs/evidence/router_control_20261103/`。Route 0 曾报 PR0004（0 unrouted nets）并失败，原日志保留；最终使用 Route 1。
交付包位于 `build/router_control_delivery_20261103/mecanum_router_control.zip`，包含对应位流、冻结 FPGA 工程、PC 控制台及校验记录。
