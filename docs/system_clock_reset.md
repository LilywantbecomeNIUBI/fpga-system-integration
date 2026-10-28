# 统一顶层时钟、复位和跨域依据

本文记录实现依据，不能代替板级时钟、电气和CDC工具审计。

| 域 | 时钟 | 复位与依赖 |
|---|---|---|
| 底盘、UART、IMU | 板载50MHz `clk50m` | `system_top` 两级释放 `reset_sync`，异步置复位；不依赖相机、PHY、PLL或IMU ready |
| 相机配置、MDIO | 视觉PLL输出50MHz | `reset_n && pll_lock`，在本域同步释放 |
| 摄像头采集、LK | 外部DVP PCLK，约84MHz | 视觉系统ready、PHY初始化、相机配置共同许可，再在PCLK域同步释放 |
| 图传、以太网TX | PLL输出125MHz | 感知发送和网络控制分别在125MHz域同步释放；感知依赖相机配置，网络控制仅依赖系统与PHY |
| 以太网RX | 独立PHY RXC，125MHz | 系统与PHY共同许可，在RXC域同步释放 |

`system_top` 直接实例化 `imu_1khz_core`，不会搬入旧顶层的驱动占位输出或ADC/FOC启动依赖。IMU原子提交和故障状态只进入遥测。IMU失败不会复位底盘；相机/网络的复位只作用于视觉层级。

串口RX、两轮AB和IMU SDA/SCL均经现有同步器进入50MHz域；SDC仅对这些异步输入的第一级D设置例外。急停既有异步关断链，又经过稳定释放/明确CLEAR/ARM，测试覆盖异步急停输出关断。所有电机闭环状态和串口打包在同一50MHz域，不跨到相机时钟。

视觉复用冻结实现：`vision_frame_stream` 通过稳定的帧缓存和请求/应答交接帧所有权；`vision_mailbox` 对语义种子和LK观测使用双级toggle请求/应答，发送数据在应答前保持不变，只保留最新待发快照。两端共同异步置复位，分别同步释放。网络RX到TX控制交接沿用 `network_runtime_dual` 的握手邮箱。独立DVP与PHY RX域在来源SDC中声明异步；没有新增整模块false path或multicycle，也没有降低50/84/125MHz目标。

同步器、握手、共复位及异常隔离依据已结合RTL和完整顶层仿真检查；没有声称完成专用CDC形式化签核。相机数据IO延时和RGMII RX延时沿用来源初始假设；TX外部接收窗口及板上PHY内部延时仍需电气/PVT核对。STA端点0违例只说明现有约束下的内部时序结论，不能覆盖未测外部接口。
