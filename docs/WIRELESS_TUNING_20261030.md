# 无线调参和前后带载短测

## 连接设置与恢复

USB重插后暴露临时ACL丢失问题：设备存在但网关Permission denied。已在Nano安装`/etc/udev/rules.d/70-robot-fpga-uart.rules`，仅匹配CH340 VID/PID 1a86:7523的ttyUSB设备，赋予liwei读写ACL；同时恢复当前端口权限，网关无需重启即恢复新鲜遥测。该匹配不含唯一序列号，适用于此Nano的CH340类设备，未全局放开串口权限。热插拔规则实际重插验收尚未执行。页面区分网关在线、串口缺失/权限、遥测等待及完整链路正常。当前恢复时保持STOP和零输出，断线后的command_timeout故障保留，须用户显式解锁流程清除，不自动运动。

调参页顶部增加Nano地址、路由器/有线、手机热点、Tailscale预设及切换并重连，地址保存到忽略的本机配置，凭据不回传页面。连接请求绕过系统HTTP代理。断线立即清理旧租约和缓存状态，以0.2至3秒退避重试；正常轮询50ms，重连不重放ARM/运动指令。切换也释放原租约；200ms控制超时和FPGA看门狗保持。44项Python测试与两组Node UI测试通过。

机器人路由器SSID由用户确认为CMCC-rDtN-5G，无互联网，不能为了在线协作直接将PC切入后假定对话仍可用。PC与Nano同连手机热点Li‘s HUAWEI Mate60 pro已验证PC172.20.10.14→Nano172.20.10.2:8873，无Tailscale，三次查询137/34/39ms。页面已切热点地址，IP可能随DHCP变化。此时Nano无CH340 USB设备、串口路径不存在，只有网关网络可达，不能声称已恢复电机遥测或完成无线运动实测。

## 当前页面：点击持续行驶

用户改为自行操作后，主页和调参页入口改为“持续行驶”。先显式解锁，点击前进或后退开始，以50ms周期发送原有counts心跳（held=true），不再调用ground_test限时接口；点击STOP或空格停车并释放解锁。运行中方向和幅值禁用，改变方向须先STOP再解锁。幅值仍为1～8计数/10ms，默认4，方向flags3。切换窗口、隐藏/关闭页面、失联或遥测过期会释放控制，200ms网关超时和250ms FPGA看门狗保留；重连不自动恢复。后文的按住/1秒说明为此前版本。42项Python测试及Node持续心跳、停止与失焦回归通过，修改时未发运动指令。

## 续接检查（2026-10-30）

本机8874服务退出后已重新启动，连接Nano有线地址192.168.50.20:8873。实际遥测新鲜，未ARM、双轮速度和输出均0、故障0；当前位置左-9119、右10294。此次未发运动指令。全套41项Python测试和Node带载按钮测试通过。

Nano的BIT-Mobile已连接，地址10.194.12.227；但PC访问该地址8873超时，访问已有Tailscale地址100.71.33.3亦超时。Nano tailscaled日志显示控制/中继连接失败，尚不能认定校园网无线调参可用。保留当前有线配置，不应拔掉网线后直接下地。可改用PC连接机器人路由器Wi-Fi，再验证同一192.168.50.20网关；该无线切换尚未实测。

此前有线经Nano的300ms左轮试验实际增加69计数，右轮不动、峰值96‰、按时停车。用户页面记录的一次测试目标为零，另一次前进按键约130ms后已释放；使用方向按钮须先解锁并持续按住，每次最多1秒。它们不能替代无线链路或下地带载验收。以下移交PID和错误帧累计为当时快照。

上位机主页 `/home` 增加调参入口，系统状态台 `/` 也可进入 `/motor`（别名 `/tuning`）。专用调参页保留单轮限时测试、实时曲线、CSV，增加前进/后退按住短测。参数默认Kp4096/Ki128、左电机反向、编码器原方向；仅为架空短测初步候选。

## 带载按钮

先显式解锁，按住前进/后退。默认目标幅值4计数/10ms，允许1～8。前进左负右正，后退相反，基于此前车体前进方向的手转记录；第一次仍需架空核对滚动朝向。每次最多1000ms，松开/移出按钮/失焦/隐藏页面均尝试STOP或释放，到期停车并撤销控制，下一次需要重新解锁。不会自动ARM或恢复上一次运动。

新网关动作`ground_test`仅在已确认ARM且flags3时接受`direction: forward|backward`、整数`speed:1..8`、整数`duration_ms:100..1000`，服务端换算双轮目标并保留已有独占租约、200ms心跳/遥测超时及FPGA250ms看门狗。原`test`接口保持不变。FPGA PWM限幅仍为300‰，不能把之前试验的主机160‰抽样监测说成当前页面或硬件限幅。

## 无线链路

PC（机器人路由器Wi-Fi）→Nano网关HTTP:8873→Nano USB串口→FPGA。PC拔掉的是PC至FPGA串口线；FPGA的UART必须移接Nano且保持逻辑供电。FPGA断电会丢失SRAM候选，须重新下载后才能确认固件。图传以太网不能替代电机UART。

Nano包已准备在`dist/nano_ground_20261030_0928`，含MANIFEST SHA256，无模型和凭据。部署到独立目录，不覆盖现有视觉服务。Nano配置只选择实际CH340串口稳定by-id路径，禁用雷达；不要把Gowin JTAG FTDI接口当作UART。启动网关时从受控本地文件读取会话token到ROBOT_GATEWAY_TOKEN，不在终端打印。重连及启动只STOP，不运动。

PC本地配置`config/local_motor_wireless.json`（Git忽略）：

```json
{"gateway_url":"http://192.168.50.20:8873","token_file":"local_motor_token.json"}
```

`token_file`相对本配置目录，内容为`{"token":"与Nano网关相同的本地会话令牌"}`，不要提交。退出占用8874的旧PC串口会话后运行`start_wireless_tuner.cmd`，或：

```powershell
python host/motor_tuner.py --live --home --config config/local_motor_wireless.json
```

PC前端只访问网关、不打开串口。切换前确认STOP；切换后确认连续新鲜遥测、未ARM、输出0、新网关声明ground_test_supported，再架空验证前后。仅构建/单元测试通过不能代表无线链路或落地实测通过。


## 本次实际移交结果

Nano `/home/liwei/robot_motor_ground_20261030` 已部署并启动，PID19035，串口稳定路径 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。PC调参进程连接192.168.50.20:8873，8874提供主页和调参。实际Python3.6.9暴露冻结IMU查看器的future annotations兼容问题，现改为shared/imu_protocol.py的独立无GUI解析器，保留原协议/统计行为，并增加与冻结源的碎片、CRC及非法字段等价回归；41项Python测试通过。Nano上31个源码文件与28个串口依赖文件哈希已核对，实际3.6解析smoke通过。

观察约5秒增加100个电机状态包，30次查询均新鲜、未使能且输出0，接收CRC0、独立diagonal新增0；FPGA错误帧累计5，观察新增0。本次只STOP，无ARM/DRIVE。PC当前仍以太网接路由器，Wi-Fi为BIT-Mobile；纯无线需先接机器人路由器Wi-Fi并验证新鲜遥测，不能直接拔PC网线认定无线已通过。

liwei原不在dialout组，本次仅对当前/dev/ttyUSB0添加用户读写ACL，未全局放开设备权限；USB重插或重启后可能需要恢复。网关进程不是自启动服务；Nano断电后需重新启动。FPGA独立SRAM固件亦不能靠Nano自动恢复。
