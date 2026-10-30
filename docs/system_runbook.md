# 第一阶段系统集成：使用与验收

本阶段为有刷底盘遥控与感知并发候选。环境光流和避障状态以观测形式接入；自动跟随/避障运动、音频、SLAM、FOC和平衡环不在本固件内。最新架构见 [2026-10-30合并说明](PERCEPTION_MERGE_20261030.md)。原模块与无刷工程保留。软件构建和打包不会连接硬件；本文部署及运动步骤尚待现场执行。

## 链路和供电

- FPGA：ACG720 GW5AT-138B，D157B双路控制和3.3V调理后的AB编码器，IMU接F19/F20，摄像头和PHY保持原连接。D157B控制使用P7-21~24，左编码器31/32，右编码器27/36，必须拔下旧ADC数字连接并释放载板雷达TTL接口；详见引脚表。旧无刷使能F13/C15、全部PWM和CAL固定低，电机机械及相线可保留，建议断开旧驱动动力电源支路。软件STOP和独立看门狗保留；当前无P7-37外部许可输入，不能把它视为急停状态输入。
- Nano：一个USB串口连接FPGA，一个USB串口连接STL-27L。端口使用`/dev/serial/by-id/`稳定路径，不能让旧查看器和网关同时占用。雷达只接收，不发送启动/调速命令。
- FPGA与Nano通过以太网传输图像；PC通过无线访问Nano网关。保留PC/Nano双目标图传时，PC图传接收地址必须在FPGA的192.168.50.x同一二层网段，通常通过交换机/AP桥接；仅把PC接到另一个无线子网不会自动收到FPGA图像。不要假定Nano会路由或转发视频。
- FPGA及Nano沿用各自供电，D157B电机VM接12V；模块5V/3.3V输出不与其他电源输出并联。

## 本地离线构建

```powershell
python -m pip install -r requirements.txt
python tools/shared_check.py
python tools/test_system.py
python tools/test_perception.py
python tools/test_full_top.py
python tools/build_system.py
python tools/prepare_nano.py
```

`build/system/system.gprj`是统一工程，顶层`system_top`，位流为`build/system/impl/pnr/system.fs`。`build/system/result.json`必须为passed且与当前源码哈希一致。这里的passed只检查构建、源码一致性及Setup/Hold；完整时序签核另看`docs/evidence/system_v1/timing_audit.json`，当前最小脉宽告警尚未解决，不能据此直接上板。原`tools/build_motor.py`和独立电机入口继续可用。

完整数字顶层仿真需要本机Gowin原语库；CI运行协议、控制、UART、LK与双目标网络回归，不把Gowin仿真模型上传Git。完整顶层测试保持相机/网络及IMU时钟，使用缩小图像缓冲、处理后像素注入、行为传感器/PLL，并加速电机/UART计数器10倍；它验证并发与故障隔离，不能代替真实传感器、电气或实际周期验收。

来源固定在`docs/integration_sources.json`及`docs/perception_sources.json`；`vendor/reference/source_manifest.json`保留上一级模块来源。不可修改的快照逐字节检查，允许修改的上位机适配文件明确标记adapted。构建不从相邻仓库隐式复制文件。

## PC先看离线页面

```powershell
start_console.cmd
```

浏览器打开`http://127.0.0.1:8872`。默认离线；解锁、方向按钮和物理单位控制不可用。`--mode demo`保留明确模拟标记，模拟模式不向Nano发送控制。页面关闭、切换后台、失焦或松开方向按钮都会尝试撤销控制；即使撤销请求丢失，Nano租约和FPGA看门狗仍独立生效。

## Nano部署准备与启动（现场单独执行）

`dist/nano_system_v1`是代码和配置模板包，包含逐文件SHA256清单，不含模型、凭据或位流。打包工具拒绝覆盖已有包，需保留旧包后再生成新包。上传到独立目录后校验MANIFEST；不要覆盖现有视觉目录，不升级Nano系统或TensorRT。

将`config/nano.example.json`复制为本地配置，填写两个不同USB串口的稳定路径。Nano网关只需现有Python3.6+与pyserial3.5；PC的OpenCV依赖不必安装到网关环境。

在Nano设置`ROBOT_GATEWAY_TOKEN`环境变量（至少24字符，不放URL、不入Git），然后：

```bash
python3 nano/gateway.py --config config/nano.local.json
# 上面默认离线；接线和串口所有权确认后才使用：
python3 nano/gateway.py --config config/nano.local.json --live
```

网关默认监听8873端口，所有读写API均要求`X-Session-Token`。当前HTTP接口用于受控车载局域网；不直接暴露到公网。启动和重连均先发送STOP，不恢复旧控制权。退出时尝试STOP；写入失败时仍依靠FPGA独立超时停机。

视觉继续独立进程运行，明确选用现有YOLOv8n TensorRT引擎：

```bash
python3 nano/start_vision.py --bind 192.168.50.20 --port 8766 --model yolov8n --backend tensorrt --engine /EXISTING/model.engine --model-dir /EXISTING/models --token "$VISION_TOKEN" --out runs/vision
```

确认旧接收器已释放6102/6202后再启动，不自动终止任何其他工程进程。保持既有FPGA端点配置协议，PC为slot0、Nano为slot1，视频6102、反馈6201、事件6202、端点配置6200。网关不加载模型，YOLO阻塞不会阻塞串口控制线程。

PC复制`config/pc.example.json`为本地配置，填写实际网卡和Nano地址，设置相同`ROBOT_GATEWAY_TOKEN`后执行：

```powershell
start_console.cmd --mode live --config config/pc.local.json
```

实时入口只连接服务，不部署Nano、不改操作系统网络、不下载FPGA。PC不打开FPGA串口。Nano服务、PC服务和位流分别显式启停。

FPGA两个图传目标默认未配置。接收器启动并确认网络连通后，在PC显式设置端点：

```powershell
python tools/configure_endpoints.py --config config/pc.local.json
# 上面只打印计划；现场确认后才加 --apply 写入FPGA运行时端点：
python tools/configure_endpoints.py --config config/pc.local.json --apply
```

此入口沿用原网络协议和AV归属保护；已有AV会话占用时先由操作者正常结束并释放，不能绕过保护。

## 遥控与接口

页面先“申请控制并解锁”，网关通过STOP→CLEAR→ARM并逐步等待FPGA状态确认，才允许方向输入。20Hz心跳携带递增客户端序号；目标只保留最新一组，不排队。松开即STOP并撤销会话，下一次运动重新解锁。运动期间不支持更改增益或极性。

- Nano `GET /api/state`：运动状态、IMU、完整雷达扫描、健康状态、记录与标定状态。
- Nano `POST /api/control`：`claim`返回独占lease；`arm`提供lease、递增sequence、flags和Q8增益；`heartbeat`提供lease、sequence、held和目标；`release`只撤销匹配lease；`stop`为鉴权后的紧急软件停机。
- 目标单位为`counts`（仅闭环计数/10ms；开环及permille入口已删除）；物理单位`physical`使用`v`（m/s）、`w`（rad/s），只在实测标定确认且极性匹配时开放，初始上限0.15m/s、0.5rad/s。这些上限不是已实车验证的安全速度。
- 超过200ms没有有效控制心跳或新电机状态、会话释放、FPGA故障、串口断开，网关撤销控制权并尝试STOP。重复或旧遥测不刷新状态期限。FPGA自身250ms看门狗不依赖Nano线程或HTTP。

IMU与雷达在本阶段仅观测；它们失效不自动改写手动运动指令。姿态显示仅为传感器轴互补滤波，不是车体定位。标定后提供编码器二维里程计，断连时重建计数参考，不执行SLAM或IMU航向融合。

## 标定、记录和动态验收

标定配置默认confirmed=false，CPR、轮径、轮距和极性掩码为空，不猜测Xp28减速比。动力断开手转测得输出轴四倍频计数，再以悬空短时操作确认轮向与反馈符号，之后填入实测几何并确认。PI增益和保护阈值仍需实车整定。

Nano每次live启动创建独立记录目录：原始混合UART、原始雷达UART、命令/异常及10Hz完整状态JSONL。events.jsonl为每段原始数据记录文件名、字节偏移、长度、入队时的主机monotonic和wall_time；控制撤销及电机故障变化单独成事件。时间戳是接收端观察时间，不代表设备精确同步。磁盘写入使用有界队列；队列丢弃或磁盘错误明确计数，不能判为记录完整。PC另有显式状态录制和单帧保存。

按动力断开感知并发→悬空单轮→悬空双轮→落地低速→30分钟并发递增验收。先记录相同配置的静止基线，再记录运动窗口；保留原始日志、版本和哈希。通过`tools/evaluate_dynamic.py --baseline ... --moving ...`检查至少30分钟、四路吞吐≥静止基线90%、各保护通过、CRC与记录错误为0，以及预先填定的停车延迟和轮速误差限值。缺项或非有限数值会拒绝通过。

当前没有本车上述实测结果，不能用仿真、历史单模块记录或STA代替。第二阶段先修复有效麦克风采样，再接入共享网口和语音，先支持语音STOP。

## 综合实现

视觉/LK使用冻结原件，数值算法与帧格式不变。构建启用Gowin支持的BSRAM寄存器重定时优化；不启用DSP重定时，不降低时钟，不增加整模块时序例外。

动态指标可从 `docs/dynamic_baseline.template.json` 和 `docs/dynamic_moving.template.json` 复制填写；模板故意为null，不能通过验收。记录相同配置标识、源码版本、原始日志路径及测量方法。IMU采集率由内部sample_seq及设备时间增量计算，100Hz串口上传率单独统计，不能混用；设备与主机时间也不能当作精确同步。控制周期/抖动和物理停车时间须用现场仪器或明确的板内测量依据记录，20Hz状态接收周期不能替代100Hz速度环周期。动态验收时另行归档这些测量证据。

## 分步联调：暂不接雷达

配置`lidar_enabled: false`时，只要求FPGA串口，不打开雷达串口；雷达状态明确标记enabled=false。默认仍为true，启用时必须使用两个不同串口。PC临时持有FPGA串口的阶段仅用于分链路诊断，不代表Nano独占串口的最终链路已通过。移交Nano之前必须先关闭PC串口进程。

本车后续实物照片显示D157B 1.1，电机插座的编码器供电脚丝印为3V3；用户确认编码器采用3.3V供电，客服说明编码器支持3.3～5V。若AB也在3.3V域且没有5V上拉，可直接接FPGA，无须额外转换模块。旧资料原理图的5V供电标注不能替代本板电压确认；采用5V信号时仍需适当转换，不能并联两路电源输出。

专用轮速调参页为 `/motor`，`test` 动作需已显式 ARM，传入 `left/right` 与 `duration_ms`（100..5000）。网关计时到期 STOP/释放控制；持续 heartbeat 不延长截止时间。P7-37/F18 不再连接许可线，不接29→37跳线。先断开12V手转验证编码器；实际转动前重新确认供电、双轮悬空、人员与接线。下载和运动分开执行，禁止自动转动。
