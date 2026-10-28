# 第一阶段系统集成：使用与验收

本阶段为有刷底盘遥控与感知并发候选。音频、自主跟随、避障、SLAM、FOC和平衡环不在本固件内。原模块与无刷工程保留。软件构建和打包不会连接硬件；本文部署及运动步骤尚待现场执行。

## 链路和供电

- FPGA：ACG720 GW5AT-138B，D157B双路控制和3.3V调理后的AB编码器，IMU接F19/F20，摄像头和PHY保持原连接。P7旧FOC对应网络必须断开；旧使能F13/C15在网表中保持低。急停既有硬件断能回路，又向B16提供状态。
- Nano：一个USB串口连接FPGA，一个USB串口连接STL-27L。端口使用`/dev/serial/by-id/`稳定路径，不能让旧查看器和网关同时占用。雷达只接收，不发送启动/调速命令。
- FPGA与Nano通过以太网传输图像；PC通过无线访问Nano网关。保留PC/Nano双目标图传时，PC图传接收地址必须在FPGA的192.168.50.x同一二层网段，通常通过交换机/AP桥接；仅把PC接到另一个无线子网不会自动收到FPGA图像。不要假定Nano会路由或转发视频。
- FPGA及Nano沿用各自供电，D157B电机VM接12V；模块5V/3.3V输出不与其他电源输出并联。

## 本地离线构建

```powershell
python -m pip install -r requirements.txt
python tools/shared_check.py
python tools/test_system.py
python tools/test_full_top.py
python tools/build_system.py
python tools/prepare_nano.py
```

`build/system/system.gprj`是统一工程，顶层`system_top`，位流为`build/system/impl/pnr/system.fs`。`build/system/result.json`必须为passed且与当前源码哈希一致。原`tools/build_motor.py`和独立电机入口继续可用。

完整数字顶层仿真需要本机Gowin原语库；CI运行协议、控制、UART、LK与双目标网络回归，不把Gowin仿真模型上传Git。完整顶层测试保持相机/网络及IMU时钟，使用缩小图像缓冲、处理后像素注入、行为传感器/PLL，并加速电机/UART计数器10倍；它验证并发与故障隔离，不能代替真实传感器、电气或实际周期验收。

来源固定在`docs/integration_sources.json`；`vendor/reference/source_manifest.json`保留上一级模块来源。不可修改的快照逐字节检查，允许修改的上位机适配文件明确标记adapted。构建不从相邻仓库隐式复制文件。

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
- 目标单位为`counts`（闭环计数/10ms）或`permille`（显式开环调试）；物理单位`physical`使用`v`（m/s）、`w`（rad/s），只在实测标定确认且极性匹配时开放，初始上限0.15m/s、0.5rad/s。这些上限不是已实车验证的安全速度。
- 超过200ms没有有效控制心跳或新电机状态、会话释放、FPGA故障、串口断开，网关撤销控制权并尝试STOP。重复或旧遥测不刷新状态期限。FPGA自身250ms看门狗不依赖Nano线程或HTTP。

IMU与雷达在本阶段仅观测；它们失效不自动改写手动运动指令。姿态显示仅为传感器轴互补滤波，不是车体定位。标定后提供编码器二维里程计，断连时重建计数参考，不执行SLAM或IMU航向融合。

## 标定、记录和动态验收

标定配置默认confirmed=false，CPR、轮径、轮距和极性掩码为空，不猜测Xp28减速比。动力断开手转测得输出轴四倍频计数，再以悬空短时操作确认轮向与反馈符号，之后填入实测几何并确认。PI增益和保护阈值仍需实车整定。

Nano每次live启动创建独立记录目录：原始混合UART、原始雷达UART、命令/异常及10Hz完整状态JSONL。磁盘写入使用有界队列；队列丢弃或磁盘错误明确计数，不能判为记录完整。PC另有显式状态录制和单帧保存。

按动力断开感知并发→悬空单轮→悬空双轮→落地低速→30分钟并发递增验收。先记录相同配置的静止基线，再记录运动窗口；保留原始日志、版本和哈希。通过`tools/evaluate_dynamic.py --baseline ... --moving ...`检查至少30分钟、四路吞吐≥静止基线90%、各保护通过、CRC与记录错误为0，以及预先填定的停车延迟和轮速误差限值。缺项或非有限数值会拒绝通过。

当前没有本车上述实测结果，不能用仿真、历史单模块记录或STA代替。第二阶段先修复有效麦克风采样，再接入共享网口和语音，先支持语音STOP。

## LK时序实现的来源

`fpga/vision/src/vision/vision_lk_tracker.v` 保留冻结原件；系统构建选择 `fpga/system/vision_tracker_timing.v`。差异仅为 `present` 历史帧查询函数：保留有效位和高位标签比较；有效bank的标签低HB位必然等于bank索引，因此不再重复比较这些低位。复位清空有效位，写入时以帧ID低位选择bank，这个不变量在每次查询时保持。历史深度、状态机、数值算法、帧格式和时钟均不改变。LK回归使用系统实际选择的实现，完整顶层使用同一构建文件清单。

动态指标可从 `docs/dynamic_baseline.template.json` 和 `docs/dynamic_moving.template.json` 复制填写；模板故意为null，不能通过验收。记录相同配置标识、源码版本、原始日志路径及测量方法。IMU采集率由内部sample_seq及设备时间增量计算，100Hz串口上传率单独统计，不能混用；设备与主机时间也不能当作精确同步。控制周期/抖动和物理停车时间须用现场仪器或明确的板内测量依据记录，20Hz状态接收周期不能替代100Hz速度环周期。动态验收时另行归档这些测量证据。
