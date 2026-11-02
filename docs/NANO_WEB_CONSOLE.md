# Nano 独立局域网观测台

目标入口为 `http://192.168.50.20:8872`。Nano 汇集 FPGA 遥测和本机视觉结果，多个浏览器共享状态及视频缓存；电脑只需接入机器人路由器，即使原调试电脑关机也能查看。主机 IP 不是 FPGA 的 IP。

本次不修改 FPGA 源码或下载位流。页面复用现有雷达全量/720/360点、自动范围、IMU姿态、轮速、YOLO/LK及环境光流显示。局域网页面只读，所有 POST 返回403，不提供电机控制、参数修改、关闭服务或远程文件路径；不会因为队友关掉页面而撤销其他控制会话。原 PC 调试入口继续保留。

## 链路

- FPGA 状态 → Nano 唯一串口网关，或兼容固件的 UDP6205 → Nano网页。
- FPGA 视频/事件 → 既有 Nano 视觉进程（UDP6102/6202、HTTP8766）→ 网页共享缓存。
- Nano HTTP8872 → 路由器 LAN/Wi-Fi → 队友浏览器。所有图片 URL 都与页面同源，没有指向 PC 的回环地址。

`nano/web_console.py` 内嵌网关，独占 FPGA 串口；它不重复绑定视觉 UDP 端口，也不再加载一份模型。可选兼容网关 API 仅监听 `127.0.0.1:8873`，仍要求令牌。需要本机工具复用它时，在服务环境设置 `ROBOT_GATEWAY_TOKEN`；令牌不会进入公开状态或页面。

网页服务只需 Python3.6+及 live 模式的 pyserial。它不导入 OpenCV、NumPy 或 TensorRT。视觉进程沿用 Nano 原来的依赖和已存在的模型；新增 `/camera.jpg` 返回最新接收画面，最多10次/秒编码，与浏览器数量无关。旧视觉进程缺少该接口时，检测结果仍可查看，原图区域会报告错误；需要升级并正常重启视觉进程后恢复原图。

## 准备部署包

在开发电脑的本仓库执行，输出目录必须是新目录：

```powershell
python tools/prepare_nano.py --output dist/nano_web_20261102
```

将包完整上传到 Nano 的独立目录，如 `/home/liwei/robot_system_web_20261102`。保留既有部署目录及模型，不覆盖正在运行的视觉程序。包内 `MANIFEST.json` 列出逐文件 SHA256；现场上传后应逐文件校验。

## 配置与手动启动

在 Nano 的新部署目录内执行：

```bash
cp config/nano.web.example.json config/nano.web.local.json
cp config/nano.vision.example.json config/nano.vision.local.json
ls -l /dev/serial/by-id/
```

修改 `nano.web.local.json` 的 `fpga_serial` 为连接 Nano 的真实 FPGA UART 稳定路径，串口线须从 PC 移交给 Nano。雷达经 FPGA 上传时保留 `lidar_source: "fpga"`；旧 USB 雷达配置可显式选 `usb` 并填写另一个 `lidar_serial`。`record_dir` 在 Nano 本机记录遥测；用户需管理长期录制的磁盘空间。

`telemetry_transport` 默认 `serial`，兼容现有 UART 固件，网页迁移不要求重新编译 FPGA。只有板上已经运行并验收 UDP 遥测固件时，才改为 `udp`，确认第二目标地址为 Nano `.20`、端口6205。电机指令仍走原 UART。

仅通过网口观察时，可同时设置 `receive_only: true`、`telemetry_transport: "udp"` 和 `lidar_source: "fpga"`。此模式不要求 `fpga_serial`，不导入 pyserial，不打开任何 UART，也不启动8873控制网关；仅接收 UDP6205。适合队友共享查看，以及暂未移交串口的阶段。没有兼容的 UDP 遥测输入时各状态显示离线，不能把网页在线当作固件验收通过。

`nano.vision.local.json` 填写 Nano 已存在的模型及 engine 路径。若旧视觉进程已经运行，先复用其 HTTP8766，单独启动网页即可。需要新视觉版本时，先通过原启动入口正常停止旧进程并确认6102/6202/8766已释放；不要同时启动两个接收器。

```bash
# 离线检查：不连接设备、不连接视觉服务、不生成模拟数据。
python3 nano/web_console.py --config config/nano.web.local.json
# 退出离线检查后，正式接收；启动网关只发 STOP，不自动 ARM。
python3 nano/web_console.py --config config/nano.web.local.json --live
# 仅当原视觉进程已经正常停止时，在另一终端运行：
python3 nano/boot_vision.py --config config/nano.vision.local.json
```

FPGA 原有视频目标需包含 `.20:6102`。原双目标配置已正确时无需更改；断电重启后，可在 Nano 用原配置协议恢复第二目标，不依赖 PC：

```bash
python3 tools/task4/telemetry_endpoint.py --bind 192.168.50.20
# 上一行只查询；下一行显式写第二目标，不改第一目标或发送运动指令：
python3 tools/task4/telemetry_endpoint.py --bind 192.168.50.20 --target 192.168.50.20 --video-port 6102
```

本服务不会自动改 FPGA 端点，沿用原 AV 归属检查。FPGA断电后的端点恢复与SRAM位流恢复仍须按现有固件启动方案处理；仅安装网页服务不能替代板卡上电加载。

## 开机启动

新视觉进程、串口及网页均手动验证后：

```bash
# 先打印具体 unit 内容供检查，不改系统：
python3 tools/install_nano_web.py --config config/nano.web.local.json --vision-config config/nano.vision.local.json --user liwei
# 安装并启用，下次开机自动启动；本命令不抢占现有进程：
sudo python3 tools/install_nano_web.py --config config/nano.web.local.json --vision-config config/nano.vision.local.json --user liwei --install
# 已正常停止旧服务且端口释放后，启动新服务：
sudo systemctl start robot-vision robot-web
systemctl status robot-web robot-vision
journalctl -u robot-web -u robot-vision -n 80 --no-pager
```

已由其他启动机制管理视觉时，可以省略 `--vision-config`，只安装网页服务，但要确认原视觉服务也能随 Nano 开机启动。若同名系统 unit 内容不同，安装器拒绝覆盖，先保留旧 unit。退出和重启网页服务会按现有网关逻辑 STOP 并释放串口，不自动恢复运动控制。

卸载启动项：`sudo systemctl disable --now robot-web robot-vision`，会停止新网页及由该 unit 管理的视觉进程；保留部署文件和历史记录。重新使用 PC 网关前先停止 Nano 网关，串口始终只由一个进程持有。

## 队友连接与验收

1. Nano 接机器人路由器 LAN，保持 `.20` 地址；队友接同一路由器的 Wi-Fi，使用不同的 `.x` 地址。路由器需允许无线和有线设备互访。
2. 若 Nano 本机防火墙已启用，允许机器人子网到 TCP8872；浏览器不需要访问私有8873或视觉8766。没有 WAN 不影响查看。
3. 在两台队友电脑同时打开 `http://192.168.50.20:8872`，检查雷达、IMU、视频和事件新鲜度；断开原PC网络后继续推进才算脱离PC通过。
4. 临时断开某传感器，应显示离线/过期并清除其旧画面；恢复后再出现新数据。网页可达不等于各传感器健康。
5. 验证 Nano 重启后服务恢复、模型路径正确、串口权限正确；FPGA断电后的位流/端点按原方案恢复。

本地验证命令：

```powershell
python -m unittest discover -s tests -p test_nano_web.py -v
node tests/test_nano_web_ui.js
```

本地测试覆盖真实 HTTP、多浏览器只读、共享上游缓存、视频帧与状态过期、断线清除、Python3.6语法及隔离部署包导入；不代表 Nano 硬件、实际帧率、Wi-Fi吞吐或开机启动已验收。
