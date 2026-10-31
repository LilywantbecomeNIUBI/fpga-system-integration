# FPGA 光流避障 V0-V2

本分支对应任务3。先读[调研与取舍](research.md)。既有 `image_v23` 与 Nano YOLO 工程保留；光流版本独立命名。

当前冻结范围、启动与复现入口、已知问题及其他视觉调试改动见 [2026-10-21进度说明](checkpoint_20261021.md)。

2026-10-19 基础实板记录见[实板验证](hardware_20261019.md)。本次发现并修复综合后张量累加丢失的问题，增加实际 Gowin 网表逐条对照。修改算术或综合选项后，除 RTL 回归外还需重新综合核心并运行下列检查；已有失败用例可在未带修复约束的旧网表上复现。

```powershell
python scripts/build_optical_flow.py --stage 0
python scripts/test_optical_flow_synth.py --stage 0
python scripts/build_optical_flow.py --stage 1
python scripts/test_optical_flow_synth.py --stage 1 --cases still
python scripts/build_optical_flow.py --stage 2
python scripts/test_optical_flow_synth.py --stage 2
```

网表检查使用本机 Gowin 厂商原语模型，默认核对每个场景前8个候选；`--points 190` 可检查全场，运行更慢。完整场景和190点逐条数值仍由 RTL 测试覆盖。

## V0：single-scale LK

`flow_pipeline` 输入160x90灰度帧流，固定8像素间隔、边缘8像素，产生190个候选向量/帧对。每个向量来自5x5 patch 的中心差分和局部最小二乘求解。前后帧必须完整、连续，三缓冲隔离采集与计算。低纹理、病态矩阵、越界、步长过大显式 invalid。

输出 `sample_valid` 表示一个候选已完成，`sample_good` 才表示可用光流；`flow_u/v` 为 signed16 / 128，向右/向下为正，单位为160x90图像像素/摄像帧。第一帧只建立历史，不输出伪造的零运动。

```powershell
python scripts/test_optical_flow.py
python scripts/build_optical_flow.py --stage 0
```

独立软件 reference：`reference/optical_flow/model.py`；测试条件与种子固定在测试脚本，运行时生成小型MEM/逐条参考记录到 `tests/optical_flow/`，不依赖手工准备图片。综合默认目标为GW5AT-138B，只综合核心，非实板验收。各阶段证据在本目录的 `vN_validation.json`。

V0 不含 IMU、SNN 或聚类；不能据单目原始光流直接认定障碍类别、深度或已完成实车避障。

## V1：三区域基础避障

`flow_avoidance_top` 复用V0，并接入 `flow_avoidance`。三等分视野中累积有效向量数和接近向量数。相对图心的径向点积 `d dot flow` 为正、扩张率大于等于2/128每帧、运动大于等于0.25像素才计入risk；risk占有效数至少1/4则该区危险。这是可复现的启发式接近程度，不是标定距离或TTC。

每区至少4个有效样本，否则UNKNOWN并停车。危险立刻生效，两个连续完整安全帧后才解除停车。中间危险时选可通行侧，左右风险同分时固定向左；全阻塞BRAKE。输入断帧/坏帧即时失效，100ms无新结果失效。`stop_request`、`steer`（-1左/+1右）、`speed_limit`（0..255归一化上限）供后续车体适配，不等同PWM。状态0 UNKNOWN、1 FORWARD、2 LEFT、3 RIGHT、4 BRAKE。

```powershell
python scripts/test_optical_flow.py --stage 1
python scripts/build_optical_flow.py --stage 1
```

reference中的原理和V0整数模型保持不变。安全测试覆盖三区统计、扩张/收缩、左右避让、中间危险、全阻塞、最小运动门限、无纹理、解除迟滞、输入失效和超时。任意消费端须在独立控制时钟中检测结果超时，不能指望停止的摄像头PCLK执行自身计时器。

## V2：two-scale pyramid LK

160x90 fine -> 2x2 box filter -> 80x45 coarse。每个网格点先在粗层做3次双线性warp/LK残差更新，初值乘2后在细层做2次refinement。两层共享一套求解器，不保存整场warp图；三帧的两层像素始终独立，不在计算中覆盖前后帧。粗层/细层任一阶段越界、低纹理、张量病态或步长超限，该点即invalid。

这是稀疏两级、有限次迭代的实现。没有声称覆盖任意位移；当前合成纹理4像素平移较好，6像素测试明显失效。详细误差、有效点数、资源与周期见[实际验证结果](validation.md)。V2也会略增加部分亚像素案例的误差，不能只展示改善的测试。

```powershell
python tools/project.py test flow
python scripts/build_optical_flow.py --stage 2
python scripts/build_optical_flow.py --stage 2 --board
```

## 板级工程与遥测

三个项目分别位于 `projects/image_flow_v0`、`projects/image_flow_v1`、`projects/image_flow_v2`。`--board` 同时执行综合、布局布线、setup/hold检查和位流哈希；输出位于 `build/image_flow_vN/impl/pnr/image_flow_vN.fs`。只构建，不编程。原始冻结Release清单不用于新光流位流。

当前三个板级入口均使用 `place_option=2, route_option=1`；V0 的 `max_fanout=32`，V1/V2 为64，由构建脚本固定默认值。小乘积映射修复后，V0 原配置在125MHz网络路径出现setup违例；限制扇出并启用已有资源复制后重新完成0/0检查。可用 `--place-option`、`--route-option`、`--max-fanout` 显式试验，但新参数必须重新检查时序；失败构建不作为可下载版本。

三个入口均复用1280x720 RGB565 / 30fps摄像头配置、320x320 letterbox双路图传。光流仅用实际视野，fine每像素对应原图8像素；输出坐标右/下为正，物理左/右仍须实板确认镜像配置。VDET传输与回执保留，Nano识别独立运行；光流入口没有原`image_v23`的颜色融合/历史目标跟踪，切回该入口可恢复原功能。

光流包走现有**第二图传目标**的固定UDP6202端口，版本1，32字节，网络字节序：

| 字段 | OFLW 向量 | OFST 状态 |
|---|---|---|
| 4字节 magic | OFLW | OFST |
| 4个U8 | version, stage, valid, 0 | version, stage, flags, state |
| 8个U16 | frame, x, y, signed-Q7 u, signed-Q7 v, sequence, 160, 90 | frame, 三个valid数, 三个risk数, speed高8位/steer低8位 |
| 2个U32 | 本帧计算cycles, 丢弃摄像帧数 | 同左 |

OFST flags：bit0停车请求、bit1结果可信、bit2..4左/中/右危险、bit5过期或不可用。独立125MHz网络时钟在100ms无新状态、端点不可用时将状态置UNKNOWN、停车、方向/速度置零。状态包优先；诊断向量通过现有最新值邮箱，在拥塞时会合并，`sequence`（0..189）用于识别缺失。控制端不依赖完整诊断向量接收。`dropped_frames`是输入帧丢弃数，不是UDP丢包计数。

后期实板时，可在同网段接收机上使用以下命令；`--slot 1 set`会改变第二路图传目的地，应在专门的光流测试会话中运行。目标IP为auto时使用接收机实际网卡地址。

```powershell
python pc/network_control.py --board 192.168.50.30 --slot 1 set --target auto --port 6102 --wait-arp 5
python pc/optical_flow_monitor.py --source 192.168.50.30 --seconds 30 --output runs/flow.jsonl
```

监听器不负责配置网络或下载FPGA；200ms无OFST会在本地标记UNKNOWN/STOP，即使仍有诊断向量到达。原视觉dashboard的FSTA解析器与OFST格式不同，光流观察请使用专用[光流观测台](dashboard.md)或此监听器。观测台入口为 `pc/start_optical_flow_dashboard.cmd`，支持同帧图像叠加、三区风险、录制回放和 V0–V2 参考演示。Nano继续接收图像时，可把监听器放到Nano运行。

## 文件与复现证据

- 软件逐位/浮点reference及风险模型：`reference/optical_flow/model.py`；上游原版与MIT许可：同目录`upstream/`。
- RTL：`src/optical_flow/`；唯一共用源码改动是`vision_divider`参数化，默认32位接口不变。
- Testbench：`sim/optical_flow/`。输入场景、MEM测试向量和oracle由两个测试脚本确定性生成到`tests/optical_flow/`，本机现有输出保留，克隆后重新运行即可恢复；无需下载图片。
- V0/V1阶段验证在`v0_validation.json`和`v1_validation.json`，对应各自提交。当后续修正公共流水时，以最终`v2_validation.json`及当前源哈希为准。
- `validation.md`是当前仿真与构建摘要；JSON保存源哈希、位流哈希、实际综合/布局布线结果、仿真误差和不包含实板测试的标记。实板基础观察另列于 `hardware_20261019.md/.json`，不代表车辆验收。

## 后期实板检查范围

先离地验证图像方向、静止/平移光流和无纹理停车，再测相邻帧稳定性、光照/曝光变化、车体pitch/yaw误触发与处理延迟。当前三个入口的电机使能固定关闭，`stop_request/steer/speed_limit`需与独立车体控制器适配并实测后才能用于真实行驶。IMU补偿属于V3；V0-V2没有去除自身转动造成的光流。
