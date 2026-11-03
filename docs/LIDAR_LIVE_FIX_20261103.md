# Nano 雷达实时点云恢复

用户在联合调试台看到雷达离线；雷达实际持续输出，FPGA UART 的 CRC、格式和停止位错误均为零。
先前 FPGA 重新下载/复位后，Nano 网关仍绑定旧序列，API 报 `restart_required`；重启网页清除该状态，
但旧的接收、逐包记录和网页处理共用 Python 进程，随后又因积压、UDP socket 丢包而无法持续恢复完整圈。
联合调试台只是嵌入 Nano 感知页；故障发生在 Nano 接收后端。

## 最终运行方式

- 网页：`nano-lidar-20261103-r4`，目录 `/home/liwei/robot_releases/nano-lidar-20261103-r4`。
- `robot-web` 的 `95-lidar-receive.conf` 指向该版本；原 `90-frozen-baseline.conf` 保留。
- `robot-vision`、模型、engine 及原 `nano-video-20261103` 冻结内容保持完整。
- 独立进程独占 UDP6205，完成 CRC、来源、顺序、时效校验和原始半包/整圈重组。
  每 100ms 向网页发布观测状态，进程退出或状态过期后隐藏旧扫描。
- 已验证边界和 CRC 的原始 UDP 点包直接进入雷达解码器，避免再次送进两个 UART 流解析器。
  六点使用一次结构解包，角度、距离、强度和有效性检查保持一致。
- 网页只复制需要更新年龄的状态字典，已完成扫描中的点列表不再每次深复制。
- 观测配置 `receive_only: true, telemetry_process: true, record_telemetry: false`。
  当前不逐包落盘；完整原始点仍全部接收和显示。旧的显式记录模式保留，独立接收模式不支持同时记录。
- 不改变 FPGA 位流、运动控制、网口端点，也不发送运动指令。

入口继续为联合调试台 `http://127.0.0.1:8874/` 的“Nano 感知”，
或 `http://192.168.50.20:8872`。选择“全量”和“自动适配”。

## 验证

33 项 Python 检查通过：网页、UDP 时效及来源、原始点重组、显示协议、真实 UDP 子进程接收。
原始点快速路径与旧 UART 解码路径逐点比较一致，包括零回波、重复角度、丢包后恢复及坏 CRC 拒收。
子进程退出时不会继续展示旧扫描。

实机窗口和视频实际交付率见 `docs/evidence/nano_lidar_fix_20261103/`。
最终约 65.10 秒、60 次状态查询全部在线，完成 647 圈；每圈 2174–2179 点，
扫描帧龄最高 195.47ms，雷达原始包缺失、过期包和 CRC 错误增量均为零。
同时通过公共入口实测 20 秒视频：原图 24.60 FPS、推理图 18.65 FPS；
没有宣称两路持续 30 FPS。联合网页已实际核对全量点云，截图为 `unified_live.png`。
原冻结基线 76 个文件校验完整，新网页清单 39 个文件校验通过；最终网页目录已设只读。
不把遥测总序列的 missing/rejected 当成纯雷达丢包：保留的 Nano 观测协议不解析新麦轮
M4/P4 遥测，底盘和姿态仍由联合调试台的底盘页处理；雷达专用 `missing_raw_packets` 单独核验。

FPGA 后续复位导致 `restart_required` 时，仍需要重新连接接收会话（`sudo systemctl restart robot-web`）；
本次没有更改控制链路的复位保护，也没有自动重授权控制会话。

## 回退

如需明确回退网页版本，将 `/etc/systemd/system/robot-web.service.d/95-lidar-receive.conf`
移动到不以 `.conf` 结尾的备份名，执行 `sudo systemctl daemon-reload`、
`sudo systemctl restart robot-web`。这会恢复原冻结网页，也会恢复旧接收链路的限制。
不删除任何部署目录或历史记录。
