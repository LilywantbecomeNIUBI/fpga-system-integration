# 全点雷达与麦轮整机时序迁移

本次在独立分支 `codex/mecanum-integration-20261103` 上继续，麦轮初始集成检查点为 `3772ef7`。
没有修改以太网智能体所在的原 checkout，也没有下载板卡。

## 保留的基线

按原冻结清单逐个核对 SHA256 后，从 `20261102T192736Z/source` 迁入全部有差异的 RTL。
包括 reset_sync 的复位分发、camera_perception、分 Bank mailbox 和 network_runtime_dual，以及其他先前雷达/视觉优化。
未用当前工作目录里的浮动文件代替冻结源码。证据在 `docs/evidence/raw-migration-20261103/baseline360_*.json`。

该基线 Setup/Hold 为 0/0，但 MPW 为 −0.849 ns，仍有 camera_pclk_d 的 PR1014；原 `result.json` 的整体 passed 为 false。
最差 Setup 为 `refresh2_due → retry_count2[4]/CE`，+0.001 ns。
本次既不放宽 125 MHz 周期，也不添加数据路径 false path 来隐藏违例。

## 同拍局部使能

在 lookup_valid/refresh_due 更新的同一沿，从 found_comb、cache_age 和原有失效条件计算
`retry_hit_clear0/1`，并生成 dispatch 局部副本。它们满足：

```text
retry_hit_clear0(t) == lookup_valid(t) && !refresh_due(t)
retry_hit_clear1(t) == lookup2_valid(t) && !refresh2_due(t)
```

当前周期的 clear_cache 仍在消费者端用同步局部副本屏蔽，因此缓存清除脉冲不会错移一拍。
P_COMMAND 的 lookup 失效条件沿用原始 memory[47] 判断；不能改用 qualified command_ok，因为原逻辑对非法命令也会撤销这一拍 lookup。
重试计数器依次执行减计数、命中/命令清零、dispatch 装载，最后一个装载保持最高优先级。

`syn_preserve/syn_keep` 保留局部寄存器；实际综合网表已确认局部与 dispatch 副本未被合并掉。
并非将 `(found && !refresh_due)` 寄存后再延迟使用。

`test_final_timing.py` 新增与冻结旧网络 RTL 的逐拍对比：CLK_HZ 为 125000000/999/1，CACHE_SECONDS 为 30/1，
每组 50000 拍；另外保留分段计数、命令/装载优先级和复位 mailbox 等价测试。
原计数算术测试用 force 绕过查表，因此同步驱动新的局部谓词；新逐拍测试不 force 新谓词，验证它们由真正的查表组合输入同拍生成。
参考源码 SHA256 固定为 `28c4c01c562903759751459662dc843fdf6a119d07f6b46153de286e9a8975a7`。

首轮全点/麦轮布线 `20261102T195814Z`：重试端点最差 +0.258 ns，原 retry_count2[4]/CE 也是 +0.258 ns；
全点 RAM DO → active_points 最差 +15.785 ns。因此不修改全点读口流水线。
该轮尚有 3 个 Setup：target_ip → retry_hit_clear0/dispatch 两条 −0.045 ns，parse_state → target_ip[4]/CE 一条 −0.006 ns。
失败记录保留，不将它标记为零违例版本。

为清理这三条路径，继续增加独立的 32 位 retry_target_ip0/1，同样在原 P_COMMAND 沿同步更新；
目标 IP/端口寄存器从大型 parser/reply 分支移到独立 always，SET 的合法性在整个校验扫描期间预解码。
新旧 RTL 的 `test_network_cycle.py` 在真实的 47 帧 golden 场景下逐拍比较公开输出、有效 TX 字节、重试计数和 ARP due，
并检查两个局部 IP 副本与实际目标 IP 始终一致；测试通过。
后续综合网表也确认原目标 IP 与局部副本各自保留 32 个触发器。

## 全点路径与引脚

构建命令：

```powershell
python tools/task4/build_ethernet.py --system --mecanum --raw-points --clock-cleanup
```

顶层 `system_mecanum_top`，冻结副本 `LIDAR_RAW=1`。麦轮/IMU/遗留禁用输出使用队友引脚，
相机和以太网保持原约束，雷达仍为 B16。`--sector360`/默认模式及原双轮入口均保留。

LOCAL_CLOCK 根据模式选择真实节点：

- 360 点：`ethernet_scan.lidar_display/capture`。
- 全点：`ethernet_raw.lidar_display/capture_crc`，已在综合网表确认存在。

麦轮的 snapshot capture 会被综合吸收/改名，故不凭 RTL 名称添加 `chassis/capture` 约束。
首次构建在约束解析阶段检测到这一无效名称并保留失败记录，后续已移除；未删除任何电气引脚约束。

全点缓存是 256×32 的窄同步 RAM，读取先进入 ram_q，再在 R_TAKE 装载 active_sequence/ms/error/points。
后面才是 R_BUILD/R_CRC；没有把 RAM 输出接到角度 `/100` 或跨 4 Bank 的大组合选择链。
因此先保留其既有流水级数，以实际 sys50 报告决定是否还要改动，不为类比问题随意增加整包延迟。
构建报告另增加 retry_count 端点和全点 RAM/active_points 路径的定向输出。

## 验证状态

实际构建、仿真和时序结果在完成后记录到本目录的 evidence。
初始 360 点麦轮整机已经离线验证四轮/P4、真实 MPU6500 I²C 模型、1000 个雷达包、坏 CRC 恢复、独立最近回波参考及视频并发。
全点测试在同样的生产顶层逐个比对 12000 个角度/距离/质量值、原包序号、两个半包及错误断点，不以“有 UDP”代替内容验证。

`audit_raw_timing.py` 单独报告 Setup/Hold、定向重试端点、全点 RAM 读出和完整 signoff，要求实际定向报告命中真实引脚；
原 1 ps 端点加固和重试端点整体裕量分别报告，另保留 0.100 ns 的建议裕量检查；建议项不等同于用户要求的 Setup/Hold 闭合，
更不把时钟/脉宽问题排除在完整 signoff 之外。

旧网络 golden 的仿真墙钟超时从 45 s 调为 180 s，适应与高云及整机仿真共用 CPU；仿真时长、包数和逐字节断言不变。
所有这些均为离线验证，不能代替真实 PHY、相机和四轮接线的实板验收。

## 最终实测：20261102T201012Z

| 项目 | 结果 |
| --- | --- |
| Setup / Hold 违例 | **0 / 0**，sys50、cam_pclk、eth_rx125、control50、eth125 均为 0 |
| 原 retry_count2[4]/CE 端点 | **+0.275 ns**；最差起点已从 refresh2_due 变为 action[3] |
| 全部重试计数器端点最差 | **+0.098 ns**，为 action[3] → retry_count2[1]/CE |
| 全点 RAM DO → active_points 最差 | **+15.937 ns**，数据延时 4.032 ns；保持现有读口流水 |
| 全局最差 Setup | **+0.014 ns**，相机 Bank 3 读地址寄存器 → BSRAM 地址口 |
| 资源 | Logic 35806/138240，FF 31755/139095，BSRAM 224/340，DSP 103/298 |
| 时钟树 | PRIMARY 5/8，LW 7/8 |
| 复位分发审计 | 10 个分支，最大实际叶子扇出 139，通过 |
| 完整时序 signoff | **未通过**：MPW −0.849 ns、camera_pclk_d PR1014 仍在 |

原临界端点的裕量从 1 ps 增至 275 ps；重试端点整体最小值 98 ps，比建议的 100 ps 少 2 ps，审计保留该建议项为 false。
全局最小 14 ps 仍薄，不将本版本描述为留有充足余量的整机稳定固件。
camera_pclk_d 在实际分配表上已是 PRIMARY，但仍报 PR1014，不能仅凭 LW 使用量就把它归因于 PRIMARY 被控制信号占满。
不使用自动生成的 n18771 等节点名作为新的固定 CST 约束。

相同冻结源码的整机测试再次通过：2000 个半包、12000 点逐项一致，25 个完整原始密度扫描，
1000 包转发且丢包 0，MPU6500、M4/P4、视频并发、两层 CRC、未使能电机低电平及 UART 空闲均通过。
12 项时序功能回归、真实网络 47 帧逐拍等价测试、麦轮引脚/两种雷达模式构建检查和旧 UDP 回归均通过。

冻结工程与压缩位流保存在 `releases/mecanum-raw-setuphold-20261103`。
位流 SHA256 与完整冻结文件哈希见包内 manifest/result/SHA256.json；本次未运行下载或 Flash 工具。
