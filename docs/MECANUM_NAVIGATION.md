# 麦轮导航交付说明

代码已接入整机，并由`NAVIGATION=0`默认关闭。只有显式`--navigation`构建
才启用自主控制。源码、周期设计、模块/兼容性仿真和综合核查已完成；本轮已获明确
全量整机实现授权。最终134947Z的Setup/Hold、复位释放与整机CDC审计通过；
保留PR1014/MPW实验边界，不是完整STA签核。实际交付状态以对应冻结目录的
`result.json`、`navigation_timing_audit.json`和`release.json`为准，不能把仿真当成实车验收。
上述为实现阶段记录；2026-11-03用户随后明确授权上板，已下载导航SRAM候选并验证实时M4/P4/N4、雷达及V4取消应答，未写Flash，未启动实体运动任务。上板证据见 `build/mecanum_navigation/board_20261103_navigation/`。

## 实现范围

* FPGA内保留16个任务原点相对路径点，逐点100mm到达判据，末点要求静止300ms；
  任务中始终保持起始朝向，航向偏差超过15°撤销许可。
* 复用原Task4数学、前视目标/VFH/直方图及麦轮CORDIC、四轮测速/PI、V3协议。
  新增全向动态窗口、候选排序、每轮前8矩形精检与失败候选后续轮换。
  圆形筛查只增加代价，不能用圆形误碰直接否决矩形候选。
* 128x128、50mm分辨率、两代地图；完整扫描清图/车身FREE初始化/射线更新后发布。
  填充矩形精检覆盖内部占用，含采样/制动余量。规划50ms硬预算超时整轮停止。
* UNKNOWN高代价且速度<=50mm/s；OCCUPIED与保留码阻塞。当前协议未证明有明确
  超量程状态，因此没有把零距离/零强度当成“可清空至1.2m”的证据。
* 雷达轴向危险区与前向光流方向门控，允许已精检的切向脱困；检查实测动量。
  斜角障碍保守地同时约束相邻轴向分量；最终门控另支持8个法向。
* 光流无新鲜有效状态时降至雷达50mm/s模式。IMU新鲜度由导航独立强制检查，
  不把原手动模式“缺帧保持最近航向”的行为当成导航新鲜数据。
* 独立V4 24字节事务、N4 48字节状态、CRC、序号和250ms租约。
  失联停车、保留任务，必须显式继续；重复心跳不续租。
* 网页“路径导航”入口、路径与观测图、标定导入、上传/开始/继续和任务2切换。
  网页图中橙线仅是当前速度预览，不是上传完整FPGA规划轨迹。

## 文件与复核证据

* 周期设计：`docs/MECANUM_NAV_CYCLE_DESIGN.md`，先于复杂RTL创建，附最终状态、位宽、
  RAM请求/响应与冲突、WCET/FIFO预算和后续修正。
* 来源与版本：`docs/MECANUM_NAV_SOURCES.json`。Nav2仅参考填充轮廓问题背景，
  无源码复制；固定Apache-2.0版本见清单。
* 代码复核：`docs/MECANUM_NAV_CODE_REVIEW.md`，列出修复项和未关闭的物理审核项。
* 模块测试：`build/mecanum_navigation/kernel_tests.json`及各bench日志。
  100个独立SAT/密集扫掠模型场景没有漏碰，5个保守拒绝；964个坐标除法向量。
* 旧麦轮：`build/mecanum/tests/result.json`；旧导航：`build/task4/validation.json`。
  最终候选的源码哈希、整机仿真与资源时序摘要见 [实现记录](evidence/mecanum_navigation_20261103/implementation_20261103T134947Z/)。目录中记载的位流和布局布线参考路径对应本机交付。

## 运行与标定

`config/mecanum.navigation.pending.json`故意保持`confirmed=false`和待实测值。
须实测前/后/半宽、外接圆、轮式原始计数到毫米的前向及横向比例、旋转等效臂长、
雷达安装外参、最高速度下停止距离与减速度，再确认。测试用尺寸没有写进可启动配置。
pad至少包括36mm格心余量及实测停止距离；规划器另加64mm采样/旋转/整数步长余量。

网页顺序为：连接底盘→接管→切换任务3/4（先STOP/取消旧任务）→导入标定→上传校验
→开始。上传不使能。开始/继续会先STOP并等待连续静止300ms，再CLEAR/ARM和V4握手。
任务2按同样停车取消流程切回，保留现有定点闭环转向。旧固件没有N4时导航按钮禁止启动。

2026-11-04，8874已更新为 `unified-teach-20261103`，增加遥控与示教记点；使用方式及验证边界见
`MECANUM_TEACH_REMOTE_20261104.md`。上板已确认新版遥测、雷达、车载画面和遥控响应。短距离试车观察到直线偏移，行驶日志记录了间歇性 IMU 通信错误。实测标定和自主过弯、停车及绕行验收仍待完成。

## 复现

```powershell
python tools/mecanum/test_navigation.py
python tools/mecanum/test_grid_gate.py
python tools/mecanum/test.py
python -m unittest discover -s tests -p test_navigation.py -v
python tools/task4/test.py
python tools/task4/build_ethernet.py --system --mecanum --navigation --raw-points --clock-cleanup --route-option 1 --prepare-only
```

最后一条保留后续默认的完整整机增量策略。旧参考与navigation mailbox新增约束
不兼容曾使准备流程拒绝PnR；用户随后明确授权本轮导航全量，使用如下入口：

```powershell
python tools/task4/build_ethernet.py --system --mecanum --navigation --raw-points --clock-cleanup --full-rebuild --place 0 --route-option 2
```

快速选项仍须相同物理门槛，不因速度设置而放宽时钟或例外；实现历史及后续修正
见`MECANUM_NAV_IMPLEMENTATION_20261103.md`。整机仿真与审计入口：

```powershell
python tools/mecanum/test_system.py <冻结目录>
python tools/mecanum/audit_navigation.py <冻结目录>
python tools/mecanum/audit_navigation_timing.py <冻结目录>
python tools/mecanum/package_navigation.py <冻结目录>
```

## 尚未完成的硬件门槛

本轮全量来自用户对导航的单独授权，不沿用此前“三项流水线优化”的全量例外。
后续增量遇到PA2136仍必须停止，不自动回退。最终DSP物理占用、扇出/拥塞/布局热点、
CDC物理路径/例外命中、时钟连接与约束激活、Setup/Hold/Recovery/Removal/MPW由
对应冻结工程报告验收；只有完整.pr、.fs和候选审核齐备才发布导航SRAM。
异步电机/UART/LED输出没有外部同步I/O时序契约，不将其称为已覆盖所有I/O约束。

旧整机SRAM回退镜像及哈希保存在交付目录；它不含本轮导航，旧Setup/Hold为0违例，
但仍有PR1014与eth125 MPW -0.849ns，不作为完整STA通过的位流。

功能边界：仅局部静态/缓慢横穿障碍，横穿时以停车为主；不是SLAM或全局迷宫规划。
持续最长射线可能超过点处理吞吐，32深FIFO不能修复持续过载；必须丢弃该圈并停车。
大外廓/大裕量的八项完整精检可能超预算，当前实现会停车而非宣称总能完成八项。
轮胎摩擦、低速起步、打滑和实际停止包络仍须板上标定验收。
