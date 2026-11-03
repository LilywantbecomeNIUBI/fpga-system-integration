# 1kHz 麦轮控制流水优化：交给队友布线的完整工程

芯片：GW5AT-LV138PG484AC1/I0（GW5AT-138B）。推荐 Gowin V1.9.12.04。
顶层：`system_mecanum_top`；源文件均在本目录 `source/`，工程使用相对路径。
这是完整整机工程，保留摄像头、雷达原始点、以太网和 IMU；四轮内环 1kHz、外环 100Hz。
包含 360/720 度相对旋转、3.5 米直线/斜线及 4 米边界的固件与协议。
相应上位机位于仓库根目录 `host/mecanum/`，不要与旧固件控制比例混用。

## 在队友电脑构建

1. 克隆仓库并切换到 `codex/mecanum-pipeline-opt-20261103`。
2. Gowin IDE 打开本目录 `sensor.gprj`，选顶层 `system_mecanum_top`。
3. 必须使用下面的脚本运行，保留整机增量布线设置；不要在 IDE 中直接启动一次全新布线。
4. 脚本需要 Python 3 解压压缩基准；命令不同可传 `-Python py` 或 Python 可执行文件路径。

```powershell
cd projects/mecanum-pipeline-opt-20261103
# 综合+布局布线+位流生成；仅在队友电脑手动执行
powershell -ExecutionPolicy Bypass -File .\run_build.ps1 -GowinRoot 'C:\Gowin\Gowin_V1.9.12.04_x64'
# 如需先只看综合资源
powershell -ExecutionPolicy Bypass -File .\run_build.ps1 -Stage syn
# 综合已完成后继续 P&R
powershell -ExecutionPolicy Bypass -File .\run_build.ps1 -Stage pnr
```

本目录 `reference/pnr_reference.zip.part1`～`part3` 为完整旧整机 `.pr` 的压缩分卷（合计约 106MB）。脚本自动校验、合并并解压到 `reference_baseline/`，无需手动拼接。
布线固定使用 `-inc_place 0` 和 `-inc_pnr`，缺少基准、约束/顶层不符或文件损坏会停止，不退回全新布线。
基准已完成 P&R，但仍有 1 个 setup 违例与既有脉宽/时钟路由余项，不代表时序验收通过。

安装位置不同，用 `-GowinRoot` 指向自己的目录。脚本不设置 1800 秒硬超时，也不执行下载。
产物位于 `impl/pnr/sensor.fs`，完整日志是 `build.log`。

## 验收及反馈

此优化版本在开发机只做仿真与源码冻结，**没有运行综合、布局布线或上板**。
回归结果见仓库 `docs/MECANUM_PIPELINE_OPT_20261103.md` 和 `docs/evidence/mecanum_pipeline_opt_20261103/`。
开发机其他聊天可能仍在运行旧冻结版本的布线，该旧构建不验证本工程。
请回传 `build.log`、综合资源报告、`sensor.tr`/时序 HTML 和 P&R 报告。
检查所有时钟域 setup/hold、最小脉宽与时钟路由警告，再决定是否上板。
既有 camera_pclk_d PR1014 和负脉宽裕量是尚未解决的独立余项，本次没有放宽时钟周期或屏蔽它们。
`manifest.json` 与 `SHA256.json` 可核对完整源文件及工程一致性。

若使用仓库根目录的标准构建工具，先执行 `python projects/mecanum-pipeline-opt-20261103/prepare_reference.py`，再按 `AGENTS.md` 的整机增量命令运行。
