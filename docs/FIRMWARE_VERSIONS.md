# FPGA 固件版本索引

版本以不可移动的 Git 标签保存；新的试验创建新标签，不覆盖稳定标签。
最新时序优化分支：`codex/timing-closure-20261103`。

麦轮源码集成分支：`codex/mecanum-integration-20261103`，从 `timing-refactor-20261103` 创建独立 worktree。
这是待整机时序/实板验证的源码版本，不替代下表的稳定传感器固件；说明见 [麦轮集成记录](MECANUM_INTEGRATION_20261103.md)。
后续离线检查点 `mecanum-raw-setuphold-20261103`：队友麦轮引脚、全点雷达，Setup/Hold 0/0；原重试端点 +0.275 ns。
PCLK 的 MPW/PR1014 仍未通过，未上板；详情见 [全点时序迁移](RAW_MECANUM_TIMING_20261103.md)。

| 标签 | 内容 | 验证/用途 |
|---|---|---|
| `lidar360-static-20261102` | 原始 360 点静态传感器基线 | 实板雷达通过；旧 IMU 识别逻辑不支持当前 WHO=0x70 |
| `lidar360-mpu6500-20261102` | 360 点＋MPU6500 静态传感器 | 稳定恢复版本；时序和实板并发验收通过，无图传/光流 |
| `experimental-360-routing-20261102` | 整机 Bank 写入流水 | 实验版本，setup 881、MPW/PR1014 未通过；曾下载 SRAM，不能视为稳定版 |
| `frame-read-pipeline-20261103` | 每 Bank 读出寄存器 | 离线实验，读出两段 setup 通过；整机统计 550，未下载 |
| `timing-refactor-20261103` | 计数器分段、事件缓冲、Bank 地址、雷达/VFH 和局部发送控制 | 最新离线实验，统计 6；4 条数据 setup、2 条复位释放，MPW/PR1014 仍未过；未下载 |

两个最新标签按上述顺序形成提交历史，直接承接 `experimental-360-routing-20261102`。
对应的 `releases/<标签>/` 提供独立 Gowin 工程、实际冻结源码、压缩原位流、时序报告和哈希清单。
包内 `.gitattributes` 禁止换行转换，以保持冻结文件的原始字节。

恢复硬件时优先使用已通过实板验证的 `lidar360-mpu6500-20261102`。下载前核对解压后的位流 SHA256，SRAM 下载断电后不会保留；本轮版本管理没有进行下载或写 Flash。

共享工作区使用独立 Git 索引生成这些提交，未切换当前工作区分支，其他未提交工作保留。
最新实现与残余问题详见 [时序根因核查报告](TIMING_ROOT_CAUSES_20261103.md)。
