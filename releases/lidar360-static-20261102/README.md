# 已实板验证的 360 点静态版本

此目录固定当前已下载并收到扫描数据的原始版本。全量点实现保留在主工程中，本目录不修改它。

- 360 点/圈、约 10Hz、保留实际角度；全部状态使用 UDP 6205。
- M2 引脚配置，IMU 为 B 载板插座 F19/F20；雷达 TX 为 M2-14/B16。
- 十二路电机输出固定低，无运动命令接收入口。
- 不包含图传、光流或运动导航。
- Setup/Hold=0/0，最小脉宽裕量 +0.021ns。
- 实板网口接收正常，360 个有效点持续更新；未写 Flash。
- 当前 IMU 的 WHO_AM_I=0x70，本原始固件仅接受0x68，尚无有效六轴样本。后续修复单独发布，不修改此版本。

`sensor.gprj` 与 `source/` 是独立冻结工程，可用 Gowin V1.9.12.04 打开。`build.tcl` 只构建，不下载。

`firmware.fs.gz` 为原位流的无损压缩包，避免将35MB文本直接存入Git。解压示例：

```python
import gzip, hashlib
from pathlib import Path
data = gzip.decompress(Path('firmware.fs.gz').read_bytes())
assert hashlib.sha256(data).hexdigest() == '63dc4e2b3ae47e2a98eaa6d465578207e9ba09fa28acbed128029ac23a67d49e'
Path('firmware.fs').write_bytes(data)
```

原始仿真台和结果保留作证据；其中夹具路径指向原构建目录。`SHA256.json` 覆盖本目录全部交付文件（清单自身除外）。
