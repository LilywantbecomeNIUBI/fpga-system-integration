# 全点麦轮整机离线检查点

Setup/Hold 0/0，原 retry_count2[4]/CE 端点 +0.275 ns，重试整体最差 +0.098 ns。
全局最差 +0.014 ns；MPW -0.849 ns、PR1014 仍在，完整 signoff 未通过。
采用队友麦轮引脚，LIDAR_RAW=1；未下载、未实板验证。

运行 build.tcl 可重建，GPRJ/CST/SDC 与源码均已冻结。sensor.fs.gz 为压缩的原始位流。
解压后校验 result.json 中 bitstream_sha256。所有文件哈希见 SHA256.json。
详见 ../../docs/RAW_MECANUM_TIMING_20261103.md。
