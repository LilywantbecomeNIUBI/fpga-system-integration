# 麦轮 IO 电气约束修正与实板检查点

构建 20261102T204810Z，完整继承队友 IO_PORT 电气设置。Setup/Hold 0/0；PCLK MPW/PR1014 仍在。
已下载 SRAM，IMU WHO=0x70 并校准、ARM/STOP/看门狗和架空短时前进通过，图传/光流在线。
运动中累计 4 次可恢复 I2C NACK；雷达未接暂缓；落地精度未验，完整硬件验收为 false。
详见 ../../docs/MECANUM_BOARD_20261103.md。解压 sensor.fs.gz 后按 result.json 校验 SHA256。
