# IMU对照与直连证据

- `imu_i2c_audit_20261102`：原F19/F20独立只读探测的布线、仿真和实板结果。
- `imu_i2c_swap_20261102`：仅交换软件映射的探测结果；没有改变实物信号线。
- `m2_sensor_diag_mpu6500_20261102`：兼容MPU6500后的原引脚静态固件结果；`latest_board_result.json`是恢复该位流后的真实接收，IMU仍未通过。
- `direct`：已验证但尚未下载的F16/E17直连静态候选。包括全部RTL源码、CST/SDC、Gowin项目、构建Tcl、仿真测试台及结果。

`direct/source`、约束、项目与Tcl逐文件核对了原构建清单的SHA256。清单中的路径保留原隔离工作区相对路径，结果中的时间、来源路径和`programmed=false`等字段为各次构建时的历史记录；是否实际下载以相应实板记录和说明为准。未上传位流、串口二进制采集、会话令牌或运行进程配置。

重新布线时，在`direct`目录执行：

```powershell
& 'C:/Gowin/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe' build.tcl
```

项目使用GW5AT-LV138PG484AC1/I0，顶层`m2_sensor_diag_top`。此命令只构建，不下载板卡。主工程的生产顶层与此静态快照分别维护；不能将这里的布线结果当作当前完整融合工程的时序签核。
