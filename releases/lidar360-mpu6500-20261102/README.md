# 360 点 + MPU6500：实板通过版本

原稳定基线标签 `lidar360-static-20261102` 保持不变。本版本只更新基线中的两个源码文件：

- `fpga/imu/mpu6050_reader.v`：兼容 WHO_AM_I=0x70，采用已对照 STM32 的 MPU6500 初始化、滤波和14字节六轴突发读取。
- `fpga/system/imu_status_source.sv`：MPU6500 使用 config_id=6。

雷达360点数据路径、网口、引脚约束和其他硬件源文件与原基线一致。全量点代码仍保留在主工程，未处理其布线。

实板20秒并发验收通过：IMU约1000.41Hz原始读取、50.02Hz遥测，WHO=0x70、ready=1、fault/stale=0，I2C错误为0；雷达198圈、每圈360点；遥测无新增丢失/拒绝/过期，电机未ARM且输出为0。

Setup/Hold=0/0，最小脉宽裕量+0.021ns，无时钟布线告警。下载SRAM，未写Flash。当前未进行传感器零偏与安装轴向标定，不自动控制电机；此静态版仍不含图传/光流。

引脚：IMU为B载板原插座 F19=SCL/F20=SDA；雷达TX接M2-14/B16，共地M2-2。页面：http://192.168.50.20:8872/

独立Gowin工程为 `sensor.gprj`。原位流已无损压缩为 `firmware.fs.gz`，解压后SHA256必须为：

`2d5f71660757ec199d7dff6177f4fb04c7fc81093cb1cbaf7da58807f7653de7`

可用 `gzip.decompress(Path('firmware.fs.gz').read_bytes())` 解压。`SHA256.json` 记录交付文件哈希；仿真和实板验收记录单独保存。
