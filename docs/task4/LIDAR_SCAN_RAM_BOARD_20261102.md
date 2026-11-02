# 雷达 RAM 重构：M2 静态布线与上板记录

2026-11-02，Asia/Shanghai。用户授权“开始布线，板上验证”，继续先前选定的 M2 静态诊断范围。

## 实际运行的版本

- 顶层：`m2_sensor_diag_top`，物理引脚配置 `task4_m2`，器件 GW5AT-LV138PG484AC1/I0，系统时钟 50 MHz。
- 隔离工作目录：`E:/Gowin_project/fpga_project/tmp/m2_mpu_20261102T063953Z`；构建目录 `build/m2_sensor_diag_ram_20261102`。
- 在此前冻结的 M2/MPU 诊断源码中仅换入新的 `lidar_scan_source.sv`，SHA256 `dbcc9acebbaea2de3cd6cc682ebe85a53a055c274c6ae23a06980fddcdad9b3a`。没有混入其他聊天正在开发的轮控修改。
- 电机十二路物理输出均为常量低电平，没有命令接收入口，不包含摄像头、图传或运动导航功能。
- 实际串口重新枚举为 **COM25 / CH340**，JTAG USB location 为294，扫描 ID `0x0011081B`。上位机与验证脚本从旧 COM12 改为 COM25；上位机只保留 STOP/release 操作。

## 构建与下载

- Gowin V1.9.12.04，Place0 / Route2，运行约 **98.24秒**，完成位流与时序报告。
- Setup / Hold 违例 **0 / 0**；最差 Setup 裕量 **+6.543 ns**；最小脉宽裕量 **+7.317 ns**；没有 PR1014 时钟布线告警。
- 布线后时序摘要 Fmax **74.312 MHz**，关键路径12层。此结果属于静态诊断顶层，不能代替完整视觉/导航融合顶层的布线结果。
- 全顶层综合：8,329 LUT、11,631寄存器、5块BSRAM；其中雷达显示子模块含CRC为1,494 LUT，缓存确认为BRAM。
- 新版静态顶层仿真：真实雷达解析器接收108包及1个故意损坏包，72扇区输出完全正确；MPU行为模型、编码器转动、混合UART CRC和全部十二路电机输出固定低检查通过。
- **16:51 成功下载到 SRAM**，没有写 Flash。位流SHA256：`b91785a8b5af9994b388d1ba0d5b43ee5ad76a2a8b700cd96d134440fcd84074`。

构建、引脚与集成仿真证据：[pnr_result.json](../evidence/lidar_scan_ram_20261102/pnr_result.json)、[pin_audit.json](../evidence/lidar_scan_ram_20261102/pin_audit.json)、[static_rtl_test.json](../evidence/lidar_scan_ram_20261102/static_rtl_test.json)。

## 真实雷达与串口验证

下载后只接收 COM25 数据，不发送运动命令。首次10秒接收296个雷达分块，成功重组49圈完整扫描，每圈72个扇区；串口混合帧CRC错误0，导航状态帧CRC错误0。采集从当前运行中的扫描开始，首个未完整收到的圈按不完整扫描记录，未用它刷新显示。

随后通过上位机实际接口连续观察约44.78秒，共86个观察点：

| 项目 | 结果 |
| --- | ---: |
| 新增完整上传扫描 | 223圈（约4.98 Hz） |
| 雷达原始包增量 | 80,600 |
| 雷达CRC错误增量 | 0 |
| 雷达串口帧错误增量 | 0 |
| 混合/导航遥测CRC错误 | 0 / 0 |
| 雷达在线观察点 | 全部 |
| 最大显示数据年龄 | 约259.96 ms |
| 电机状态 | 全程未解锁，两轮输出为0 |

初始雷达接收计数中的3次帧错误没有在该稳定性观察窗口内增加。原始整圈约10 Hz、上传限速约5 Hz，因此 `skipped_scans` 正常递增，不能据此判定UART丢包。

编码器遥测已收到，但实际车轮保持静止、计数为0；尚未验证手转时的计数方向或动态响应。

证据：[board_capture_result.json](../evidence/lidar_scan_ram_20261102/board_capture_result.json)、[live_stability_result.json](../evidence/lidar_scan_ram_20261102/live_stability_result.json)。

## IMU 未通过及进一步探测

主诊断固件在0x68收到持续I²C NACK：`ready=0`、`fault=1`、`WHO_AM_I=0`、样本序号0。收到的是每秒一次的故障状态帧，没有有效六轴样本。初次上板脚本的 `static_uart_passed=false` 与错误记录原样保留；雷达通过不等于全部传感器通过。

用户确认模块已安装供电，并提供参考工程：`D:/活动/比赛/电赛/FPGA/团队工程/STM32F103_Balance_交接_20261102/STM32F103_Balance`。该工程最终实测身份为0x70、地址0x68，代码明确支持MPU6500；早期MPU6050名称不能用于确认当前器件身份。

另直接核对 `D:/活动/比赛/电赛/FPGA/Carrier_Hall_RevB/pin_netlist.csv`、CST及IMU连接说明：转接板B的J_IMU1-1～4为3.3V、GND、SCL、SDA，SCL/SDA对应P7-39/F19及P7-40/F20，与本次位流一致。插座后四针原为ICM模块设计；新模块的实际丝印针序仍需实物核对。

为区分速率、地址与身份问题，额外制作临时只读身份探测固件：100 kHz、交替读取0x68/0x69的0x75寄存器，不进行配置写入。该固件仿真确认可以原样报告0x70身份及地址NACK；布线Setup/Hold为0/0，十二路电机输出仍全为GND。

临时SRAM下载后取得30条CRC正确诊断帧：**0x68及0x69均NACK，SCL/SDA空闲读值均高**。尚未读到身份，因而没有仅通过放宽WHO判断或更改地址来掩盖故障。已向用户询问实际模块八针丝印顺序，待核对插装与电气连接。

探测后已重新下载并恢复上述雷达RAM静态诊断位流，上位机也已恢复。临时探测未写Flash。

用户随后提供模块实装照片，丝印可辨识为从VCC端依次：`VCC / GND / SCL / SDA / XDA / XCL / AD0 / INT`。前四针的功能定义与转接板B一致；后四针对应关系为：

| 插座针号 | 当前模块标识 | 转接板B网络 |
| --- | --- | --- |
| 5 | XDA | IMU_AD0，经10k上拉 |
| 6 | XCL | IMU_CS_H，经10k上拉 |
| 7 | AD0 | NC，底板未连接 |
| 8 | INT | IMU_INT2_UNUSED，100k弱下拉 |

因此底板原有的AD0上拉实际接到了模块XDA，模块真正的AD0没有由底板固定电平。此前资料中“模块自带AD0下拉”的说法仍缺少该实物模块电路或测量确认；不能直接据此认定地址已固定。建议断电后将**模块标有AD0的第7针**接至模块GND第2针，再上电重新下载SRAM诊断固件并观察应答。该项是下一步排查，不把未应答根因断定为AD0悬空；照片也不能证明排针接触、焊点及模块芯片供电正常。

AD0低选择0x68的依据为[MPU-6000/6050官方规格9.2节](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf)；MPU6500的0x68/0x69与WHO身份区别见[官方寄存器手册4.38节](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf)。

用户表示不便接跳线，后续以尽量保持模块直插为约束；上面的接地建议是可选诊断，不能据此认定必须改线或换转接板。为继续进行无需改线的排查，又制作只读探测版本：I²C降到10kHz，事务超时相应延长为20ms，F19/F20启用内部弱上拉。该版本通过行为模型和物理时序检查，十二路电机输出仍全为GND。真实采集得到30条CRC正确记录，0x68/0x69仍均NACK，空闲SCL/SDA均读高；未取得芯片身份。随后再次恢复原RAM雷达诊断位流和COM25上位机。弱上拉仅用于排查，没有修改正式静态固件的引脚上拉配置。

补充证据：[imu_pullup_probe_board.json](../evidence/lidar_scan_ram_20261102/imu_pullup_probe_board.json)、[imu_pullup_probe_pnr.json](../evidence/lidar_scan_ram_20261102/imu_pullup_probe_pnr.json)、[programmer_pullup_probe_restore.log](../evidence/lidar_scan_ram_20261102/programmer_pullup_probe_restore.log)。转接板的雷达功能已正常；当前没有证据认定必须更换整块板。IMU仍需实际核对供电、电气连通和AD0电平，不能把软件诊断结果写成已修复。

用户随后实测报告：模块AD0为0V。由此当前静态AD0状态应选择0x68，不再建议为了排查先给AD0另接地线，也不把AD0悬空作为当前故障结论。该测量尚不能判断总线传输期间的电平变化；现阶段优先核对模块VCC相对其GND的实际电压、模块端SCL/SDA空闲高电平，以及模块到转接板/FPGA的连通情况。此前0x68/0x69均NACK的探测结果仍保留。

用户进一步怀疑更换模块后的上拉配置，并报告单片机直连可达到1MHz。核对RevB原装配表：SCL/SDA各自默认2.2k上拉，经各自0Ω使能供电，4.7k并联选装支路默认不装；实际焊接与模块自带上拉尚未实测。若模块同电源域也有2.2k，上拉总等效约1.1k；如选装支路也安装，总值还会降低，必须结合从机低电平能力和实际波形判断。启用FPGA内部弱上拉只增加并联支路，不能减弱外部过强上拉，因此此前10kHz测试**不能排除外部上拉过强、错装或连通问题**。不因其原用于ICM就断定2.2k不适用于MPU。

所提供F103参考工程的当前`Core/Src/board.c`配置为`hi2c2.Init.ClockSpeed = 400000`；这仅代表该文件版本，不否定用户另行完成的1MHz测试。MPU6500官方额定I²C为400kHz，先以低速应答及400kHz通信为排查基准，不用1MHz设定替代实际波形验证。依据：[TI上拉计算说明](https://www.ti.com/lit/pdf/slva689)、[TDK MPU6500产品说明](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6500-datasheet2.pdf)。

用户新增电压报告：SCL=2.7V，SDA最初报告3.28V、随后更正为3.18V；模块VCC电压仍待报告。不能直接把万用表SCL读数等同波形高电平，也不能仅据2.7V断定逻辑电平不合格或上拉阻值错误。

随后用户补充模块VCC对GND为**3.283V**。当前已知实测为VCC=3.283V、AD0=0V、SCL=2.7V、SDA=3.18V；VCC输入接近预期3.3V，但尚未测得小板稳压后的芯片供电。重点转为确认SCL是否在无时钟且FPGA高阻释放时仍低于VCC，以及其上拉支路、连通和模块负载；不因SCL=2.7V就直接判定上拉阻值错误。

用户随后报告断电在路电阻：SCL到VCC约70Ω、SDA到VCC约2.2kΩ。两路明显不对称；若70Ω来自实际上拉通路，则会明显增加FPGA拉低SCL的负载，但在路测量尚不能区分电阻错装、焊桥、模块或FPGA内部半导体并联通路。已要求先保持断电，暂停进一步主动I²C测试，优先做隔离测量。

用户提出停用SCL上拉并询问位号。依据原始BOM、网表和PCB，建议隔离点为 **`R_IMU_PU_EN1`（0Ω，0603，PCB坐标X=42mm/Y=56mm）**，它串在3.3V与SCL上拉供电网之间；拆下可同时隔离`R_IMU_SCL1`（2.2k，X=34/Y=60）与选装`R_IMU_SCL_OPT1`的底板供电。SDA的`R_IMU_PU_EN2`（X=42/Y=59.5）保持不动。需断开USB及全部供电后操作，未确认用户已经拆除；建议拆后先在断电状态复测SCL到VCC。若仍约70Ω，继续隔离模块/FPGA通路，不能将拆除上拉写成已修复。

后续用户补充并明确确认：**蓝色模块已经从转接板拔下、完全断电，单独模块测得SCL对GND约80Ω、SDA对GND约2.2kΩ。** 这是与先前“对VCC”不同的测量，不混用。该低阻读数的通路属于模块内部，不能再归因于底板上拉配置；SDA对GND的2.2kΩ也不直接等于其上拉阻值。暂停进一步底板改动及主动通信。建议在断电且模块独立状态，用电阻档交换红黑表笔复测SCL/GND并等待读数稳定；若双向均稳定几十欧，优先检修模块SCL支路、焊桥或芯片漏电。尚无双向读数，不能仅凭单向80Ω直接确认芯片损坏；未确认底板使能电阻是否已拆。

该阶段发现主上位机IMU状态年龄已超过30秒、雷达离线，之前的NACK不再是当前实时状态。COM25和JTAG294仍在，随后再次下载SHA256相同的已验证RAM静态诊断位流，Programmer确认成功，但后续5秒COM25读取为0字节。上位机已重新启动；当前没有新的UART结果，不能把此次恢复写成实时雷达/IMU已正常。先前49圈和223圈稳定性测试是历史真实记录。当前等待VCC对GND电压，以继续核对供电、复位及串口连接。恢复证据：[voltage_measurement_recovery.json](../evidence/lidar_scan_ram_20261102/voltage_measurement_recovery.json)、[下载日志](../evidence/lidar_scan_ram_20261102/voltage_measurement_recovery_programmer.log)。

证据：[imu_identity_probe_board.json](../evidence/lidar_scan_ram_20261102/imu_identity_probe_board.json)、[imu_identity_probe_pnr.json](../evidence/lidar_scan_ram_20261102/imu_identity_probe_pnr.json)、[programmer_probe_restore.log](../evidence/lidar_scan_ram_20261102/programmer_probe_restore.log)。

## 当前上位机与待测项

- 雷达/导航诊断页面：`http://127.0.0.1:8876/`；主诊断页面：`http://127.0.0.1:8872/`。
- 两个页面由同一进程接收COM25，不争抢串口；雷达使用真实设备数据。
- 模块八针丝印已按照片辨识；待实物确认AD0电平、IMU通信及手转编码器方向。没有执行ARM、DRIVE、任务启动、车轮电动试转或自主避障。
