# 断电后重新下载 PWM 诊断固件

1. 接好 FPGA 逻辑供电和 Gowin USB/JTAG 下载线。
2. 在仓库根目录双击 `download_pwm.cmd`。需要本机 Python 和 Gowin V1.9.12.04。
3. 只有显示 `SUCCESS` 才算下载完成。失败记录保存在 `runs/commissioning/manual_download_时间/programmer.log`。脚本会自动寻找 USB location，不依赖原来的 290 或 354。
4. 下载不会启动电机。打开 PWM 控制台后，再自行点击单次测试。

这是 **SRAM 下载**：断电后丢失，重新双击即可；不写 Flash。固件为 M1/M2 纯 PWM 诊断，10kHz、最高20%、单次最长5000ms，不包含视觉或 IMU 功能。保留整个仓库和 `build/pwm_probe/impl/pnr/motor.fs`，不要单独移动脚本。

也可在 Gowin Programmer 中手动操作：扫描下载器，选择 GW5AT-138B，将操作设为 **SRAM Program**，文件选择本仓库下 `build/pwm_probe/impl/pnr/motor.fs`，然后执行。不要选择 Flash 操作。应看到成功完成信息；若扫描卡住或报错，先检查 FPGA 供电、JTAG 插头和下载线，关闭其他 Programmer 实例再重试。

当前 UART 是 COM13，拔插后可能变化。它与 JTAG 下载器是两个设备；串口连上不代表 JTAG 可下载。启动控制台示例：

```powershell
python host/pwm_console.py --live --serial-port COM13 --port 8874
```

控制台地址：http://127.0.0.1:8874/ 。已有控制台运行时不要重复启动。

新版时长可输入100～5000ms，步进1ms，固件单次硬上限5秒；需下载新版固件（flags=0x22）。

慢衰减修正：测试时IN2固定高，IN1低电平为01驱动，高电平为11刹车（手册反向）。占空比表示有效驱动时间，20%对应IN1低20%、高80%。启动先11唤醒约1.1ms，计入单次总时长；停止、0占空比和复位输出00。引脚回读统计01驱动区间。上限5秒、20%、单轮及看门狗保持。不要外接拉高线。
