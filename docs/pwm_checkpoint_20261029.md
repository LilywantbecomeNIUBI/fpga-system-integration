# 2026-10-29 PWM 电机实测存档

用户在慢衰减固件下载后确认：“现在电机测试正常”。该反馈为现场观察，未记录本次具体通道、占空比和时长，不扩展为闭环或整车验收。

- FPGA：ACG720 / GW5AT-138B，纯PWM诊断固件 flags=0x22。
- 驱动：AT8236，IN2=1、IN1低有效；01驱动、11慢衰减刹车，启动11约1.1ms，STOP/复位00。
- 10kHz，单轮0～20%有效驱动占空比，100～5000ms；无PI或编码器控制。
- SRAM镜像 SHA256：387c4a8c8664bfd6e60ed290146fbb560ec9f4e43a266840a725d7f26963f8c6。
- Gowin构建通过，Setup/Hold违规均0；9项控制台测试及两项RTL仿真通过。
- 本地下载日志：runs/commissioning/manual_download_20261029_224533/programmer.log，Finished确认。
- 下载入口：download_pwm.cmd；控制台和操作说明见 pwm_console.md、manual_pwm_download.md。

构建文件和原始日志在Git忽略目录中，本提交存档源代码、测试及说明。新克隆需运行 python tools/build_pwm_probe.py 构建；下载脚本固定校验上述已验证镜像的哈希，不同工具版本构建若哈希变化需先复核，不能直接跳过校验。
