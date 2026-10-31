# 有刷电机调参接续入口（2026-10-30）

> Nano移交已完成：CH340稳定路径/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0；192.168.50.20:8873网关PID19035，PC8874已连接该网关，纯PC串口服务已退出。Nano Python3.6.9启动兼容已修复为shared/imu_protocol.py无GUI解析器，冻结vendor未改，41项Python测试通过；Nano31个源码文件哈希校验及IMU解析smoke通过。持续约20Hz、CRC0，FPGA累计bad_frames=5但观察无新增，左右diagonal无新增，未ARM/零速度零输出。未重载SRAM，D4/A2和旧计数连续。当前仅PC有线→Nano网关已验证，PC仍BIT-Mobile校园Wi-Fi，纯无线和下地未验收。本次未发送运动指令。12V最后按移交要求关闭，尚未收到再次恢复的确认。当前设备liwei ACL重插可能失效。证据hardware/wireless_tuning_20261030.json。

> 当前接续：COM12已恢复，上拉4840e1cb…静态99帧CRC0；手转每圈约1500计数，左右独立diagonal新增0；双轮±8目标、1秒初步PI候选Kp4096/Ki128，详见MOTOR_PULLUP_PI_20261030.md。随后用户电池掉电、开始向Nano移交UART；当前板上状态须用新鲜遥测重新确认。PC串口网关已关闭，8874已改连接Nano 192.168.50.20:8873，新主页/home、调参/motor、前后按住短测默认4/最多1秒。Nano独立代码部署在/home/liwei/robot_motor_ground_20261030，30文件及28个pyserial源文件哈希校验；尚未启动网关，Nano尚未检测到CH340，PC仍列COM12。未作无线或下地验收。详见WIRELESS_TUNING_20261030.md；不要用下面旧状态启动运动。

> 09:01最新：用户示波器报告FPGA编码器输入Vmax约2.2V，要求四路AB内部上拉。tools/build_motor.py已将四路DOWN改UP，其余约束与归档逐行一致；候选Setup/Hold0/0，SHA256 `4840e1cb55d889e5d31af4e5b1a8b21ed4bb25e033cee34e16b07c649ac2d415`已SRAM Program Finished，Flash未写，但静态仍0帧/0UART字节，一圈验证未做。左D4/右A2控制算法未改。证据hardware/encoder_pullup_20261030.json；最后下载尝试以runs/commissioning/last_motor_sram_attempt.json核对，active_motor_sram.json仅为上次静态成功记录。12V最后确认OFF，未发ARM/DRIVE。

> 08:55最新：7ffbafff…恢复候选已上板并完成左轮双向与1秒PI比较。左轮初始候选Kp4096/Ki128（Q8），±8目标末段约+7.17/−7；右轮仅正向短测，反向和PI未完成。之后COM13丢失数据，用户断12V后重载同一位流成功但静态0帧，当前无新鲜遥测，未ARM/DRIVE；12V最后确认OFF。不要用旧静态成功记录或旧串口快照启动。详见[当前结果与阻断](ENCODER_RECOVERY_20261030.md)，参数摘要hardware/tuning_progress_20261030.json。

> 08:40后续接续：有序AB记录版621cb4a0…已写SRAM；两方向实录分别213/110跃迁，均完整、无CRC错误，固定A/B各在一个方向抵消。方向连续性候选已通过9组RTL（含实录回放）与38项Python测试及Setup/Hold0/0，候选SHA256 `7ffbafff2425a15cadd8e39d38b74f8e251c5ce5ccde4ae1270746a97589c20c`，等待现场断12V后下载，尚未宣称上板。详见[有序实录与恢复候选](ENCODER_RECOVERY_20261030.md)。当前STOP、未ARM、零输出，PI尚未整定；实际已下载身份以runs/commissioning/active_motor_sram.json核对。

> 后续会话更新：A相双边沿、每边沿±2的位流已下载SRAM，SHA256 `980b3747fe091b2bcc55a48c2e1c8f73132bc6b90f3f713be8069b0e0298fb1c`；按用户明确授权invalid固定0，不能再用此值判断信号质量。已做动力短测，左轮物理反向转动与净计数不匹配，PI仍未整定。最新过程、保护语义、flags3候选和停止状态见[本次解码与实测记录](ENCODER_A_EDGE_20261030.md)。下文的旧固件哈希、PID和“尚未动力测试”为此前封存状态。

当前工作仓库：`E:/Gowin_project/fpga_project/team_repos/fpga-system-integration`，分支`codex/system-integration-v1`。用户准备在另一个对话继续速度PI调参。本记录描述实际已下载的独立电机固件；不要把旧无刷fpga-balance/FOC工程或视觉系统集成固件当作当前底盘入口。

## 当前进度与边界

- D157B/AT8236 + MG513有刷电机，独立双轮速度PI已经实现，100Hz控制、10kHz慢衰减PWM。正输出01驱动/11刹车、负输出10驱动/11刹车；零目标输出、STOP、禁用及复位为00，启动11约1.1ms唤醒。
- 独立慢衰减固件已两次SRAM下载成功，未写Flash。当前运行的是**左编码器换脚版**，不是默认ADC引脚版。用户断电后曾丢失固件并于02:17重载；后续是否再次断电应重新核查，磁盘记录不是板内版本读取证明。
- 只完成静态与手转反馈检查，尚未执行本轮非零ARM/DRIVE动力测试，未整定PI，未验收落地或整车。
- 用户随后自行测试，报告左轮一圈基本在738计数附近。记为**用户报告的临时CPR参考**，未获固定圈数原始日志、重复性误差、准确型号及齿比；未写入正式标定，physical控制仍未开放，右轮CPR未标定。
- 用户曾明确未接共地，已告知D157B GND接载板ENC1-5或FPGA GND；之后左轮约半圈时右通道不再跟随。本记录不替代实际地线通断和电压测量。

## 当前接线：务必使用此表

|功能|P7 / FPGA球号|载板取线点或模块端点|
|---|---|---|
|左E1A|31 / F16|原ENC1第3脚CLK|
|左E1B|32 / E17|原ENC1第4脚DO|
|右E2A|27 / C18|原ADC RESET，J_ADC_DIG1-9|
|右E2B|36 / D19|原雷达RX，J_LIDAR_CORE1-1|
|左IN1 / IN2|21/D17、22/C17|D157B AIN1 / AIN2|
|右IN1 / IN2|23/E16、24/D16|D157B BIN1 / BIN2|
|共地|P7-12/30，或ENC1-5|D157B GND|

旧ENC1/MT6701信号必须断开；左编码器不再使用P7-25/B17、26/B18。原ADC模块数字连接断开，载板旧雷达TTL输出不能占用右编码器B。右侧磁编码器原接线与旧无刷驱动端口也不能混作本车AB反馈。

D157B到电机的6Pin防呆排线保留。ENC1五脚为1=3.3V、2=CS、3=CLK、4=DO、5=GND；编码器由D157B供电，ENC1的1/2脚不接，不把两块板的3.3V输出并联。未使用P7-29→37许可跳线。旧无刷EN/PWM/CAL均固定低。

## 固件与下载

- 当前换脚工程：`build/motor_left_alt/motor.gprj`。
- 位流：`build/motor_left_alt/impl/pnr/motor.fs`。
- SHA256：`be049edb9456adf52f59454b9aff5829289c93f32b4ee524b8d79c6b81ddd78c`。
- 左AB/右AB均LVCMOS33、Bank 2；综合布线Setup/Hold违规0，源码哈希一致。换脚没有隔离Bank供电。
- 构建：`python tools/build_motor.py --left-encoder-alt`。
- 重载：串口唯一拥有者正常退出、驱动12V断开且逻辑供电保留后，`python tools/commission_motor_static.py --port COM13 --left-encoder-alt --power-off-confirmed`。只有现场真的确认12V断开才使用该参数。
- `--verify-stop`是直接串口STOP确认的另一条下载前检查路线，不声称物理断电。当前固件已丢失、无遥测时不能用它替代断电准备。
- 工具仅写SRAM，并只发送STOP、捕获5秒静态遥测。Flash不写。下载与非零转动分开执行。
- `build/motor/`和`hardware/pins.csv`仍是默认旧ADC映射；按当前实物接线**不能误用不带--left-encoder-alt的下载命令**。系统集成位流也未改为本换脚版。

## 现有上位机与串口所有权

当前仅一个Python进程运行`runs/commissioning/speed_live_session.py`，独占COM13。它将网关8873、调参台8874放在同一进程，启动只发送STOP。封存时PID 22604，仅供定位，后续须重新查询。

- 页面：`http://127.0.0.1:8874/`；网关：`http://127.0.0.1:8873/`。
- 本地会话目录：`runs/commissioning/speed_live_20261030_021751/`。
- 最新目录指针：`runs/commissioning/speed_live_latest.txt`，恢复/新会话会变化。
- 会话令牌：该目录`session_token.json`。只在本机内存读取，不打印、不提交Git。
- 正常停止服务可用本机页面`POST /api/shutdown`（同源请求），它会关闭网关、尝试STOP并释放串口；不要强杀后同时另开COM13。
- 本地启动包装脚本及原始runs在Git忽略目录，同一电脑可读；新克隆需按`docs/system_runbook.md`使用`nano/gateway.py --config <本地配置> --live`与`host/motor_tuner.py --live --token-file <本地令牌>`重建唯一会话。本阶段配置应设置`fpga_serial=COM13`、`lidar_enabled=false`，不要恢复默认COM20或自动打开雷达。

封存时实时读取：遥测新鲜、phase=stopped、未ARM、左右速度和PWM均0、faults=1（上电待CLEAR），CRC及拒绝命令0。累计位置左−25641/右25128、累计非法跳变15035/12；这些不是新测试的增量。**驱动12V当前物理状态未重新确认**，软件STOP不等于断电。

页面“本次计数清零”只设本页位置/非法跳变显示起点，不清FPGA累计、故障或保护。刷新/失联需重新设置。CSV保留原始值和相对值；清零时不发送运动命令。

## 非法跳变的最终解释

当前RTL将滤波后AB两位同时变化（00↔11、01↔10）计为invalid，忽略该次位置增量，但后续合法状态继续计数。累计非零不等于位置全程错误，也不证明编码器损坏。

清零后1188次错误集中在约3.4s内、相邻遥测位置净变化0；此后约27.9s位置跨度8479计数而错误不再新增。实际RTL离线实验也确认：相关双位扰动可能不改变净位置；漏中间有效状态则会丢计数。所以不能忽略新错误，也不能把所有历史累计当作当前故障。

用户只转左轮的第一次约半圈曾左右同时响应：左−362/invalid+233，右+819/invalid+0。提醒共地后第二次约半圈：左−280/invalid+147，右0/invalid+0。半圈角度为估计值，不用于精确CPR。用户报告一圈约738是后续独立观察，应与这些粗略半圈区分。

保护只在ARM时检查**每10ms新增>8**，STOP期间仍更新统计基线；历史累计本身不会在下一次ARM直接作为1188次触发。门限是初版待验证值，未修改或屏蔽。20Hz遥测不能完整还原100Hz速度环的每个窗口。

资料/机理详见`docs/encoder_review_20261030.md`，换脚和下载详见`docs/left_encoder_alt_20261030.md`。厂家旧标准库TI12为四倍频、HAL TI1为二倍频，滤波设置也不同；GMR与霍尔参数不能照搬。

## 下一步：由新对话继续

1. 先核实当前接线、共地、逻辑/驱动供电和双轮悬空状态，读取新鲜STOP遥测，保证唯一串口所有权。不要因本记录存在就自动启动。
2. 若需补标定，供电稳定后设基线，做左/右独立手转与标记整圈重复性检查；738仅临时参考。不再用累计invalid单独否定位置。
3. 准备好后做单轮低目标、短时动力方向验证，核对电机输出与反馈符号形成负反馈；当前手转左前进为负，原始flags=1，电机极性尚未通过动力验证，不能直接认定应翻转哪个方向位。
4. Ki先0，调P；方向/跟踪正确后逐步加I消除稳态误差。速度环先用PI，不急加D。逐档检查饱和、超调、恢复、启停和故障，再分别验证右轮、双轮及落地带载。不要把悬空调参当作带载验收。
5. 默认PWM限幅±300‰、斜坡20‰/10ms；网关限时测试100～5000ms，到期STOP并释放控制，网关200ms/FPGA250ms失联保护。初始阈值不是实车已验证安全电流/速度。

## 本次封存验证

6组电机RTL仿真、31项Python回归、页面清零Node测试通过；研究性编码器RTL台另外通过4种输入场景。换脚版Gowin构建记录和静态下载记录已抽取到`docs/evidence/motor_checkpoint_20261030/`，不包含令牌、供应商原始包、完整原始UART或位流；完整产物仍在本机build/runs。
