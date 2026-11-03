# 整机增量布局布线

用户已要求后续布线全部使用整机增量构建，不再从零开始。

正常入口：

```powershell
python tools/task4/build_ethernet.py --system --mecanum --raw-points --clock-cleanup --route-option 1
```

`config/pnr.json` 指定最近的完整整机参考目录；也可用 `--incremental-from <整机目录或.pr文件>` 明确选择。
构建器先冻结当前全部整机源码，再将基线 `impl/pnr/sensor.pr` 复制到新目录的 `incremental_base/sensor.pr`。
清单记录引用来源和SHA256，Tcl使用 `set_option -inc_place 0`、`set_option -inc_pnr {incremental_base/sensor.pr}`。
新RTL仍需综合，布局和布线复用参考结果；没有将它称为增量综合。

基线必须已完整结束Gowin运行，并与当前整机顶层、引脚、CST和SDC匹配。
缺少或不兼容时直接报错，不启动全量构建。参考位流可以未通过STA，因为它只用于复用实现；
新生成的位流必须重新通过自身的功能和时序检查，才能进入对应的上板流程。
完整结束且Setup/Hold与整机审计通过后，配置才自动更新到这轮结果。
PR1014及已接受的实验性MPW边界仍原样记录，`passed`不会被改写为完整STA通过。

`--prepare-only`可先生成源码与增量参考快照，不运行综合、布局布线或下载。
已用本机Gowin V1.9.12.04打开完整工程并通过 `saveto -all_options` 回读 `inc_pnr` 文件路径，
确认选项已被当前安装版本接受。

2026-11-03本机试跑发现Gowin会自行回退：旧基线匹配值为0.10、门槛80，报PA2136后进入全量流程。
该轮已人工终止；构建器现会监测PA2136/全量回退提示，自动终止仍在运行的子进程并保存拒绝原因，
即使进程已快速结束也会拒绝其结果。进程终止和记录行为已用模拟子进程回归。

用户随后明确授权本轮优化版本全量、优先速度，入口为：

```powershell
python tools/task4/build_ethernet.py --system --mecanum --raw-points --clock-cleanup --full-rebuild --place 0 --route-option 2
```

`--full-rebuild`必须来自用户明确授权，与`--incremental-from`互斥；不会由增量失败自动启用。
默认仍为整机增量，快速配置的位流也必须通过相同的Setup/Hold和整机审计。

官方依据：本机SUG100-4.4.7E的“Incremental PnR”（42–43页），以及SUG1220-2.1E的 `-inc_pnr`（44页）。
GW5AT-138B属于支持此功能的Arora V。`.pr`包含布局和布线；`.p`仅包含布局。

给队友传递冻结工程时，必须同时传递 `incremental_base/sensor.pr`。本轮参考约187MB，不能直接作为普通GitHub大文件提交；
应通过压缩包或合适的大文件传递方式携带，并在接收端核对清单SHA256。

2026-11-03，用户又明确授权本轮新增麦轮导航全量整机布局布线，生成SRAM候选；
若布线缓慢，检查并修复RTL关键路径、扇出和拥塞，再跑受影响检查。入口为：

```powershell
python tools/task4/build_ethernet.py --system --mecanum --navigation --raw-points --clock-cleanup --full-rebuild --place 2 --route-option 1
```

此授权仅覆盖本轮导航及其修正，不改后续默认增量策略；不包含下载SRAM或写Flash。
