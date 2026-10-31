# 精简多点 LK 跟踪试用版

该版本以多点 LK 替换 `image_v23` 的中心模板 SAD 匹配。保留现有摄像头、双路图传、YOLO 原图 frame_id、16帧特征历史、语义代际门控、颜色融合和独立网络时钟看门狗。电机使能保持关闭。

## 当前实现

- 复用 `flow_lk_patch.v` 的定点 Q7 求解器；在原有80×45灰度/红色特征网格上运算，一个网格对应原图16×16像素。
- 每个目标框4×4采样，最多16个互异且不越界的点；无纹理或病态张量点由LK核拒绝。不是Shi-Tomasi角点检测。
- 每点前向3次残差迭代、反向3次迭代；前后向位移和每轴≤96/128网格像素才保留。反向起点取整，因此这只是定点近似检查。
- 分别排序水平/垂直位移，取上中位数；每轴与中值距离≤1网格的点构成一致集。要求至少4点且至少占保留点的一半。
- 只更新平移，保持框大小；累计亚像素余数，发布框仍为原图整数坐标。跟踪质量是有效一致点数/16的硬件诊断分数，不是目标概率。
- 稀疏点少于8个时请求YOLO刷新；不足4个或历史不完整时明确无效。低有效点使用原reason=8；reason=4表示运动共识不足。
- 晚到检测先锚定自己的历史帧，再依次处理每个相邻帧对。坏帧、覆盖、越界、语义过期和无目标仍失效。

这不是完整OpenCV MedianFlow，没有金字塔/NCC/尺度估计；也没有接入独立光流避障状态机。低纹理和细小目标仍可能失败。特别是80×45特征平均会丢失细节，本版优先复用已有历史缓存来尽快试用，不声称其精度等同160×90金字塔光流。

`src/vision/vision_lk_tracker.v` 提供生产接口 `vision_tracker`。旧 `src/vision/vision_tracker.v` 的模块更名为 `vision_sad_tracker`，仅保留历史回归与对照；完整视觉工程仅包含新LK实现。

## 构建与试用

```powershell
python scripts/test_vision_lk_tracker.py
python scripts/build_vision_lk.py --core
python scripts/test_vision_lk_tracker.py --netlist build/vision_lk_core/impl/gwsynthesis/vision_lk_core.vg
python scripts/build_vision_lk.py
python scripts/program_vision_lk.py
```

`build/image_lk` 与原冻结位流独立。下载脚本要求当前源码哈希、位流哈希、核心网表验证和完整工程setup/hold 0/0全部一致，检查器件ID后仅写SRAM。原冻结manifest不重写，旧下载入口不会把新源码误当成旧冻结版本。

PC/Nano仍使用原YOLO入口，无需更新模型或协议。确认瓶子完整入镜、尽量有瓶盖/标签纹理，先静止再缓慢移动，观察FSTA的track_valid、track_confidence、frame_id和semantic_frame_id；透明瓶或素色瓶身可能缺少可靠点。红色target_valid和几何track_valid分别判断。

## 验证记录边界

必要RTL回归覆盖历史回放、亚像素平移、左右反向、低纹理失效、非红目标排除、坏帧、无目标清除、历史覆盖、语义有效期和帧号回绕。实际Gowin网表使用厂商原语检查一个完整16点历史帧对的前后向LK、中值及平移结果；没有重新进行大型数据集对比。

第一轮板级布局布线有59个setup违例、hold 0，未下载。STA指出ROI选点计算到状态判断的组合路径，以及原网络逻辑在新布局下的路径超时；增加选点判断流水并调整扇出后重新构建。旧失败不覆盖为成功。仿真测试台初版遗漏厂商全局复位驱动导致超时，修正后按新测试台重新检查。

当前构建、网表与程序下载事实以 `build/image_lk/result.json`、`tests/vision_lk/netlist_result.json` 和 `build/image_lk/programmed.json` 为准。未生成programmed记录时，不能宣称LK版已经上板。

修正后的完整工程（max_fanout=32、place=2、route=1）已通过setup/hold 0/0；位流SHA256为 `503de65c9be6719a3b27b94a55522f03833a16677384a4084f9ae4ae1aa9762f`。PnR资源为Logic 13226/138240、寄存器10523/139095、BSRAM 185/340、DSP 30/298。该数据属于本次构建，不是已有光流避障位流的资源，也不是相机外部时序裕量证明。

## 2026-10-24 首次上板试用

23:56:43下载上述LK位流到SRAM，未写Flash；恢复原YOLOv8n TensorRT FP16与PC页面 `http://127.0.0.1:8767/`。静止瓶子收到track_valid=true、track_confidence=1000（16点一致）；红色比例不足门槛时target_valid=false是预期属性结果，不能等同几何跟踪失败。

随后31.77秒累计计数测量：PC收帧29.995 fps、Nano收帧29.964 fps、推理27.414 fps，双方各跳过1个完整帧；匹配回执帧龄P95为2帧，拒绝增量0。292个轮询唯一事件状态中，282个track_valid=true；reason为正常289、历史失效1、代际切换2。这里统计的是约10Hz轮询采到的事件，不能写成完整摄像帧覆盖率、跟踪准确率或与旧SAD相比的提升比例。

该窗口292个观测推理结果都有超过50%门槛的瓶子，收到30种有效语义框。没有运动真值，不宣称精准跟随或遮挡恢复已验收。服务与新LK位流保留运行，电机输出保持关闭。小型验证摘要见 [validation/vision_lk_trial_20261024.json](validation/vision_lk_trial_20261024.json)，原始日志及采样在 `runs/lk_board_trial_20261024/`。
