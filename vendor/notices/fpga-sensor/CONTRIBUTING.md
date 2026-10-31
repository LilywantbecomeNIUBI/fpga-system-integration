# 传感器工程 Git 开发约定

后续在本仓库开发；`main` 保留通过检查的版本。原工作区 `imu_1khz/` 和 `imu_bringup/` 为历史实验目录，不再通过手工覆盖同步。

```powershell
git switch main
git pull --ff-only
git switch -c codex/your-change
# 按实际修改选择测试；改公共接口时运行两版
python tools/project.py check
python tools/project.py test all
# 修改 RTL、约束或工程配置后，运行对应 Gowin 构建
git add <明确的相关文件或目录>
git diff --cached --stat
git diff --cached --check
git commit -m "feat: describe the change"
git push -u origin HEAD
```

通过 PR 合并到 main。需要固化可交付版本时创建新标签/Release；记录源码提交、工具版本、位流 SHA256、时序和实板条件。历史 `baseline-20261016` 及 `evidence/original/` 保持原样，不能拿旧验收结果证明新功能。

提交 HDL、传感器模型、测试源码、CST/SDC、Gowin 工程、脚本、说明与小型结果摘要。`tests/` 是测试源码，必须纳入 Git。`results/`、`impl/`、`.deps/`、位流、波形和原始采集数据不进入源码历史，交付附件通过 Release 保存。不要提交 token、口令或个人凭据。

`python tools/project.py check` 适用于正在开发的工作树；`check --baseline` 用于核对旧 Release 的输入；`check --artifacts` 还要求恢复原始附件并验证其哈希、回放 UART。源码变化后应重新构建，下载脚本会拒绝将不匹配的源码和旧位流混用。

未接硬件时可报告仿真和构建通过，不能报告实板通过。采集发布率、芯片 ODR、调试帧率、观察到提交耗时和完整运动延迟要分别记录。
