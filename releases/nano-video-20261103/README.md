# nano-video-20261103

本版本为后续默认Nano基线，已在设备固定目录中启用并设只读。

- `source.zip`：从实际Nano部署取得的公开源码与示例配置，内含`MANIFEST.json`。
- `BASELINE_MANIFEST.json`：完整部署（含模型与本机配置）的SHA256、公开与私有包SHA256。
- `verification.json`：76个实际文件校验、只读状态与两项systemd服务固定路径核对记录。
- 完整私有恢复包保留Nano及PC本机，未加入Git。

运行入口、校验、开发约定和回退步骤见 `../../docs/NANO_BASELINE.md`。

切换固定目录后复测20秒：原图27.84 FPS（558个唯一帧键）、识别27.65 FPS（555个唯一帧键），见`delivery.json`；API版本和视频在线见`api_verification.json`。该版本未宣称持续满30 FPS。
