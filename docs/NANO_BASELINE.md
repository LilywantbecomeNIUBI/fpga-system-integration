# Nano 默认固定版本

后续默认使用 **nano-video-20261103**，Git 标签 `nano-video-20261103`。不要再依据目录日期或旧PC图传记录判断版本。

- 网页：http://192.168.50.20:8872
- 运行源码、模型、engine、配置：`/home/liwei/robot_releases/nano-video-20261103`，已设只读。
- 日志与运行数据：`/home/liwei/robot_runtime/nano-video-20261103`，可写，不属于冻结内容。
- `robot-web`、`robot-vision` 已通过 `90-frozen-baseline.conf` 启动覆盖项指向固定目录，均为 enabled / active。
- `/api/state` 的 `software_version` 必须为 `nano-video-20261103`。
- FPGA第二端点为Nano `.20:6102`，共享遥测为UDP6205；板端端点不是磁盘快照的一部分，下载其他固件或配置后仍需核对。此版本不包含FPGA位流。

冻结包、逐文件SHA256、服务核对证据位于 `releases/nano-video-20261103/`。源码包来自Nano实际部署文件，不是将当前存在其他未提交改动的工作区整体打包。原部署 `/home/liwei/robot_system_web_20261102` 保留，已不被默认服务使用。

## 后续开发约定

此版本作为默认基线。开发在工作区进行；需要新版时使用新的版本号和独立部署目录，重新验证再切换服务。不要修改固定目录、清除只读属性后覆盖它，或用旧的“更新原目录”脚本判断当前服务是否升级。版本固定仅保护此应用的代码、配置和模型，不冻结Nano操作系统或其他应用。

## 校验与复测

```bash
python3 - <<'PY'
from pathlib import Path
import hashlib,json
root=Path('/home/liwei/robot_releases/nano-video-20261103')
m=json.loads((root/'BASELINE_MANIFEST.json').read_text())
assert all(hashlib.sha256((root/n).read_bytes()).hexdigest()==h for n,h in m['files'].items())
print('PASS',len(m['files']),'files')
PY
systemctl show robot-web robot-vision --property=ActiveState,UnitFileState,WorkingDirectory,DropInPaths
```

PC实测浏览器入口实际交付帧率：

```powershell
python tools/measure_video_delivery.py --seconds 20 --out runs/nano_baseline_video.json
```

此前稳定窗口原图28.84FPS、识别28.10FPS，不能将此数值视作任何负载下的30FPS保证。软件修复细节见 `VIDEO_FIX_20261103.md`。

## 恢复与回退

Nano上完整私有备份为 `/home/liwei/robot_releases/nano-video-20261103-private.zip`，包括实际模型和本机配置。公开Git源码包不包含本机配置及模型权重，不能单独用它恢复整台设备。PC本机完整备份放在本仓库忽略版本控制的 `runs/nano_baseline_backup/nano-video-20261103-private.zip`。恢复时在空目录解压完整私有包，按清单校验，再恢复两项服务覆盖配置中的固定路径；保留运行数据目录并确认Nano原有Python/OpenCV/TensorRT依赖。

如需回退原部署，先将两项服务目录下的 `90-frozen-baseline.conf` 移出 `.conf` 后缀，再执行 `sudo systemctl daemon-reload` 和 `sudo systemctl restart robot-vision robot-web`。原unit仍保留原部署路径。回退属于显式版本切换，不作为默认启动方式。
