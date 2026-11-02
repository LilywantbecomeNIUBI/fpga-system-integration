# Nano默认运行基线

用户已指定后续默认使用 `nano-video-20261103`。参见 `../docs/NANO_BASELINE.md` 及 `../releases/nano-video-20261103/`。

Nano两项开机服务已指向 `/home/liwei/robot_releases/nano-video-20261103`，代码、配置与模型只读。旧目录 `robot_system_web_20261102` 已不代表实际运行源。

后续修改可在工作区开发。发布新版时使用新的独立目录和版本号，不覆盖冻结包或解除基线目录只读属性；仅用户要求切换版本时更新默认服务。核对运行版本必须检查API版本号、逐文件SHA256与systemd实际路径，不能只看目录名。图传验收须测量公共入口实际完整MJPEG帧及唯一帧键，不以接收FPS替代显示交付帧率。
