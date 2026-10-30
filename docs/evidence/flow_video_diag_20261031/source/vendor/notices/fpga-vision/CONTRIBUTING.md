# 团队 Git 使用规则

1. 从本仓库开发。主分支`main`保存已验证的可复现版本；功能/修复从main创建`codex/<功能>`或`feat/<功能>`、`fix/<问题>`分支。
2. 修改前`git pull --ff-only`，不要用复制旧实验目录覆盖新仓库；音频和图像仓库各自拥有共享网络模块副本，跨仓修改需分别验证并记录。
3. 提交源码、IP配置/仿真模型、CST/SDC、工程、脚本、说明和小型验证摘要。不要提交`build/`、`tests/`生成结果、波形、位流、录音、模型、密钥或运行token。
4. 提交前执行`python tools/check_sources.py`、`python tools/project.py test vision`和对应Gowin构建。厂商原语模型来自本机安装；CI只能覆盖无需厂商安装的检查。
5. 使用明确路径`git add`，检查`git diff --cached --stat`、`git diff --cached --check`后提交。不要将未验收硬件功能写成已通过。保存输入条件、编译/仿真退出码、资源/时序及位流SHA256。
6. 推送功能分支并提交PR，经团队审查后合入main。为可交付版本打带日期标签，位流通过GitHub Release发布，明确匹配的提交和构建参数。

```powershell
git switch main
git pull --ff-only
git switch -c feat/your-change
# 修改、检查、构建和仿真
git add src constraints projects scripts docs
git diff --cached --check
git commit -m "feat: describe the actual change"
git push -u origin HEAD
```

`tools/project.py check`用于核对最初冻结交付的文件哈希，不是要求开发后继续匹配旧位流；源码变更时旧位流下载检查会拒绝混用。新稳定版本需更新基线清单、重新构建和验收后再发布。不要用旧Release位流验证新的未匹配源码。

本机两个仓库的credential helper只引用现有`GITHUB_PAT_TOKEN`环境变量，不把口令写进Git配置。其他队友应自行`gh auth login`或使用自己的Git凭据；不得共享个人token。
