# 分支与保护规则

## 分支模型

```
main        受保护。只接受 PR 合并；禁止直接 push、禁止 force push、禁止删除
 └── dev    日常开发与实车联调，允许直接 push
      ├── feat/<topic>    新功能
      ├── fix/<topic>     缺陷修复
      └── chore/<topic>   构建、脚本、文档、结构整理
```

## 提交信息

```
feat(scope): 做了什么
fix(scope): 修了什么
chore(scope): 结构/构建/脚本
docs(scope): 文档
refactor(scope): 重构，行为不变
```

`scope` 取 `main` / `gpio_proj` / `driver` / `libraries` / `build` / `docs`。

## 涉及安全逻辑的改动

改到以下任一处，PR 描述必须写清验证方式（台架 / 赛道 / 单元测试）：

- `gpio_proj/src/watchdog.cpp`、`main.cpp`（心跳与超时停车）
- `main/main.cpp` 的 `emergency_stop()`、各失效判定分支
- 电机 PWM / 方向引脚映射（`PROJECT_STRUCTURE.md` 中的引脚表）
- `libraries/drv/src/lq_reg_*.cpp` 中的寄存器操作

## 本地防护

已通过 `core.hooksPath` 启用 `.githooks/pre-push`：直接推 `main` 会被拒绝。

```bash
git config core.hooksPath .githooks          # 新克隆后执行一次
ALLOW_MAIN_PUSH=1 git push origin main       # 确需紧急推送时的逃生口
```

## 远端保护（GitHub 网页设置，一次性）

仓库 → **Settings → Branches → Add branch protection rule**

1. Branch name pattern：`main`
2. 勾选：
   - ✅ Require a pull request before merging
   - ✅ Require approvals（单人仓库可设为 0，仅保留 PR 流程）
   - ✅ Dismiss stale pull request approvals when new commits are pushed
   - ✅ Do not allow bypassing the above settings
3. 勾选 **Rules applied to everyone including administrators**（这一项才真正防误操作）
4. Save

命令行等价写法（需先 `gh auth login`）：

```bash
gh api -X PUT repos/remukisama/remuki-loongson/branches/main/protection \
  -f required_pull_request_reviews.required_approving_review_count=0 \
  -F enforce_admins=true \
  -F required_status_checks=null \
  -F restrictions=null
```

## 仓库重命名后要做的事

GitHub 改名会自动重定向旧地址，但本地建议同步：

```bash
git remote set-url origin https://github.com/remukisama/remuki-loongson.git
git remote -v
```
