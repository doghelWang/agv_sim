# 发给接手 agent 的内容

```text
克隆 https://github.com/doghelWang/agv_sim (main)，读 docs/handoff/HANDOFF_NAV_FOLLOWUP.md，先看最上面的「环境更新」：现在没有树莓派，手机在 10.70.136.239 (Termux SSH 端口 8022)，另有一台不能联网的 RK3588 (192.168.1.64)。
先确认能 SSH 登录手机，然后按第 3 节的优先级在手机上排查导航遗留问题 (第 1 项: Nav2 从 P0 出发即中止)；再评估把执行进程 (ROS 2 / Nav2) 离线部署到 RK3588 的方案，需要我配合开通 SSH 或拷文件时告诉我。
遵守第 1 节的约束，每个问题按第 5 节的完成标准交付。
```
