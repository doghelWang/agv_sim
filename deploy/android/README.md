# Android 部署脚本

完整说明见 [docs/DEPLOY_ANDROID.md](../../docs/DEPLOY_ANDROID.md)。

| 文件 | 运行位置 | 说明 |
|---|---|---|
| `install.sh` | Termux | 一键安装：proot Ubuntu 22.04 → ROS 2 Humble/Nav2/MuJoCo → 代码 → 脚本 → 配置 → 启动 |
| `proot_setup.sh` | proot 容器内 | 安装运行环境 (install.sh 调用) |
| `agv_common.sh` | Termux | 公共配置 (读 `~/.agv.env`) |
| `start_agv.sh` / `stop_agv.sh` / `status_agv.sh` | Termux | 启动 / 停止 / 状态 |
| `update_from_git.sh` | Termux | 从 git 更新代码 |
| `termux-boot-01-start-agv.sh` | Termux:Boot | 开机自启 |
| `proot_spawner.py` | Termux 原生 Python | proot 派生服务 :8069 |
| `fastdds_localhost.xml` / `cyclonedds_localhost.xml` | 执行进程 | Android 回环 DDS 配置 |
| `autostart.py` | Termux 原生 Python | `AGV_AUTOSTART=1` 时开机自动拉起实例 |
| `cover_app/` | Termux 编译，电脑 adb 安装 | 外屏状态面板 APK：显示本机平台的 `/cover.html` (折叠屏外屏；`bash cover_app/build.sh`) |

GPU 射线求交服务 `~/gpucastd` (源码 `sim_core/native/gpucast/`) 由 `update_from_git.sh` 用 Termux 的 `cc` 编译，`start_agv.sh` 启动；
说明与实测见 [docs/PERFORMANCE.md](../../docs/PERFORMANCE.md) 6.8 节。`AGV_GPU_CAST=0` 关闭。

### 外屏状态面板

页面本身可以直接用浏览器打开：`http://<平台>/cover.html` (`?inst=i01` 指定实例，`?period=2` 刷新周期)，内容：节点/实例/实时率/温度、
激光俯视图、规划路径 (含过渡圆弧) 与目标点。

手机上用面板应用显示 (电脑上执行一次)：

```bash
adb install -r agv-cover.apk
adb shell appops set com.agvsim.cover SYSTEM_ALERT_WINDOW allow     # 悬浮窗权限
adb shell appops set com.agvsim.cover TURN_SCREEN_ON allow          # 屏幕灭着时由面板点亮 (开机自启)
adb shell am start --display 1 -n com.agvsim.cover/.MainActivity    # Z Flip 外屏是 display 1；之后 start_agv.sh 会自动打开
```

- **悬浮显示**：面板以悬浮窗盖在 Termux 上面，Termux 仍是前台应用。三星系统会把不在前台的 Termux 及其全部进程放进 `moderate` cpuset
  (只有 3 个小核，绑定服务、前台服务、电池不受限都不管用)；在小核上启动实例时 Nav2 的 TF 监听容易卡死 (表现为 Nav2 任务不动或冲过终点)。
  没有悬浮窗权限时退回普通全屏页面，此时 Termux 被限核。
- **屏幕常亮** (低亮度 0.2)：息屏后系统进入 Doze，同样限核，局域网也连不上。`--ez keep_on false` 关闭常亮，`--ef brightness 0.5` 调亮度。
- **让出屏幕 (听音乐、用别的应用)**：长按面板 → 面板收起、回到外屏桌面，右下角留一个"↩ 面板"小按钮；这时可以打开音乐等应用。
  点小按钮立刻回到面板，不点的话 3 分钟后自动回来 (`--ei yield_s 600` 改时长)；音乐应用在后台继续播放。长按小按钮才是彻底关闭悬浮窗。
  注意让出期间 Termux 不在前台、只有 3 个小核，实时率会掉，正在跑的 Nav2 任务可能停下等定位 —— 选好歌就回来。
- **前台看守** `~/keep_front.sh` (`start_agv.sh` 启动，`stop_agv.sh` 结束，`AGV_KEEP_FRONT=0` 关闭)：每 10 秒检查一次，Termux 连续 30 秒不在前台
  (按了 HOME、别的应用弹到前面) 就让面板把自己和 Termux 调回前台；"让出屏幕"期间不打扰。记录在 `~/agv_front.log`。
- 悬浮窗被关掉后，在 Termux 里 `am broadcast -n com.agvsim.cover/.StartReceiver` 重新打开 (三星不允许从外屏上的应用打开别的应用页面，所以 Termux 里不能用 `am start`)。`AGV_COVER_PANEL=0` 时 `start_agv.sh` 不自动打开。
- `bash ~/status_agv.sh` 显示当前可用的 CPU 核，`0-7` 才是正常状态。
- 手机开着 VPN 时，局域网里的其它设备可能连不上手机的 8022 / 8082 端口；用 `adb forward tcp:8022 tcp:8022` 走 USB，或关掉 VPN。
