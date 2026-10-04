#!/data/data/com.termux/files/usr/bin/bash
# Termux:Boot 开机自启 (install.sh / update_from_git.sh 复制到 ~/.termux/boot/01-start-agv.sh；需安装 Termux:Boot 应用并打开一次)
# 手机设了锁屏密码时，重启后要先解锁一次系统才会触发。start_agv.sh 会先打开外屏面板 (点亮屏幕、把 Termux 调到前台) 再起平台与实例。
LOG=~/agv_boot.log
echo "[$(date '+%F %T')] 开机自启" >> "$LOG"
termux-wake-lock 2>/dev/null || true
sshd 2>/dev/null || true
sleep 8
bash ~/start_agv.sh >> "$LOG" 2>&1 &
