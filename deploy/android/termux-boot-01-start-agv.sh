#!/data/data/com.termux/files/usr/bin/bash
# Termux:Boot 开机自启 (install.sh 复制到 ~/.termux/boot/01-start-agv.sh；需安装 Termux:Boot 应用并打开一次)
LOG=~/agv_boot.log
echo "[$(date '+%F %T')] 开机自启" >> "$LOG"
termux-wake-lock 2>/dev/null || true
sshd 2>/dev/null || true
sleep 5
bash ~/start_agv.sh >> "$LOG" 2>&1 &
