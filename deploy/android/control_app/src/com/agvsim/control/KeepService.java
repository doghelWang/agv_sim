package com.agvsim.control;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Intent;
import android.net.wifi.WifiManager;
import android.os.IBinder;
import android.os.PowerManager;

/** foreground service: keeps Wi-Fi in high-performance mode and the CPU awake so the platform stays reachable on the LAN */
public class KeepService extends Service {
    private WifiManager.WifiLock wifi;
    private PowerManager.WakeLock cpu;

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        NotificationManager nm = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);
        nm.createNotificationChannel(new NotificationChannel("keep", "保持在线", NotificationManager.IMPORTANCE_LOW));
        PendingIntent pi = PendingIntent.getActivity(this, 0, new Intent(this, MainActivity.class), PendingIntent.FLAG_IMMUTABLE);
        Notification n = new Notification.Builder(this, "keep").setContentTitle("AMR 仿真平台保持在线")
                .setContentText("Wi-Fi 与 CPU 保持唤醒, 局域网可随时访问").setSmallIcon(android.R.drawable.stat_sys_upload_done)
                .setContentIntent(pi).setOngoing(true).build();
        startForeground(1, n);
        if (wifi == null) {
            WifiManager wm = (WifiManager) getApplicationContext().getSystemService(WIFI_SERVICE);
            wifi = wm.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "agv:wifi");
            wifi.setReferenceCounted(false); wifi.acquire();
        }
        if (cpu == null) {
            PowerManager pm = (PowerManager) getSystemService(POWER_SERVICE);
            cpu = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "agv:cpu");
            cpu.setReferenceCounted(false); cpu.acquire();
        }
        return START_STICKY;
    }

    @Override
    public void onDestroy() {
        if (wifi != null && wifi.isHeld()) wifi.release();
        if (cpu != null && cpu.isHeld()) cpu.release();
        super.onDestroy();
    }

    @Override public IBinder onBind(Intent i) { return null; }
}
