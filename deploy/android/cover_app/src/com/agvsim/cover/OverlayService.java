package com.agvsim.cover;

import android.app.ActivityOptions;
import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.graphics.PixelFormat;
import android.hardware.display.DisplayManager;
import android.os.Build;
import android.os.Handler;
import android.os.IBinder;
import android.os.PowerManager;
import android.view.Display;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.webkit.WebView;

/** 悬浮窗面板: 全屏盖在指定屏幕上，不抢焦点 (下面的 Termux 仍是前台应用)。长按面板关闭。
 *  打开: 应用图标 / adb am start，或在 Termux 里 am broadcast -n com.agvsim.cover/.StartReceiver
 *  (可带 --es url … --ez keep_on false --ef brightness 0.5 --ei display 1) */
public class OverlayService extends Service {
    private WindowManager wm;
    private WebView web;
    private final Handler h = new Handler();

    @Override public IBinder onBind(Intent i) { return null; }

    @Override
    public int onStartCommand(Intent in, int flags, int id) {
        if (in == null) in = new Intent();
        NotificationManager nm = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);
        nm.createNotificationChannel(new NotificationChannel("panel", "状态面板", NotificationManager.IMPORTANCE_MIN));
        startForeground(1, new Notification.Builder(this, "panel").setContentTitle("AMR 仿真面板")
                .setContentText("悬浮显示中，长按面板关闭").setSmallIcon(android.R.drawable.ic_menu_view).build());
        remove();
        h.removeCallbacksAndMessages(null);
        final Intent req = in;
        final boolean keep = in.getBooleanExtra("keep_on", true);
        DisplayManager dm0 = (DisplayManager) getSystemService(DISPLAY_SERVICE);
        Display m0 = dm0.getDisplay(Display.DEFAULT_DISPLAY);
        boolean anyOn = m0 != null && m0.getState() == Display.STATE_ON;
        for (int i = 1; i <= 3 && !anyOn; i++) { Display x = dm0.getDisplay(i); anyOn = x != null && x.getState() == Display.STATE_ON; }
        if (keep && !anyOn) {
            // 屏幕是灭的 (开机自启、息屏后重新打开): 先点亮，等屏幕状态稳定后再决定显示在哪块屏
            try {
                PowerManager pm = (PowerManager) getSystemService(POWER_SERVICE);
                pm.newWakeLock(PowerManager.SCREEN_BRIGHT_WAKE_LOCK | PowerManager.ACQUIRE_CAUSES_WAKEUP, "agvcover:wake").acquire(5000);
            } catch (Exception e) { /* 没有 WAKE_LOCK 权限时照常显示 */ }
            h.postDelayed(new Runnable() { public void run() { show(req); } }, 1200);
        } else {
            show(in);
        }
        return START_STICKY;
    }

    private void show(Intent in) {
        // 显示在哪块屏: extra display 指定；否则外屏亮着用外屏，不然用主屏
        DisplayManager dm = (DisplayManager) getSystemService(DISPLAY_SERVICE);
        Display d = in.getIntExtra("display", -1) >= 0 ? dm.getDisplay(in.getIntExtra("display", -1)) : null;
        if (d == null) {
            // 折叠屏合上时主屏是灭的，亮的是外屏。三星的外屏不出现在 getDisplays() 里，但能按编号取到 (Z Flip 是 1)
            Display main = dm.getDisplay(Display.DEFAULT_DISPLAY);
            if (main == null || main.getState() != Display.STATE_ON) {
                for (int i = 1; i <= 3 && d == null; i++) {
                    Display x = dm.getDisplay(i);
                    if (x != null && x.getState() == Display.STATE_ON) d = x;
                }
                // 都没亮 (没能点亮屏幕): 有外屏就用外屏 —— 之后屏幕亮起来时面板已经在上面
                for (int i = 1; i <= 3 && d == null; i++) d = dm.getDisplay(i);
            }
        }
        if (d == null) d = dm.getDisplay(Display.DEFAULT_DISPLAY);
        Context c = d != null ? createDisplayContext(d) : this;
        if (Build.VERSION.SDK_INT >= 30) {       // createWindowContext (API 30)；Termux 自带的 android.jar 较旧，用反射调用
            try {
                c = (Context) Context.class.getMethod("createWindowContext", int.class, android.os.Bundle.class)
                        .invoke(c, WindowManager.LayoutParams.TYPE_APPLICATION_OVERLAY, null);
            } catch (Exception e) { /* 用显示屏上下文 */ }
        }
        wm = (WindowManager) c.getSystemService(WINDOW_SERVICE);
        String u = in.getStringExtra("url");
        web = Panel.create(c, (u != null && u.startsWith("http")) ? u : Panel.DEFAULT_URL, h);
        int f = WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE | WindowManager.LayoutParams.FLAG_LAYOUT_IN_SCREEN
                | WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS | WindowManager.LayoutParams.FLAG_HARDWARE_ACCELERATED;
        boolean keepOn = in.getBooleanExtra("keep_on", true);
        if (keepOn) f |= WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON;
        WindowManager.LayoutParams lp = new WindowManager.LayoutParams(WindowManager.LayoutParams.MATCH_PARENT,
                WindowManager.LayoutParams.MATCH_PARENT, WindowManager.LayoutParams.TYPE_APPLICATION_OVERLAY, f, PixelFormat.OPAQUE);
        if (d != null) {                         // 按屏幕实际像素铺满 (MATCH_PARENT 会留出导航栏/输入法区域)
            android.graphics.Point sz = new android.graphics.Point();
            d.getRealSize(sz);
            lp.width = sz.x; lp.height = sz.y; lp.gravity = android.view.Gravity.TOP | android.view.Gravity.LEFT;
        }
        if (keepOn) lp.screenBrightness = in.getFloatExtra("brightness", 0.2f);
        if (Build.VERSION.SDK_INT >= 28) lp.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES;
        final GestureDetector gd = new GestureDetector(c, new GestureDetector.SimpleOnGestureListener() {
            @Override public void onLongPress(MotionEvent e) { stopSelf(); }
        });
        web.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View v, MotionEvent e) { gd.onTouchEvent(e); return true; }
        });
        try { wm.addView(web, lp); } catch (Exception e) { web = null; stopSelf(); return; }
        // 把 Termux 调到同一块屏的前台 (面板不抢焦点，盖在它上面): Termux 是前台应用时仿真/导航进程才能用全部 CPU 核。
        // 本应用有悬浮窗权限，允许从后台打开页面；--ez termux false 不调
        if (in.getBooleanExtra("termux", true)) {
            Intent t = getPackageManager().getLaunchIntentForPackage("com.termux");
            if (t != null) {
                t.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
                try {
                    if (d != null) startActivity(t, ActivityOptions.makeBasic().setLaunchDisplayId(d.getDisplayId()).toBundle());
                    else startActivity(t);
                } catch (Exception e) { /* 没装 Termux 或系统不允许: 面板照常显示 */ }
            }
        }
    }

    private void remove() {
        if (web != null) { try { wm.removeView(web); } catch (Exception e) {} web.destroy(); web = null; }
    }

    @Override public void onDestroy() { h.removeCallbacksAndMessages(null); remove(); super.onDestroy(); }
}
