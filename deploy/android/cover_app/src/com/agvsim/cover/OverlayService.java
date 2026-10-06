package com.agvsim.cover;

import android.app.ActivityOptions;
import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.graphics.PixelFormat;
import android.graphics.drawable.GradientDrawable;
import android.hardware.display.DisplayManager;
import android.os.Build;
import android.os.Handler;
import android.os.IBinder;
import android.os.PowerManager;
import android.os.SystemClock;
import android.view.Display;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.webkit.WebView;
import android.widget.TextView;

/** 悬浮窗面板: 全屏盖在指定屏幕上，不抢焦点 (下面的 Termux 仍是前台应用)。
 *  打开: 应用图标 / adb am start，或在 Termux 里 am broadcast -n com.agvsim.cover/.StartReceiver
 *  (可带 --es url … --ez keep_on false --ef brightness 0.5 --ei display 1 --ei yield_s 180)
 *  长按面板 = 让出屏幕: 面板收成右下角一个小按钮并回到桌面，可以用别的应用；点小按钮才回来 (Termux 回到前台，面板盖上)。
 *  默认不会自动回来 (启动脚本、前台看守的打开请求在让出期间都不理)；要定时自动回来，打开时带 --ei yield_s 秒数。
 *  让出期间 Termux 只有小核可用，屏幕照常休眠。长按小按钮 = 退出面板 (同页面上的「退出」)。
 *  面板页面上的按钮可以点 (下发任务 / 退出)。"退出" = 关掉面板并记住: 之后启动脚本和前台看守都不再把它打开，
 *  直到用户点应用图标 (或 am broadcast … --ez force true) 重新开启。退出后 Termux 不再被保持在前台，会被系统限制到小核。 */
public class OverlayService extends Service {
    private WindowManager wm;
    private WebView web;
    private View chip;
    private Intent lastReq = new Intent();
    private Display lastDisplay;
    private long yieldUntil;             // 让出屏幕期间 (elapsedRealtime, ms；不自动回来时是 Long.MAX_VALUE) 忽略外部的打开请求 (前台看守每 30 秒会发一次)
    private final Handler h = new Handler();

    @Override public IBinder onBind(Intent i) { return null; }

    @Override
    public int onStartCommand(Intent in, int flags, int id) {
        if (in == null) in = new Intent();
        if (Panel.exited(this)) {                // 已退出 (系统自己重启服务时会走到这里): 不显示
            if (!in.getBooleanExtra("force", false)) { stopSelf(); return START_NOT_STICKY; }
            Panel.setExited(this, false);
        }
        NotificationManager nm = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);
        nm.createNotificationChannel(new NotificationChannel("panel", "状态面板", NotificationManager.IMPORTANCE_MIN));
        startForeground(1, new Notification.Builder(this, "panel").setContentTitle("AMR 仿真面板")
                .setContentText("悬浮显示中；长按面板临时让出屏幕").setSmallIcon(android.R.drawable.ic_menu_view).build());
        boolean force = in.getBooleanExtra("force", false);
        if (SystemClock.elapsedRealtime() < yieldUntil && !force) return START_STICKY;
        if (Panel.prefs(this).getBoolean("yielded", false) && !force) {
            // 让出期间进程被系统回收后重新拉起: 只把小按钮放回去，不盖面板
            yieldUntil = Long.MAX_VALUE;
            lastReq = in;
            wm = (WindowManager) getSystemService(WINDOW_SERVICE);
            showChip(this);
            return START_STICKY;
        }
        setYielded(false);
        yieldUntil = 0;
        remove();
        h.removeCallbacksAndMessages(null);
        lastReq = in;
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
        web = Panel.create(c, (u != null && u.startsWith("http")) ? u : Panel.DEFAULT_URL, h, new Runnable() { public void run() { exitPanel(); } });
        int f = WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE | WindowManager.LayoutParams.FLAG_LAYOUT_IN_SCREEN
                | WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS | WindowManager.LayoutParams.FLAG_HARDWARE_ACCELERATED;
        boolean keepOn = in.getBooleanExtra("keep_on", true);
        if (keepOn) f |= WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON;
        WindowManager.LayoutParams lp = new WindowManager.LayoutParams(WindowManager.LayoutParams.MATCH_PARENT,
                WindowManager.LayoutParams.MATCH_PARENT, WindowManager.LayoutParams.TYPE_APPLICATION_OVERLAY, f, PixelFormat.OPAQUE);
        if (d != null) {                         // 按屏幕实际像素算 (MATCH_PARENT 在外屏上尺寸不稳定)，再让开屏幕缺口那一条:
            android.graphics.Point sz = new android.graphics.Point();   // Z Flip 外屏右下角是摄像头缺口 (下面 66 px)，同一条的左半边是系统导航键，
            d.getRealSize(sz);                                           // 盖上去的话面板的字被挡、系统按键也被面板垫底。
            int[] in4 = cutoutInsets(d);
            lp.x = in4[0]; lp.y = in4[1];
            lp.width = sz.x - in4[0] - in4[2]; lp.height = sz.y - in4[1] - in4[3];
            lp.gravity = android.view.Gravity.TOP | android.view.Gravity.LEFT;
        }
        if (keepOn) lp.screenBrightness = in.getFloatExtra("brightness", 0.2f);
        if (Build.VERSION.SDK_INT >= 28) lp.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES;
        final GestureDetector gd = new GestureDetector(c, new GestureDetector.SimpleOnGestureListener() {
            @Override public void onLongPress(MotionEvent e) { yieldScreen(); }
        });
        lastDisplay = d;
        web.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View v, MotionEvent e) { gd.onTouchEvent(e); return false; }   // 不吃掉: 页面上的按钮要能点
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

    /** 屏幕缺口占掉的四边 {左, 上, 右, 下} (像素)。Display.getCutout 是 API 29，Termux 自带的 android.jar 较旧，用反射 */
    private static int[] cutoutInsets(Display d) {
        int[] r = new int[4];
        try {
            Object c = Display.class.getMethod("getCutout").invoke(d);
            if (c != null) {
                String[] m = { "getSafeInsetLeft", "getSafeInsetTop", "getSafeInsetRight", "getSafeInsetBottom" };
                for (int i = 0; i < 4; i++) r[i] = Math.max(0, (Integer) c.getClass().getMethod(m[i]).invoke(c));
            }
        } catch (Exception e) { /* 取不到就铺满 */ }
        return r;
    }

    /** 临时让出屏幕: 面板换成一个小按钮，回到桌面；到时间或点小按钮后恢复 */
    private void yieldScreen() {
        final Context c = web != null ? web.getContext() : this;
        int sec = lastReq.getIntExtra("yield_s", 0);          // 0 = 不自动回来 (默认)
        if (web != null) { try { wm.removeView(web); } catch (Exception e) {} web.destroy(); web = null; }
        yieldUntil = sec > 0 ? SystemClock.elapsedRealtime() + Math.max(10, sec) * 1000L : Long.MAX_VALUE;
        if (sec <= 0) setYielded(true);
        showChip(c);
        Intent home = new Intent(Intent.ACTION_MAIN).addCategory(Intent.CATEGORY_HOME).addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        try {
            if (lastDisplay != null) startActivity(home, ActivityOptions.makeBasic().setLaunchDisplayId(lastDisplay.getDisplayId()).toBundle());
            else startActivity(home);
        } catch (Exception e) { /* 回不了桌面也没关系: 面板已经收起，用户自己切 */ }
        h.removeCallbacksAndMessages(null);
        if (sec > 0) h.postDelayed(new Runnable() { public void run() { restore(); } }, Math.max(10, sec) * 1000L);
    }

    private void setYielded(boolean v) { Panel.prefs(this).edit().putBoolean("yielded", v).commit(); }

    /** 右下角的小按钮: 点 = 面板回来，长按 = 关闭面板 */
    private void showChip(Context c) {
        if (chip != null) return;
        TextView t = new TextView(c);
        t.setText("\u21A9 面板");
        t.setTextColor(0xFFE8EEF5); t.setTextSize(15); t.setPadding(28, 16, 28, 16);
        GradientDrawable bg = new GradientDrawable();
        bg.setColor(0xCC0E151D); bg.setCornerRadius(40); bg.setStroke(2, 0xFF4DA3FF);
        t.setBackground(bg);
        WindowManager.LayoutParams lp = new WindowManager.LayoutParams(WindowManager.LayoutParams.WRAP_CONTENT,
                WindowManager.LayoutParams.WRAP_CONTENT, WindowManager.LayoutParams.TYPE_APPLICATION_OVERLAY,
                WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE | (yieldUntil == Long.MAX_VALUE ? 0 : WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON),
                PixelFormat.TRANSLUCENT);          // 不自动回来时屏幕照常休眠
        lp.gravity = android.view.Gravity.BOTTOM | android.view.Gravity.RIGHT; lp.x = 16; lp.y = 90;
        t.setOnClickListener(new View.OnClickListener() { public void onClick(View v) { restore(); } });
        t.setOnLongClickListener(new View.OnLongClickListener() { public boolean onLongClick(View v) { exitPanel(); return true; } });
        try { wm.addView(t, lp); chip = t; } catch (Exception e) { chip = null; }
    }

    /** 页面上点了"退出": 关掉面板，记住已退出 */
    private void exitPanel() {
        Panel.setExited(this, true);
        setYielded(false);
        h.removeCallbacksAndMessages(null);
        remove();
        stopForeground(true);
        stopSelf();
    }

    private void restore() {
        setYielded(false);
        yieldUntil = 0;
        h.removeCallbacksAndMessages(null);
        remove();
        show(lastReq);
    }

    private void remove() {
        if (web != null) { try { wm.removeView(web); } catch (Exception e) {} web.destroy(); web = null; }
        if (chip != null) { try { wm.removeView(chip); } catch (Exception e) {} chip = null; }
    }

    @Override public void onDestroy() { h.removeCallbacksAndMessages(null); remove(); super.onDestroy(); }
}
