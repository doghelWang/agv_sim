package com.agvsim.cover;

import android.app.Activity;
import android.app.ActivityOptions;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.provider.Settings;
import android.view.Display;
import android.view.View;
import android.view.WindowManager;
import android.webkit.WebView;

/** 外屏状态面板入口。默认 http://127.0.0.1:8082/cover.html；可用
 *    am start -n com.agvsim.cover/.MainActivity [--es url http://...] [--ez keep_on false] [--ef brightness 0.5] [--ez overlay false] [--ei display 1]
 *  有悬浮窗权限: 启动 OverlayService 把面板盖在 Termux 上，再把 Termux 切到前台，自己退出 (Termux 保持前台 → 全部 CPU 核可用)。
 *  没有权限 / overlay=false: 自己全屏显示 (此时 Termux 在后台，三星系统会把它限制到小核)。
 *  屏幕默认常亮 (低亮度): 息屏后系统进入 Doze，Termux 被限核、局域网也连不上。 */
public class MainActivity extends Activity {
    private WebView web;
    private final Handler h = new Handler();

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        Intent in = getIntent();
        Panel.setExited(this, false);            // 点应用图标 / am start = 重新开启
        String u = in.getStringExtra("url");
        String url = (u != null && u.startsWith("http")) ? u : Panel.DEFAULT_URL;
        boolean keepOn = in.getBooleanExtra("keep_on", true);
        float bright = in.getFloatExtra("brightness", 0.2f);
        if (in.getBooleanExtra("overlay", true) && Settings.canDrawOverlays(this)) {
            int disp = in.getIntExtra("display", -1);
            startService(new Intent(this, OverlayService.class).putExtra("url", url).putExtra("keep_on", keepOn)
                    .putExtra("brightness", bright).putExtra("display", disp));
            Intent t = getPackageManager().getLaunchIntentForPackage("com.termux");
            if (t != null) {
                try { if (disp >= 0) startActivity(t, ActivityOptions.makeBasic().setLaunchDisplayId(disp).toBundle()); else startActivity(t); }
                catch (Exception e) { /* 没装 Termux: 悬浮窗照常显示 */ }
            }
            finish();
            return;
        }
        stopService(new Intent(this, OverlayService.class));
        web = Panel.create(this, url, h, new Runnable() { public void run() { Panel.setExited(MainActivity.this, true); finish(); } });
        setContentView(web);
        if (keepOn) {
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            WindowManager.LayoutParams lp = getWindow().getAttributes();
            lp.screenBrightness = bright;
            getWindow().setAttributes(lp);
        }
        immersive();
    }

    private void immersive() {
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
    }

    @Override public void onWindowFocusChanged(boolean f) { super.onWindowFocusChanged(f); if (f && web != null) immersive(); }
    @Override protected void onPause() { super.onPause(); if (web != null) { web.onPause(); web.pauseTimers(); } }
    @Override protected void onResume() { super.onResume(); if (web != null) { web.resumeTimers(); web.onResume(); } }
    @Override protected void onDestroy() { h.removeCallbacksAndMessages(null); if (web != null) web.destroy(); super.onDestroy(); }
}
