package com.agvsim.cover;

import android.app.Activity;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebView;
import android.webkit.WebViewClient;

/** 全屏 WebView 显示本机平台的状态面板。默认 http://127.0.0.1:8082/cover.html，
 *  可用 am start -n com.agvsim.cover/.MainActivity --es url http://... 指定。平台没起来时每 5 秒重试；不强制常亮，随系统息屏。 */
public class MainActivity extends Activity {
    private WebView web;
    private String url = "http://127.0.0.1:8082/cover.html";
    private final Handler h = new Handler();

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        String u = getIntent().getStringExtra("url");
        if (u != null && u.startsWith("http")) url = u;
        web = new WebView(this);
        web.setBackgroundColor(Color.BLACK);
        web.getSettings().setJavaScriptEnabled(true);
        web.setWebViewClient(new WebViewClient() {
            @Override
            public void onReceivedError(WebView v, WebResourceRequest rq, WebResourceError e) {
                if (rq.isForMainFrame()) {
                    v.loadData("<body style='background:#000;color:#888;font:5vmin sans-serif;padding:8vmin'>平台未启动，等待中…</body>", "text/html; charset=utf-8", "utf-8");
                    h.postDelayed(new Runnable() { public void run() { web.loadUrl(url); } }, 5000);
                }
            }
        });
        setContentView(web);
        immersive();
        web.loadUrl(url);
    }

    private void immersive() {
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
    }

    @Override public void onWindowFocusChanged(boolean f) { super.onWindowFocusChanged(f); if (f) immersive(); }
    @Override protected void onPause() { super.onPause(); web.onPause(); web.pauseTimers(); }       // 息屏时停掉页面定时器，不空转
    @Override protected void onResume() { super.onResume(); web.resumeTimers(); web.onResume(); }
    @Override protected void onDestroy() { h.removeCallbacksAndMessages(null); web.destroy(); super.onDestroy(); }
}
