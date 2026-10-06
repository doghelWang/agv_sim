package com.agvsim.cover;

import android.content.Context;
import android.graphics.Color;
import android.content.SharedPreferences;
import android.os.Handler;
import android.webkit.JavascriptInterface;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebView;
import android.webkit.WebViewClient;

/** 显示面板页面的 WebView (全屏页面与悬浮窗共用)。平台没起来时每 5 秒重试。
 *  页面里可以调用 AgvCover.exit() 退出面板 (onExit 在主线程执行)。 */
final class Panel {
    static final String DEFAULT_URL = "http://127.0.0.1:8082/cover.html";

    /** 用户点了"退出"后记下来: 之后脚本发来的打开请求 (启动脚本、前台看守) 一律不理，直到用户点应用图标重新打开 */
    static SharedPreferences prefs(Context c) { return c.getApplicationContext().getSharedPreferences("panel", Context.MODE_PRIVATE); }
    static boolean exited(Context c) { return prefs(c).getBoolean("exited", false); }
    static void setExited(Context c, boolean v) { prefs(c).edit().putBoolean("exited", v).commit(); }

    static WebView create(Context ctx, final String url, final Handler h, final Runnable onExit) {
        final WebView web = new WebView(ctx);
        web.setLongClickable(false);
        web.setHapticFeedbackEnabled(false);
        web.addJavascriptInterface(new Object() {
            @JavascriptInterface public void exit() { if (onExit != null) h.post(onExit); }
            @JavascriptInterface public int version() { return 5; }
        }, "AgvCover");
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
        web.loadUrl(url);
        return web;
    }
}
