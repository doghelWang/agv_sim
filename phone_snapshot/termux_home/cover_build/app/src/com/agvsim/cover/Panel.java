package com.agvsim.cover;

import android.content.Context;
import android.graphics.Color;
import android.os.Handler;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebView;
import android.webkit.WebViewClient;

/** 显示面板页面的 WebView (全屏页面与悬浮窗共用)。平台没起来时每 5 秒重试。 */
final class Panel {
    static final String DEFAULT_URL = "http://127.0.0.1:8082/cover.html";

    static WebView create(Context ctx, final String url, final Handler h) {
        final WebView web = new WebView(ctx);
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
