package com.agvsim.control;

import android.app.Activity;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.webkit.JavascriptInterface;
import android.webkit.WebChromeClient;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;

public class MainActivity extends Activity {
    private WebView web;
    private final Handler ui = new Handler(Looper.getMainLooper());

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        web = new WebView(this);
        WebSettings s = web.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);
        s.setAllowFileAccess(true);
        s.setAllowFileAccessFromFileURLs(true);
        s.setAllowUniversalAccessFromFileURLs(true);
        s.setMixedContentMode(WebSettings.MIXED_CONTENT_ALWAYS_ALLOW);
        s.setCacheMode(WebSettings.LOAD_NO_CACHE);
        web.setWebViewClient(new WebViewClient());
        web.setWebChromeClient(new WebChromeClient());
        web.addJavascriptInterface(new Bridge(this), "Native");
        setContentView(web);
        web.loadUrl("file:///android_asset/index.html");
        if (prefs().getBoolean("keep", true)) startKeep(true);
    }

    @Override
    public void onBackPressed() {
        web.evaluateJavascript("window.onBack && window.onBack()", new android.webkit.ValueCallback<String>() {
            @Override public void onReceiveValue(String v) { if (!"true".equals(v)) moveTaskToBack(true); }
        });
    }

    private SharedPreferences prefs() { return getSharedPreferences("agv", MODE_PRIVATE); }

    private void startKeep(boolean on) {
        Intent i = new Intent(this, KeepService.class);
        if (on) { if (Build.VERSION.SDK_INT >= 26) startForegroundService(i); else startService(i); }
        else stopService(i);
    }

    private void js(final String code) { ui.post(new Runnable() { @Override public void run() { web.evaluateJavascript(code, null); } }); }

    private static String q(String s) {
        StringBuilder b = new StringBuilder("\"");
        for (char c : s.toCharArray()) {
            switch (c) {
                case '"': b.append("\\\""); break;
                case '\\': b.append("\\\\"); break;
                case '\n': b.append("\\n"); break;
                case '\r': break;
                case ' ': case ' ': b.append(' '); break;
                default: if (c < 0x20) b.append(' '); else b.append(c);
            }
        }
        return b.append('"').toString();
    }

    private void runRoot(String id, String cmd, int timeoutS) {
        int rc = -1; StringBuilder out = new StringBuilder();
        try {
            final Process p = new ProcessBuilder("su").redirectErrorStream(true).start();
            OutputStream o = p.getOutputStream();
            o.write((cmd + "\nexit $?\n").getBytes("UTF-8")); o.flush(); o.close();
            final long ms = Math.max(5, timeoutS) * 1000L;
            Thread killer = new Thread(new Runnable() { @Override public void run() { try { Thread.sleep(ms); p.destroy(); } catch (InterruptedException ignored) { } } });
            killer.start();
            BufferedReader r = new BufferedReader(new InputStreamReader(p.getInputStream(), "UTF-8"));
            String line; while ((line = r.readLine()) != null) { if (out.length() < 20000) out.append(line).append('\n'); }
            rc = p.waitFor(); killer.interrupt();
        } catch (Exception e) { out.append("ERROR: ").append(e.toString()); }
        js("window.__rootDone && window.__rootDone(" + q(id) + "," + rc + "," + q(out.toString()) + ")");
    }

    // static (not inner): javac 21 writes a MethodParameters attribute for inner-class constructors that old d8 cannot read
    static class Bridge {
        private final MainActivity a;
        Bridge(MainActivity a) { this.a = a; }
        /** run a shell command as root (Magisk su); result arrives in window.__rootDone(id, rc, output) */
        @JavascriptInterface
        public void root(final String id, final String cmd, final int timeoutS) {
            new Thread(new Runnable() { @Override public void run() { a.runRoot(id, cmd, timeoutS); } }).start();
        }
        @JavascriptInterface public void keepAwake(final boolean on) { a.prefs().edit().putBoolean("keep", on).apply(); a.ui.post(new Runnable() { @Override public void run() { a.startKeep(on); } }); }
        @JavascriptInterface public boolean keepOn() { return a.prefs().getBoolean("keep", true); }
        @JavascriptInterface public String version() { return "1.0"; }
    }
}
