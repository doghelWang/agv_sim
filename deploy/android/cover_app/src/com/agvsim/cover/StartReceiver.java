package com.agvsim.cover;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

/** 从 Termux 里打开悬浮面板的入口: am broadcast -n com.agvsim.cover/.StartReceiver [--es url …] [--ez keep_on false] [--ei display 1]
 *  用户在面板上点过"退出"后不再响应 (启动脚本、前台看守发来的都算)，除非带 --ez force true 或用户点应用图标重新打开。
 *  (三星不允许从外屏上的应用打开别的应用页面；后台应用也不能被别的应用直接 startService，所以经广播转一次) */
public class StartReceiver extends BroadcastReceiver {
    @Override
    public void onReceive(Context c, Intent in) {
        if (Panel.exited(c)) {
            if (!in.getBooleanExtra("force", false)) return;
            Panel.setExited(c, false);
        }
        Intent s = new Intent(c, OverlayService.class);
        if (in.getExtras() != null) s.putExtras(in.getExtras());
        c.startForegroundService(s);
    }
}
