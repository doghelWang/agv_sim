package com.agvsim.cover;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

/** 从 Termux 里打开悬浮面板的入口: am broadcast -n com.agvsim.cover/.StartReceiver [--es url …] [--ez keep_on false] [--ei display 1]
 *  (三星不允许从外屏上的应用打开别的应用页面；后台应用也不能被别的应用直接 startService，所以经广播转一次) */
public class StartReceiver extends BroadcastReceiver {
    @Override
    public void onReceive(Context c, Intent in) {
        Intent s = new Intent(c, OverlayService.class);
        if (in.getExtras() != null) s.putExtras(in.getExtras());
        c.startForegroundService(s);
    }
}
