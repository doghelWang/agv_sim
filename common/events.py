#!/usr/bin/env python3
"""事件总线 (Web 网关 / 执行进程共用)"""
import threading
import time


class EventHub:
    """
    Thread-safe Industrial Event Hub & Subscription Bus
    Supports categorized channels: chassis, navigation, safety, sensors, system
    """
    def __init__(self, max_history=1200):
        self.max_history = max_history
        self.events = []
        self.event_counter = 0
        self.lock = threading.Lock()
        self.category_counts = {
            "chassis": 0,
            "navigation": 0,
            "safety": 0,
            "sensors": 0,
            "system": 0
        }

    def emit(self, category: str, event_type: str, level: str, title: str, message: str, payload: dict = None):
        with self.lock:
            self.event_counter += 1
            now = time.time()
            time_str = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now % 1) * 1000):03d}"
            if category in self.category_counts:
                self.category_counts[category] += 1
            ev = {
                "id": self.event_counter,
                "timestamp": now,
                "time_str": time_str,
                "category": category,
                "type": event_type,
                "level": level,  # info, success, warning, danger
                "title": title,
                "message": message,
                "payload": payload or {}
            }
            self.events.append(ev)
            if len(self.events) > self.max_history:
                self.events.pop(0)
            return ev

    def get_events(self, since_id=0, categories=None, limit=100):
        with self.lock:
            if categories:
                cats = set(c.strip() for c in categories if c.strip())
            else:
                cats = None

            matched = []
            for e in self.events:
                if e["id"] > since_id:
                    if cats is None or e["category"] in cats:
                        matched.append(e)

            total_matched = len(matched)
            if limit and limit > 0 and total_matched > limit:
                result = matched[-limit:]
            else:
                result = matched

            return {
                "events": result,
                "latest_id": self.event_counter,
                "total_retained": len(self.events),
                "category_counts": dict(self.category_counts)
            }

    def clear(self):
        with self.lock:
            self.events.clear()
            return {"status": "cleared", "latest_id": self.event_counter}


