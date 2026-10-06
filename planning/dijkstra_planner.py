#!/usr/bin/env python3
"""
Dijkstra Topological Road-Network Planner (Dijkstra 拓扑路网最优路径规划器)
Supports multiple warehouse scenarios (Cross-dock, Narrow Aisle VNA, FMS Workshop)
with dynamic obstacle edge blockage detection, automatic rerouting, and strict orthogonal alignment.
"""

import math
import heapq
from typing import Optional, List, Tuple, Dict, Any


SCENARIO_DEFINITIONS: Dict[str, Dict[str, Any]] = {
    "grid_9_square": {
        "id": "grid_9_square",
        "name": "正方形九宫格智能立体仓",
        "description": "3×3 宽距高吞吐立体仓储区，四向宽幅十字干道 + 外围周转环线，零死锁全向调度",
        "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "shelves": [
            {"name": "1号高架原料库 (NW 仓区)", "x1": -6.0, "y1": 2.6, "x2": -2.6, "y2": 6.0, "color": "#1e293b", "border": "#38bdf8"},
            {"name": "2号精益结构件库 (NE 仓区)", "x1": 2.6, "y1": 2.6, "x2": 6.0, "y2": 6.0, "color": "#1e293b", "border": "#818cf8"},
            {"name": "3号半成品周转库 (SW 仓区)", "x1": -6.0, "y1": -6.0, "x2": -2.6, "y2": -2.6, "color": "#1e293b", "border": "#34d399"},
            {"name": "4号成品集港发运库 (SE 仓区)", "x1": 2.6, "y1": -6.0, "x2": 6.0, "y2": -2.6, "color": "#1e293b", "border": "#f472b6"}
        ],
        "walls": [
            (-9.0, -9.0, 9.0, -9.0), (9.0, -9.0, 9.0, 9.0), (9.0, 9.0, -9.0, 9.0), (-9.0, 9.0, -9.0, -9.0),
            # Bay 1 (NW 仓区)
            (-6.0, 2.6, -2.6, 2.6), (-6.0, 6.0, -2.6, 6.0), (-6.0, 2.6, -6.0, 6.0), (-2.6, 2.6, -2.6, 6.0),
            # Bay 2 (NE 仓区)
            (2.6, 2.6, 6.0, 2.6), (2.6, 6.0, 6.0, 6.0), (2.6, 2.6, 2.6, 6.0), (6.0, 2.6, 6.0, 6.0),
            # Bay 3 (SW 仓区)
            (-6.0, -6.0, -2.6, -6.0), (-6.0, -2.6, -2.6, -2.6), (-6.0, -6.0, -6.0, -2.6), (-2.6, -6.0, -2.6, -2.6),
            # Bay 4 (SE 仓区)
            (2.6, -6.0, 6.0, -6.0), (2.6, -2.6, 6.0, -2.6), (2.6, -6.0, 2.6, -2.6), (6.0, -6.0, 6.0, -2.6)
        ],
        "stations": [
            {"id": "S1", "name": "1号原料入库位 (S1)", "x": 0.0, "y": 5.0, "dock_yaw": 1.5708, "color": "#38bdf8"},
            {"id": "S2", "name": "2号智能拣选位 (S2)", "x": -5.0, "y": 0.0, "dock_yaw": 3.14159, "color": "#34d399"},
            {"id": "S3", "name": "3号成品打包位 (S3)", "x": 0.0, "y": -5.0, "dock_yaw": -1.5708, "color": "#f97316"},
            {"id": "S4", "name": "4号出库发运位 (S4)", "x": 5.0, "y": 0.0, "dock_yaw": 0.0, "color": "#f472b6"},
            {"id": "P0", "name": "中央待命调度中心 (P0)", "x": 0.0, "y": 0.0, "dock_yaw": 0.0, "color": "#6366f1"},
            # 外围环线新增 8 个点位: 4 个角部缓存位 + 4 个环线中段装卸位 (距墙/货架 ≥1.5 m，满足车体外接圆 1.40 m 原地转向)
            {"id": "S5", "name": "5号西北角缓存位 (S5)", "x": -7.5, "y": 7.5, "dock_yaw": 3.14159, "color": "#0ea5e9"},
            {"id": "S6", "name": "6号东北角缓存位 (S6)", "x": 7.5, "y": 7.5, "dock_yaw": 0.0, "color": "#8b5cf6"},
            {"id": "S7", "name": "7号西南角缓存位 (S7)", "x": -7.5, "y": -7.5, "dock_yaw": 3.14159, "color": "#14b8a6"},
            {"id": "S8", "name": "8号东南角充电位 (S8)", "x": 7.5, "y": -7.5, "dock_yaw": 0.0, "color": "#22c55e"},
            {"id": "S9", "name": "9号北侧西装卸位 (S9)", "x": -4.3, "y": 7.5, "dock_yaw": 1.5708, "color": "#f59e0b"},
            {"id": "S10", "name": "10号北侧东装卸位 (S10)", "x": 4.3, "y": 7.5, "dock_yaw": 1.5708, "color": "#ef4444"},
            {"id": "S11", "name": "11号南侧西装卸位 (S11)", "x": -4.3, "y": -7.5, "dock_yaw": -1.5708, "color": "#ec4899"},
            {"id": "S12", "name": "12号南侧东装卸位 (S12)", "x": 4.3, "y": -7.5, "dock_yaw": -1.5708, "color": "#a855f7"}
        ],
        "nodes": {
            "N_CTR": (0.0, 0.0),
            "N_N1": (0.0, 2.5), "N_N2": (0.0, 5.0), "N_N3": (0.0, 7.5),
            "N_S1": (0.0, -2.5), "N_S2": (0.0, -5.0), "N_S3": (0.0, -7.5),
            "N_E1": (2.5, 0.0), "N_E2": (5.0, 0.0), "N_E3": (7.5, 0.0),
            "N_W1": (-2.5, 0.0), "N_W2": (-5.0, 0.0), "N_W3": (-7.5, 0.0),
            "N_NW": (-7.5, 7.5), "N_NE": (7.5, 7.5),
            "N_SW": (-7.5, -7.5), "N_SE": (7.5, -7.5),
            "N_N3W": (-4.3, 7.5), "N_N3E": (4.3, 7.5),
            "N_S3W": (-4.3, -7.5), "N_S3E": (4.3, -7.5)
        },
        "connections": [
            ("N_CTR", "N_N1"), ("N_N1", "N_N2"), ("N_N2", "N_N3"),
            ("N_CTR", "N_S1"), ("N_S1", "N_S2"), ("N_S2", "N_S3"),
            ("N_CTR", "N_E1"), ("N_E1", "N_E2"), ("N_E2", "N_E3"),
            ("N_CTR", "N_W1"), ("N_W1", "N_W2"), ("N_W2", "N_W3"),
            ("N_NW", "N_N3W"), ("N_N3W", "N_N3"), ("N_N3", "N_N3E"), ("N_N3E", "N_NE"),
            ("N_NE", "N_E3"), ("N_E3", "N_SE"),
            ("N_SE", "N_S3E"), ("N_S3E", "N_S3"), ("N_S3", "N_S3W"), ("N_S3W", "N_SW"),
            ("N_SW", "N_W3"), ("N_W3", "N_NW")
        ]
    },

    "standard_cross": {
        "id": "standard_cross",
        "name": "标准十字仓储物流中心",
        "description": "四大立体货架区 (原材料/半成品/成品/辅料)，4.4m 宽幅十字通道 + 3.0m 外围周转环线",
        "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "shelves": [
            {"name": "货架区 A (原材料库)", "x1": 2.2, "y1": 2.2, "x2": 5.6, "y2": 5.0, "color": "#1f2937", "border": "#3b82f6"},
            {"name": "货架区 B (半成品库)", "x1": -5.6, "y1": 2.2, "x2": -2.2, "y2": 5.0, "color": "#1f2937", "border": "#8b5cf6"},
            {"name": "货架区 C (成品立库)", "x1": 2.2, "y1": -5.0, "x2": 5.6, "y2": -2.2, "color": "#1f2937", "border": "#10b981"},
            {"name": "货架区 D (辅料周转)", "x1": -5.6, "y1": -5.0, "x2": -2.2, "y2": -2.2, "color": "#1f2937", "border": "#f59e0b"}
        ],
        "walls": [
            (-8.5, -8.5, 8.5, -8.5), (8.5, -8.5, 8.5, 8.5), (8.5, 8.5, -8.5, 8.5), (-8.5, 8.5, -8.5, -8.5),
            (2.2, 2.2, 5.6, 2.2), (2.2, 5.0, 5.6, 5.0), (2.2, 2.2, 2.2, 5.0), (5.6, 2.2, 5.6, 5.0),
            (-5.6, 2.2, -2.2, 2.2), (-5.6, 5.0, -2.2, 5.0), (-5.6, 2.2, -5.6, 5.0), (-2.2, 2.2, -2.2, 5.0),
            (2.2, -5.0, 5.6, -5.0), (2.2, -2.2, 5.6, -2.2), (2.2, -5.0, 2.2, -2.2), (5.6, -5.0, 5.6, -2.2),
            (-5.6, -5.0, -2.2, -5.0), (-5.6, -2.2, -2.2, -2.2), (-5.6, -5.0, -5.6, -2.2), (-2.2, -5.0, -2.2, -2.2)
        ],
        "stations": [
            {"id": "st_a", "name": "工位 A (上料)", "x": 5.0, "y": 0.0, "dock_yaw": 0.0, "color": "#58a6ff"},
            {"id": "st_b", "name": "工位 B (出库)", "x": -5.0, "y": 0.0, "dock_yaw": 3.14159, "color": "#bc8cff"},
            {"id": "st_charge", "name": "自动充电桩", "x": 0.0, "y": -6.5, "dock_yaw": -1.5708, "color": "#3fb950"},
            {"id": "st_c", "name": "工位 C (质检)", "x": 0.0, "y": 6.5, "dock_yaw": 1.5708, "color": "#e3b341"},
            {"id": "st_idle", "name": "待命中心", "x": 0.0, "y": 0.0, "dock_yaw": 0.0, "color": "#f0883e"}
        ],
        "nodes": {
            "N_ORIGIN": (0.0, 0.0),
            "N_STATION_A": (5.0, 0.0),
            "N_STATION_B": (-5.0, 0.0),
            "N_CHARGING": (0.0, -6.5),
            "N_STATION_C": (0.0, 6.5),
            "N_CROSS_N": (0.0, 6.5),
            "N_CROSS_S": (0.0, -6.5),
            "N_CROSS_E": (7.0, 0.0),
            "N_CROSS_W": (-7.0, 0.0),
            "N_CORNER_NE": (7.0, 6.5),
            "N_CORNER_NW": (-7.0, 6.5),
            "N_CORNER_SE": (7.0, -6.5),
            "N_CORNER_SW": (-7.0, -6.5)
        },
        "connections": [
            ("N_ORIGIN", "N_STATION_C"),
            ("N_ORIGIN", "N_CHARGING"),
            ("N_ORIGIN", "N_STATION_A"), ("N_STATION_A", "N_CROSS_E"),
            ("N_ORIGIN", "N_STATION_B"), ("N_STATION_B", "N_CROSS_W"),
            ("N_STATION_C", "N_CORNER_NE"), ("N_CORNER_NE", "N_CROSS_E"),
            ("N_CROSS_E", "N_CORNER_SE"), ("N_CORNER_SE", "N_CHARGING"),
            ("N_CHARGING", "N_CORNER_SW"), ("N_CORNER_SW", "N_CROSS_W"),
            ("N_CROSS_W", "N_CORNER_NW"), ("N_CORNER_NW", "N_STATION_C")
        ]
    },

    "narrow_aisle": {
        "id": "narrow_aisle",
        "name": "高密窄巷道立体库",
        "description": "多排高架垂直货架阵列，3.0m 畅行巷道与南北双向 5.0m 宽幅高速干道",
        "origin": {"x": 0.0, "y": -6.0, "yaw": 0.0},
        "shelves": [
            {"name": "1号高架排架", "x1": -6.8, "y1": -3.5, "x2": -5.4, "y2": 3.5, "color": "#1f2937", "border": "#38bdf8"},
            {"name": "2号高架排架", "x1": -2.4, "y1": -3.5, "x2": -1.0, "y2": 3.5, "color": "#1f2937", "border": "#818cf8"},
            {"name": "3号高架排架", "x1": 1.0, "y1": -3.5, "x2": 2.4, "y2": 3.5, "color": "#1f2937", "border": "#34d399"},
            {"name": "4号高架排架", "x1": 5.4, "y1": -3.5, "x2": 6.8, "y2": 3.5, "color": "#1f2937", "border": "#fbbf24"}
        ],
        "walls": [
            (-9.0, -8.5, 9.0, -8.5), (9.0, -8.5, 9.0, 8.5), (9.0, 8.5, -9.0, 8.5), (-9.0, 8.5, -9.0, -8.5),
            (-6.8, -3.5, -5.4, -3.5), (-6.8, 3.5, -5.4, 3.5), (-6.8, -3.5, -6.8, 3.5), (-5.4, -3.5, -5.4, 3.5),
            (-2.4, -3.5, -1.0, -3.5), (-2.4, 3.5, -1.0, 3.5), (-2.4, -3.5, -2.4, 3.5), (-1.0, -3.5, -1.0, 3.5),
            (1.0, -3.5, 2.4, -3.5), (1.0, 3.5, 2.4, 3.5), (1.0, -3.5, 1.0, 3.5), (2.4, -3.5, 2.4, 3.5),
            (5.4, -3.5, 6.8, -3.5), (5.4, 3.5, 6.8, 3.5), (5.4, -3.5, 5.4, 3.5), (6.8, -3.5, 6.8, 3.5)
        ],
        "stations": [
            {"id": "vna_st1", "name": "1号巷道进料口", "x": -3.9, "y": 5.5, "dock_yaw": 1.5708, "color": "#58a6ff"},
            {"id": "vna_st2", "name": "2号巷道存储位", "x": 0.0, "y": 0.0, "dock_yaw": 1.5708, "color": "#bc8cff"},
            {"id": "vna_st3", "name": "3号巷道拣货位", "x": 3.9, "y": 5.5, "dock_yaw": 1.5708, "color": "#e3b341"},
            {"id": "vna_charge", "name": "高架专用充电机", "x": -3.9, "y": -6.0, "dock_yaw": -1.5708, "color": "#3fb950"},
            {"id": "vna_buffer", "name": "出库主缓存站", "x": 3.9, "y": -6.0, "dock_yaw": 0.0, "color": "#ec4899"},
            {"id": "vna_idle", "name": "窄巷道待命位", "x": 0.0, "y": -6.0, "dock_yaw": 0.0, "color": "#f0883e"}
        ],
        "nodes": {
            "N_VNA_1_TOP": (-3.9, 6.0),
            "N_VNA_1_STATION": (-3.9, 4.5),
            "N_VNA_1_MID": (-3.9, 0.0),
            "N_VNA_1_BOT": (-3.9, -6.0),
            "N_VNA_2_TOP": (0.0, 6.0),
            "N_VNA_2_STATION": (0.0, 0.0),
            "N_VNA_2_BOT": (0.0, -6.0),
            "N_VNA_3_TOP": (3.9, 6.0),
            "N_VNA_3_STATION": (3.9, 4.5),
            "N_VNA_3_MID": (3.9, 0.0),
            "N_VNA_3_BOT": (3.9, -6.0),
            "N_VNA_W_TOP": (-7.9, 6.0),
            "N_VNA_W_BOT": (-7.9, -6.0),
            "N_VNA_E_TOP": (7.9, 6.0),
            "N_VNA_E_BOT": (7.9, -6.0)
        },
        "connections": [
            ("N_VNA_W_TOP", "N_VNA_1_TOP"), ("N_VNA_1_TOP", "N_VNA_2_TOP"), ("N_VNA_2_TOP", "N_VNA_3_TOP"), ("N_VNA_3_TOP", "N_VNA_E_TOP"),
            ("N_VNA_W_BOT", "N_VNA_1_BOT"), ("N_VNA_1_BOT", "N_VNA_2_BOT"), ("N_VNA_2_BOT", "N_VNA_3_BOT"), ("N_VNA_3_BOT", "N_VNA_E_BOT"),
            ("N_VNA_W_TOP", "N_VNA_W_BOT"), ("N_VNA_E_TOP", "N_VNA_E_BOT"),
            ("N_VNA_1_TOP", "N_VNA_1_STATION"), ("N_VNA_1_STATION", "N_VNA_1_MID"), ("N_VNA_1_MID", "N_VNA_1_BOT"),
            ("N_VNA_2_TOP", "N_VNA_2_STATION"), ("N_VNA_2_STATION", "N_VNA_2_BOT"),
            ("N_VNA_3_TOP", "N_VNA_3_STATION"), ("N_VNA_3_STATION", "N_VNA_3_MID"), ("N_VNA_3_MID", "N_VNA_3_BOT")
        ]
    },

    "rect_loop": {
        "id": "rect_loop",
        "name": "长方形环线制造车间",
        "description": "长方形柔性生产大环线，包含中心双岛制造单元与中央 3.6m 南北直通旁路",
        "origin": {"x": -3.4, "y": -4.5, "yaw": 0.0},
        "shelves": [
            {"name": "精密 CNC 机械加工岛", "x1": -5.0, "y1": -2.2, "x2": -1.8, "y2": 2.2, "color": "#1e293b", "border": "#06b6d4"},
            {"name": "工业机器人柔性装配岛", "x1": 1.8, "y1": -2.2, "x2": 5.0, "y2": 2.2, "color": "#1e293b", "border": "#10b981"}
        ],
        "walls": [
            (-8.5, -6.5, 8.5, -6.5), (8.5, -6.5, 8.5, 6.5), (8.5, 6.5, -8.5, 6.5), (-8.5, 6.5, -8.5, -6.5),
            (-5.0, -2.2, -1.8, -2.2), (-5.0, 2.2, -1.8, 2.2), (-5.0, -2.2, -5.0, 2.2), (-1.8, -2.2, -1.8, 2.2),
            (1.8, -2.2, 5.0, -2.2), (1.8, 2.2, 5.0, 2.2), (1.8, -2.2, 1.8, 2.2), (5.0, -2.2, 5.0, 2.2)
        ],
        "stations": [
            {"id": "rect_st_load", "name": "1号原料上线工位", "x": -3.4, "y": 4.5, "dock_yaw": 0.0, "color": "#38bdf8"},
            {"id": "rect_st_asm", "name": "2号柔性装配工位", "x": 3.4, "y": 4.5, "dock_yaw": 0.0, "color": "#c084fc"},
            {"id": "rect_st_aoi", "name": "3号AOI智能终检位", "x": 6.8, "y": 0.0, "dock_yaw": -1.5708, "color": "#fbbf24"},
            {"id": "rect_st_pack", "name": "4号成品下线包装", "x": 3.4, "y": -4.5, "dock_yaw": 3.14159, "color": "#10b981"},
            {"id": "rect_st_charge", "name": "5号环线专用快充桩", "x": -3.4, "y": -4.5, "dock_yaw": -1.5708, "color": "#22c55e"},
            {"id": "rect_st_buffer", "name": "6号物料缓存等待位", "x": -6.8, "y": 0.0, "dock_yaw": 1.5708, "color": "#f97316"},
            {"id": "rect_st_center", "name": "环线中央调度中枢", "x": 0.0, "y": 0.0, "dock_yaw": 0.0, "color": "#06b6d4"}
        ],
        "nodes": {
            "N_L_NW": (-6.8, 4.5), "N_L_N_W": (-3.4, 4.5), "N_L_N_MID": (0.0, 4.5), "N_L_N_E": (3.4, 4.5), "N_L_NE": (6.8, 4.5),
            "N_L_W_MID": (-6.8, 0.0), "N_L_CENTER": (0.0, 0.0), "N_L_E_MID": (6.8, 0.0),
            "N_L_SW": (-6.8, -4.5), "N_L_S_W": (-3.4, -4.5), "N_L_S_MID": (0.0, -4.5), "N_L_S_E": (3.4, -4.5), "N_L_SE": (6.8, -4.5)
        },
        "connections": [
            ("N_L_NW", "N_L_N_W"), ("N_L_N_W", "N_L_N_MID"), ("N_L_N_MID", "N_L_N_E"), ("N_L_N_E", "N_L_NE"),
            ("N_L_NE", "N_L_E_MID"), ("N_L_E_MID", "N_L_SE"),
            ("N_L_SE", "N_L_S_E"), ("N_L_S_E", "N_L_S_MID"), ("N_L_S_MID", "N_L_S_W"), ("N_L_S_W", "N_L_SW"),
            ("N_L_SW", "N_L_W_MID"), ("N_L_W_MID", "N_L_NW"),
            ("N_L_N_MID", "N_L_CENTER"), ("N_L_CENTER", "N_L_S_MID")
        ]
    },

    "fms_workshop": {
        "id": "fms_workshop",
        "name": "自动化柔性制造车间",
        "description": "岛式生产布局（CNC机加工/SMT贴片/机器人装配/智能测试/中央缓存塔），全通畅 2.0m+ 环线",
        "origin": {"x": 0.0, "y": 4.5, "yaw": 0.0},
        "shelves": [
            {"name": "CNC 机械加工岛", "x1": 3.2, "y1": 3.0, "x2": 6.8, "y2": 6.5, "color": "#1f2937", "border": "#06b6d4"},
            {"name": "SMT 贴片流水线", "x1": -6.8, "y1": 3.0, "x2": -3.2, "y2": 6.5, "color": "#1f2937", "border": "#ec4899"},
            {"name": "机器人装配岛", "x1": 3.2, "y1": -6.5, "x2": 6.8, "y2": -3.0, "color": "#1f2937", "border": "#10b981"},
            {"name": "智能质检测试岛", "x1": -6.8, "y1": -6.5, "x2": -3.2, "y2": -3.0, "color": "#1f2937", "border": "#f59e0b"},
            {"name": "中央立体缓存塔", "x1": -1.2, "y1": -1.2, "x2": 1.2, "y2": 1.2, "color": "#1f2937", "border": "#8b5cf6"}
        ],
        "walls": [
            (-9.0, -9.0, 9.0, -9.0), (9.0, -9.0, 9.0, 9.0), (9.0, 9.0, -9.0, 9.0), (-9.0, 9.0, -9.0, -9.0),
            (3.2, 3.0, 6.8, 3.0), (3.2, 6.5, 6.8, 6.5), (3.2, 3.0, 3.2, 6.5), (6.8, 3.0, 6.8, 6.5),
            (-6.8, 3.0, -3.2, 3.0), (-6.8, 6.5, -3.2, 6.5), (-6.8, 3.0, -6.8, 6.5), (-3.2, 3.0, -3.2, 6.5),
            (3.2, -6.5, 6.8, -6.5), (3.2, -3.0, 6.8, -3.0), (3.2, -6.5, 3.2, -3.0), (6.8, -6.5, 6.8, -3.0),
            (-6.8, -6.5, -3.2, -6.5), (-6.8, -3.0, -3.2, -3.0), (-6.8, -6.5, -6.8, -3.0), (-3.2, -6.5, -3.2, -3.0),
            (-1.2, -1.2, 1.2, -1.2), (-1.2, 1.2, 1.2, 1.2), (-1.2, -1.2, -1.2, 1.2), (1.2, -1.2, 1.2, 1.2)
        ],
        "stations": [
            {"id": "fms_smt", "name": "SMT 供料工位", "x": -5.0, "y": 0.0, "dock_yaw": 1.5708, "color": "#ec4899"},
            {"id": "fms_cnc", "name": "CNC 进料工位", "x": 5.0, "y": 0.0, "dock_yaw": 1.5708, "color": "#06b6d4"},
            {"id": "fms_robot", "name": "机器人装配位", "x": 5.0, "y": -1.5, "dock_yaw": -1.5708, "color": "#10b981"},
            {"id": "fms_qc", "name": "质检包装工位", "x": -5.0, "y": -1.5, "dock_yaw": -1.5708, "color": "#f59e0b"},
            {"id": "fms_charge", "name": "自动化快充岛", "x": 0.0, "y": -7.5, "dock_yaw": -1.5708, "color": "#3fb950"},
            {"id": "fms_idle", "name": "车间待命总站", "x": 0.0, "y": 4.5, "dock_yaw": 0.0, "color": "#8b5cf6"}
        ],
        "nodes": {
            "N_FMS_N_MID": (0.0, 7.5),
            "N_FMS_S_MID": (0.0, -7.5),
            "N_FMS_NE": (7.8, 7.5),
            "N_FMS_NW": (-7.8, 7.5),
            "N_FMS_SE": (7.8, -7.5),
            "N_FMS_SW": (-7.8, -7.5),
            "N_FMS_CROSS_W": (-7.8, 0.0),
            "N_FMS_STATION_SMT": (-5.0, 0.0),
            "N_FMS_LOOP_W": (-2.2, 0.0),
            "N_FMS_CROSS_E": (7.8, 0.0),
            "N_FMS_STATION_CNC": (5.0, 0.0),
            "N_FMS_LOOP_E": (2.2, 0.0),
            "N_FMS_LOOP_NW": (-2.2, 2.1),
            "N_FMS_LOOP_NE": (2.2, 2.1),
            "N_FMS_LOOP_SW": (-2.2, -2.1),
            "N_FMS_LOOP_SE": (2.2, -2.1),
            "N_FMS_STATION_IDLE": (0.0, 4.5)
        },
        "connections": [
            ("N_FMS_NW", "N_FMS_N_MID"), ("N_FMS_N_MID", "N_FMS_NE"),
            ("N_FMS_NE", "N_FMS_CROSS_E"), ("N_FMS_CROSS_E", "N_FMS_SE"),
            ("N_FMS_SE", "N_FMS_S_MID"), ("N_FMS_S_MID", "N_FMS_SW"),
            ("N_FMS_SW", "N_FMS_CROSS_W"), ("N_FMS_CROSS_W", "N_FMS_NW"),
            ("N_FMS_CROSS_W", "N_FMS_STATION_SMT"), ("N_FMS_STATION_SMT", "N_FMS_LOOP_W"),
            ("N_FMS_CROSS_E", "N_FMS_STATION_CNC"), ("N_FMS_STATION_CNC", "N_FMS_LOOP_E"),
            ("N_FMS_LOOP_W", "N_FMS_LOOP_NW"), ("N_FMS_LOOP_NW", "N_FMS_LOOP_NE"), ("N_FMS_LOOP_NE", "N_FMS_LOOP_E"),
            ("N_FMS_LOOP_W", "N_FMS_LOOP_SW"), ("N_FMS_LOOP_SW", "N_FMS_LOOP_SE"), ("N_FMS_LOOP_SE", "N_FMS_LOOP_E"),
            ("N_FMS_N_MID", "N_FMS_STATION_IDLE"), ("N_FMS_STATION_IDLE", "N_FMS_LOOP_NE"),
            ("N_FMS_S_MID", "N_FMS_LOOP_SE")
        ]
    },
    # 大车版柔性制造车间: 与 fms_workshop 同样的五个工艺岛与六个工位，按长车头单舵轮 (车长 ≈1.8 m、
    # 原地转向扫掠半径 ≈1.45 m) 重新布局。所有路口/工位距设备与墙 ≥ 1.7 m，可在拓扑拐点原地转向；
    # 工位位于设备岛前 1.7 m 的通道上，车头朝向设备停靠后距设备约 0.4 m。原 fms_workshop 保留给小车型。
    "fms_workshop_xl": {
        "id": "fms_workshop_xl",
        "name": "柔性制造车间 · 大车版",
        "description": "fms_workshop 的大车型布局: 24 m×24 m，3.4 m 宽网格通道，路口/工位均可原地转向 (车长 ≤ 2.0 m)",
        "origin": {"x": 0.0, "y": 10.2, "yaw": 0.0},
        "shelves": [
            {"name": "CNC 机械加工岛", "x1": 4.6, "y1": 4.6, "x2": 8.2, "y2": 8.2, "color": "#1f2937", "border": "#06b6d4"},
            {"name": "SMT 贴片流水线", "x1": -8.2, "y1": 4.6, "x2": -4.6, "y2": 8.2, "color": "#1f2937", "border": "#ec4899"},
            {"name": "机器人装配岛", "x1": 4.6, "y1": -8.2, "x2": 8.2, "y2": -4.6, "color": "#1f2937", "border": "#10b981"},
            {"name": "智能质检测试岛", "x1": -8.2, "y1": -8.2, "x2": -4.6, "y2": -4.6, "color": "#1f2937", "border": "#f59e0b"},
            {"name": "中央立体缓存塔", "x1": -1.2, "y1": -1.2, "x2": 1.2, "y2": 1.2, "color": "#1f2937", "border": "#8b5cf6"}
        ],
        "walls": [
            (-12.0, -12.0, 12.0, -12.0), (12.0, -12.0, 12.0, 12.0), (12.0, 12.0, -12.0, 12.0), (-12.0, 12.0, -12.0, -12.0),
            (4.6, 4.6, 8.2, 4.6), (8.2, 4.6, 8.2, 8.2), (8.2, 8.2, 4.6, 8.2), (4.6, 8.2, 4.6, 4.6), (-8.2, 4.6, -4.6, 4.6), (-4.6, 4.6, -4.6, 8.2), (-4.6, 8.2, -8.2, 8.2), (-8.2, 8.2, -8.2, 4.6), (4.6, -8.2, 8.2, -8.2), (8.2, -8.2, 8.2, -4.6), (8.2, -4.6, 4.6, -4.6), (4.6, -4.6, 4.6, -8.2), (-8.2, -8.2, -4.6, -8.2), (-4.6, -8.2, -4.6, -4.6), (-4.6, -4.6, -8.2, -4.6), (-8.2, -4.6, -8.2, -8.2), (-1.2, -1.2, 1.2, -1.2), (1.2, -1.2, 1.2, 1.2), (1.2, 1.2, -1.2, 1.2), (-1.2, 1.2, -1.2, -1.2)
        ],
        "stations": [
            {"id": "fms_smt", "name": "SMT 供料工位", "x": -6.4, "y": 2.9, "dock_yaw": 1.5708, "color": "#ec4899"},
            {"id": "fms_cnc", "name": "CNC 进料工位", "x": 6.4, "y": 2.9, "dock_yaw": 1.5708, "color": "#06b6d4"},
            {"id": "fms_robot", "name": "机器人装配位", "x": 6.4, "y": -2.9, "dock_yaw": -1.5708, "color": "#10b981"},
            {"id": "fms_qc", "name": "质检包装工位", "x": -6.4, "y": -2.9, "dock_yaw": -1.5708, "color": "#f59e0b"},
            {"id": "fms_charge", "name": "自动化快充岛", "x": 0.0, "y": -10.2, "dock_yaw": -1.5708, "color": "#3fb950"},
            {"id": "fms_idle", "name": "车间待命总站", "x": 0.0, "y": 10.2, "dock_yaw": 0.0, "color": "#8b5cf6"}
        ],
        "nodes": {
            "XL_NW": (-10.2, 10.2), "XL_N_W": (-2.9, 10.2), "XL_IDLE": (0.0, 10.2), "XL_N_E": (2.9, 10.2), "XL_NE": (10.2, 10.2),
            "XL_W_N": (-10.2, 2.9), "XL_SMT": (-6.4, 2.9), "XL_C_NW": (-2.9, 2.9), "XL_C_NE": (2.9, 2.9), "XL_CNC": (6.4, 2.9), "XL_E_N": (10.2, 2.9),
            "XL_W_S": (-10.2, -2.9), "XL_QC": (-6.4, -2.9), "XL_C_SW": (-2.9, -2.9), "XL_C_SE": (2.9, -2.9), "XL_ROBOT": (6.4, -2.9), "XL_E_S": (10.2, -2.9),
            "XL_SW": (-10.2, -10.2), "XL_S_W": (-2.9, -10.2), "XL_CHARGE": (0.0, -10.2), "XL_S_E": (2.9, -10.2), "XL_SE": (10.2, -10.2)
        },
        "connections": [
            ("XL_NW", "XL_N_W"), ("XL_N_W", "XL_IDLE"), ("XL_IDLE", "XL_N_E"), ("XL_N_E", "XL_NE"),
            ("XL_SW", "XL_S_W"), ("XL_S_W", "XL_CHARGE"), ("XL_CHARGE", "XL_S_E"), ("XL_S_E", "XL_SE"),
            ("XL_NW", "XL_W_N"), ("XL_W_N", "XL_W_S"), ("XL_W_S", "XL_SW"),
            ("XL_NE", "XL_E_N"), ("XL_E_N", "XL_E_S"), ("XL_E_S", "XL_SE"),
            ("XL_W_N", "XL_SMT"), ("XL_SMT", "XL_C_NW"), ("XL_C_NW", "XL_C_NE"), ("XL_C_NE", "XL_CNC"), ("XL_CNC", "XL_E_N"),
            ("XL_W_S", "XL_QC"), ("XL_QC", "XL_C_SW"), ("XL_C_SW", "XL_C_SE"), ("XL_C_SE", "XL_ROBOT"), ("XL_ROBOT", "XL_E_S"),
            ("XL_N_W", "XL_C_NW"), ("XL_C_NW", "XL_C_SW"), ("XL_C_SW", "XL_S_W"),
            ("XL_N_E", "XL_C_NE"), ("XL_C_NE", "XL_C_SE"), ("XL_C_SE", "XL_S_E")
        ]
    }
}


def register_scenario(sc: Dict[str, Any]) -> str:
    """登记外部 (REST) 下发的场景定义；执行进程据此规划，无需本地场景库"""
    d = dict(sc)
    d["nodes"] = {k: tuple(v) for k, v in (sc.get("nodes") or {}).items()}
    d["connections"] = [tuple(c) for c in sc.get("connections") or []]
    d["walls"] = [tuple(w) for w in sc.get("walls") or []]
    SCENARIO_DEFINITIONS[d["id"]] = d
    return d["id"]


class DijkstraPlanner:
    def __init__(self, default_scenario: str = "grid_9_square", robot_half_width: float = 0.35, robot_circum_radius: float = 0.6):
        # 车体尺寸 (来自 cmodel): 通行边检查用半宽，节点 (转向/停靠处) 检查用外接圆半径
        self.robot_half_width = robot_half_width
        self.robot_circum_radius = robot_circum_radius
        self.active_scenario_id = default_scenario
        self.nodes = {}
        self.edges = {}
        self.set_scenario(default_scenario)

    def set_scenario(self, scenario_id: str):
        if scenario_id not in SCENARIO_DEFINITIONS:
            scenario_id = "grid_9_square"
        self.active_scenario_id = scenario_id
        self._corner_cache = {}
        sc = SCENARIO_DEFINITIONS[scenario_id]

        self.nodes = dict(sc["nodes"])
        self.edges = {u: [] for u in self.nodes}
        self._router = None                 # C 实现 (planning/native)，首次规划时按当前场景建立

        for u, v in sc["connections"]:
            if u in self.nodes and v in self.nodes:
                p_u = self.nodes[u]
                p_v = self.nodes[v]
                dist = math.hypot(p_v[0] - p_u[0], p_v[1] - p_u[1])
                self.edges[u].append((v, dist))
                self.edges[v].append((u, dist))

    def get_scenario_metadata(self) -> Dict[str, Any]:
        sc = SCENARIO_DEFINITIONS[self.active_scenario_id]
        walls = sc.get("walls", [])
        min_x = min([w[0] for w in walls[:4]] + [w[2] for w in walls[:4]]) if len(walls) >= 4 else -7.5
        max_x = max([w[0] for w in walls[:4]] + [w[2] for w in walls[:4]]) if len(walls) >= 4 else 7.5
        min_y = min([w[1] for w in walls[:4]] + [w[3] for w in walls[:4]]) if len(walls) >= 4 else -7.5
        max_y = max([w[1] for w in walls[:4]] + [w[3] for w in walls[:4]]) if len(walls) >= 4 else 7.5
        return {
            "id": sc["id"],
            "name": sc["name"],
            "description": sc["description"],
            "walls": sc.get("walls", []),
            "shelves": sc.get("shelves", []),
            "stations": sc.get("stations", []),
            "origin": sc["origin"],
            "bounds": {
                "min_x": min_x,
                "max_x": max_x,
                "min_y": min_y,
                "max_y": max_y
            },
            "all_scenarios": [
                {"id": k, "name": v["name"], "description": v["description"]}
                for k, v in SCENARIO_DEFINITIONS.items()
            ]
        }

    def get_walls(self) -> List[Tuple[float, float, float, float]]:
        return list(SCENARIO_DEFINITIONS[self.active_scenario_id]["walls"])

    def get_stations(self) -> List[Dict[str, Any]]:
        return list(SCENARIO_DEFINITIONS[self.active_scenario_id]["stations"])

    def get_topology(self) -> Dict[str, Any]:
        """Export topological nodes and routes for Web rendering."""
        edge_list = []
        seen = set()
        for u, neighbors in self.edges.items():
            for v, dist in neighbors:
                edge_key = tuple(sorted([u, v]))
                if edge_key not in seen:
                    seen.add(edge_key)
                    edge_list.append({
                        "from": u,
                        "to": v,
                        "p1": {"x": self.nodes[u][0], "y": self.nodes[u][1]},
                        "p2": {"x": self.nodes[v][0], "y": self.nodes[v][1]},
                        "distance": round(dist, 2)
                    })

        return {
            "scenario_id": self.active_scenario_id,
            "nodes": {k: {"x": v[0], "y": v[1], "name": k} for k, v in self.nodes.items()},
            "edges": edge_list
        }

    def _find_nearest_node(self, pt: Tuple[float, float], obstacles: Any = None) -> str:
        """最近的路网节点；若有障碍物，跳过"接入段"被障碍物阻断的节点 (避免先朝障碍物开过去)"""
        best_node = None
        min_d = float('inf')
        for node_id, pos in self.nodes.items():
            if obstacles and (self._is_node_blocked(pos, obstacles) or
                              (math.hypot(pos[0] - pt[0], pos[1] - pt[1]) > 0.3 and self._is_edge_blocked(pt, pos, obstacles))):
                continue
            if not self.edges.get(node_id):
                continue  # 孤立节点 (未连入路网) 不参与吸附
            d = math.hypot(pos[0] - pt[0], pos[1] - pt[1])
            if d < min_d:
                min_d = d
                best_node = node_id
        return best_node

    def _obs_circle(self, obs):
        if isinstance(obs, dict):
            return float(obs.get("x", 0.0)), float(obs.get("y", 0.0)), math.hypot(float(obs.get("w", 0.8)), float(obs.get("h", 0.8))) / 2.0
        ox1, oy1, ox2, oy2 = obs
        return (ox1 + ox2) / 2.0, (oy1 + oy2) / 2.0, 0.3

    def _is_node_blocked(self, p, obstacles) -> bool:
        """节点处车辆可能原地转向，按外接圆半径检查"""
        for obs in obstacles:
            cx, cy, r = self._obs_circle(obs)
            if math.hypot(cx - p[0], cy - p[1]) < r + self.robot_circum_radius:
                return True
        return False

    def _is_edge_blocked(self, p1: Tuple[float, float], p2: Tuple[float, float], obstacles: Any) -> bool:
        return self._edge_blocked_r(p1, p2, obstacles, self.robot_half_width + 0.15)

    @staticmethod
    def _edge_blocked_r(p1: Tuple[float, float], p2: Tuple[float, float], obstacles: Any, clearance: float) -> bool:
        """Check if topological edge between p1 and p2 is blocked by any dynamic obstacle."""
        for obs in obstacles:
            if isinstance(obs, dict):
                cx = float(obs.get("x", 0.0))
                cy = float(obs.get("y", 0.0))
                w = float(obs.get("w", 0.8))
                h = float(obs.get("h", 0.8))
                radius = math.hypot(w, h) / 2.0 + clearance
            else:
                ox1, oy1, ox2, oy2 = obs
                cx = (ox1 + ox2) / 2.0
                cy = (oy1 + oy2) / 2.0
                radius = 0.3 + clearance

            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            l2 = dx*dx + dy*dy
            if l2 == 0:
                d = math.hypot(cx - p1[0], cy - p1[1])
            else:
                t = max(0.0, min(1.0, ((cx - p1[0]) * dx + (cy - p1[1]) * dy) / l2))
                proj_x = p1[0] + t * dx
                proj_y = p1[1] + t * dy
                d = math.hypot(cx - proj_x, cy - proj_y)

            if d < radius:
                return True
        return False

    # ------------------------------------------------------------------ 拓扑贴合规划
    # 起点/终点不再"直线连到最近节点"(会斜穿货架/设备岛)，而是垂直投影到最近的可达拓扑边上，
    # 经投影点并入路网；路网内按 "路程 + 转向代价" 做 Dijkstra (状态=节点+来向)，路径全程沿拓扑边。
    TURN_COST_PER_RAD = 0.8     # 每弧度转向折算的路程 (m)：同等路程下优先少拐弯的路线
    CORNER_STOP_COST = 1.5      # 每个拐点的停车+原地转向折算路程 (m)
    CORNER_BLOCK_COST = 40.0    # 车体在拐点无论圆弧/原地转向都会扫到设备 → 该转移几乎禁止 (无其它路线时仍可用)
    footprint = None            # (head, tail, half_width, 过弯半径) —— 由执行进程按车型设置
    OFF_NET_COST = 3.0          # 路网外接入段的代价倍数
    ATTACH_SLACK = 0.5          # 接入候选与最近接入距离的允许差 (m)

    def _static_segments(self):
        sc = SCENARIO_DEFINITIONS.get(self.active_scenario_id, {})
        segs = [tuple(w[:4]) for w in sc.get("walls", [])]
        for sh in sc.get("shelves", []):
            x1, x2 = sorted((sh["x1"], sh["x2"]))
            y1, y2 = sorted((sh["y1"], sh["y2"]))
            segs += [(x1, y1, x2, y1), (x2, y1, x2, y2), (x2, y2, x1, y2), (x1, y2, x1, y1)]
        return segs

    @staticmethod
    def _seg_intersect(a, b, c, d) -> bool:
        def orient(p, q, r):
            return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
        return (o1 * o2 < 0) and (o3 * o4 < 0)

    def _connector_ok(self, p, q, obstacles) -> bool:
        """起终点到路网的接入段: 不穿墙/货架，不被障碍物阻断"""
        if math.hypot(q[0] - p[0], q[1] - p[1]) < 0.02:
            return True
        for w in self._static_segments():
            if self._seg_intersect(p, q, (w[0], w[1]), (w[2], w[3])):
                return False
        return not (obstacles and self._edge_blocked_r(p, q, obstacles, self.robot_half_width))

    def _attach(self, pt, obstacles, blocked_edges):
        """点 → 路网接入候选 [(投影点, [(节点, 沿边距离)], 接入距离, 所在边)]，按接入距离排序"""
        cands, fallback = [], []
        seen = set()
        for u, nbrs in self.edges.items():
            for v, L in nbrs:
                key = tuple(sorted((u, v)))
                if key in seen or key in blocked_edges or L < 1e-6:
                    continue
                seen.add(key)
                (ux, uy), (vx, vy) = self.nodes[u], self.nodes[v]
                t = ((pt[0] - ux) * (vx - ux) + (pt[1] - uy) * (vy - uy)) / (L * L)
                t = max(0.0, min(1.0, t))
                proj = (ux + t * (vx - ux), uy + t * (vy - uy))
                d = math.hypot(pt[0] - proj[0], pt[1] - proj[1])
                if t < 1e-3:
                    links = [(u, 0.0)]
                elif t > 1 - 1e-3:
                    links = [(v, 0.0)]
                else:
                    links = [(u, t * L), (v, (1 - t) * L)]
                c = (proj, links, d, key)
                if self._connector_ok(pt, proj, obstacles):
                    cands.append(c)
                else:
                    fallback.append(c)
        cands.sort(key=lambda c: c[2])
        if not cands and fallback:
            # 车辆贴墙/压在货架边缘等极端情况: 仍接入最近的边 (比直接报 NO_PATH 更可用)
            fallback.sort(key=lambda c: c[2])
            return [c for c in fallback if c[2] < 1.0][:1]
        return cands

    def set_footprint(self, head: float, tail: float, half_width: float, corner_radius: float, clear_min: float = 0.05,
                      allow_arcs=False):
        fp = (round(head, 3), round(tail, 3), round(half_width, 3), round(corner_radius, 3), round(clear_min, 3),
              allow_arcs if isinstance(allow_arcs, str) else ("arc" if allow_arcs else "rotate"))
        if fp != self.footprint:
            self.footprint = fp
            self._corner_cache = {}
            if getattr(self, "_router", None) is not None:
                self._router.set_footprint(fp)

    def _corner_penalty(self, a, b, c) -> float:
        """拐点 b 处 (a→b→c) 车体能否转过去 (静态墙体/货架)；不能则返回惩罚代价"""
        from planning import maneuver
        key = (self.active_scenario_id, round(a[0], 2), round(a[1], 2), round(b[0], 2), round(b[1], 2), round(c[0], 2), round(c[1], 2))
        cache = self.__dict__.setdefault("_corner_cache", {})
        if key not in cache:
            head, tail, hw, r, cmin, arcs = self.footprint
            h1 = math.atan2(b[1] - a[1], b[0] - a[0])
            h2 = math.atan2(c[1] - b[1], c[0] - b[0])
            _, clr = maneuver.plan_corner(self._static_segments(), b, h1, h2, math.hypot(b[0] - a[0], b[1] - a[1]),
                                          math.hypot(c[0] - b[0], c[1] - b[1]), head, tail, hw, r, clear_min=cmin, mode=arcs)
            cache[key] = self.CORNER_BLOCK_COST if clr < cmin else 0.0
        return cache[key]

    def _turn(self, a, b, c) -> float:
        if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-6 or math.hypot(c[0] - b[0], c[1] - b[1]) < 1e-6:
            return 0.0              # 零长度段 (接入点与节点重合) 无方向
        h1 = math.atan2(b[1] - a[1], b[0] - a[0])
        h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        return abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))

    def plan_route(self, start_pt, goal_pt, obstacles=None) -> Dict[str, Any]:
        """→ {"points": [(x,y)...], "labels": [节点名|None...], "length": m}；无路径时 points=[]
        有 libagvnav 时用 C 实现 (planning/native/agvnav.c，与下面的 Python 版逐项一致)；AGV_NATIVE_PLAN=0 用 Python"""
        from planning import native
        if native.lib is not None:
            if getattr(self, "_router", None) is None:
                sc = SCENARIO_DEFINITIONS[self.active_scenario_id]
                self._router = native.Router()
                self._router.set_graph(self.nodes, sc["connections"], self._static_segments())
                self._router.set_footprint(self.footprint)
            return self._router.plan(start_pt, goal_pt, obstacles, self.robot_half_width, self.robot_circum_radius)
        return self._plan_route_py(start_pt, goal_pt, obstacles)

    def _plan_route_py(self, start_pt, goal_pt, obstacles=None) -> Dict[str, Any]:
        obstacles = obstacles or []
        blocked = set()
        if obstacles:
            for u, nbrs in self.edges.items():
                for v, _ in nbrs:
                    if self._is_edge_blocked(self.nodes[u], self.nodes[v], obstacles):
                        blocked.add(tuple(sorted((u, v))))
        # 接入候选: 只保留与最近接入距离相差 < ATTACH_SLACK 的边，脱离路网的接入段按 OFF_NET_COST 倍计价
        # (避免"斜穿"到远处的边上抄近路，路线尽量贴着拓扑)
        def near(c):
            return [x for x in c if x[2] <= c[0][2] + self.ATTACH_SLACK][:3] if c else []
        sc = near(self._attach(start_pt, obstacles, blocked))
        gc = near(self._attach(goal_pt, obstacles, blocked))
        if not sc or not gc:
            return {"points": [], "labels": [], "length": 0.0}
        START, GOAL = "__S__", "__G__"
        best = None
        for s_proj, s_links, s_d, s_edge in sc:
            for g_proj, g_links, g_d, g_edge in gc:
                pos = dict(self.nodes)
                pos[START], pos[GOAL] = s_proj, g_proj
                adj = {k: list(v) for k, v in self.edges.items()}
                for k in list(adj):
                    adj[k] = [(v, w) for v, w in adj[k] if tuple(sorted((k, v))) not in blocked]
                adj[START], adj[GOAL] = [], []
                for n, w in s_links:
                    adj[START].append((n, w))
                    adj[n] = adj.get(n, []) + [(START, w)]
                for n, w in g_links:
                    adj[n] = adj.get(n, []) + [(GOAL, w)]
                    adj[GOAL].append((n, w))
                if s_edge == g_edge:        # 同一条边: 直接沿边
                    adj[START].append((GOAL, math.hypot(s_proj[0] - g_proj[0], s_proj[1] - g_proj[1])))
                # 状态 = (节点, 来自节点)，代价 = 路程 + 转向代价；中间节点被障碍物占据则不可经过
                pq = [(s_d * self.OFF_NET_COST, START, None)]
                cost = {(START, None): s_d * self.OFF_NET_COST}
                par = {}
                done = None
                while pq:
                    c, n, prv = heapq.heappop(pq)
                    if c > cost.get((n, prv), float("inf")) + 1e-9:
                        continue
                    if n == GOAL:
                        done = (n, prv, c)
                        break
                    for m, w in adj.get(n, []):
                        if m == prv or m == START:
                            continue
                        if obstacles and m not in (GOAL,) and m in self.nodes and self._is_node_blocked(self.nodes[m], obstacles):
                            continue
                        tc = 0.0
                        if prv is not None:
                            ang = self._turn(pos[prv], pos[n], pos[m])
                            tc = ang * self.TURN_COST_PER_RAD + (self.CORNER_STOP_COST if ang > 0.02 else 0.0)
                            if ang > 0.02 and self.footprint and n in self.nodes:
                                tc += self._corner_penalty(pos[prv], pos[n], pos[m])
                        nc = c + w + tc
                        if nc < cost.get((m, n), float("inf")) - 1e-9:
                            cost[(m, n)] = nc
                            par[(m, n)] = (n, prv)
                            heapq.heappush(pq, (nc, m, n))
                if not done:
                    continue
                total = done[2] + g_d * self.OFF_NET_COST
                if best is None or total < best[0]:
                    seq = []
                    st = (done[0], done[1])
                    while st in par:
                        seq.append(st[0])
                        st = par[st]
                    seq.append(START)
                    seq.reverse()
                    best = (total, seq, pos)
        if not best:
            return {"points": [], "labels": [], "length": 0.0}
        _, seq, pos = best
        pts, labels = [tuple(start_pt)], [None]
        for n in seq:
            p = pos[n]
            lab = None if n in (START, GOAL) else n
            if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 0.08:
                pts.append(tuple(p))
                labels.append(lab)
            elif lab and not labels[-1]:
                labels[-1] = lab
        if math.hypot(goal_pt[0] - pts[-1][0], goal_pt[1] - pts[-1][1]) > 0.08:
            pts.append(tuple(goal_pt))
            labels.append(None)
        # 车辆已在路网附近 (< 35 cm): 不再先开到投影点 (避免原地转向去走一段极短的接入段)，
        # 而是以投影点作为首段参考线起点 —— 首段即拓扑边本身，车辆沿边横向收敛
        if len(pts) > 2 and math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]) < 0.35:
            del pts[0], labels[0]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        return {"points": pts, "labels": labels, "length": round(length, 2)}

    def plan(self, start_pt: Tuple[float, float], goal_pt: Tuple[float, float], obstacles: List[Tuple[float, float, float, float]] = None,
             start_yaw: Optional[float] = None, goal_yaw: Optional[float] = None) -> List[Tuple[float, float]]:
        """start_yaw 给出时按"带车头朝向"规划 (plan_route_dir): 结果 last_route 里多出 reverse (每段是否倒车) 和 blocked"""
        if start_yaw is not None and self.footprint and self.allow_reverse:
            r = self.plan_route_dir(start_pt, goal_pt, obstacles, start_yaw, goal_yaw)
        else:
            r = self.plan_route(start_pt, goal_pt, obstacles)
        self.last_route = r
        return r["points"]

    # ------------------------------------------------------------------ 带车头朝向的规划 (允许倒车)
    # plan_route 的状态只有 (节点, 来向)，默认每一段都车头朝前开，起点的车头朝向和终点要求的朝向不参与规划；
    # 拐点/终点原地转不开时只能加惩罚硬选，到现场才失败 (窄巷道里的工位: 车头朝里开进去，要求车头朝外，掉不了头)。
    # 这里把"这一段前进还是倒车"放进状态: 车身朝向 = 路段方向 (前进) 或其反向 (倒车)，
    # 节点上要转的角度、扫掠净空都按实际车身朝向算；起点朝向、终点朝向也计入代价。
    # 于是"在巷道口掉头再倒车进去""倒车退出死胡同""绕到另一头正着开进去"都成为可比较的候选，按代价取最小。
    REVERSE_COST = 3.0          # 倒车每米折算的路程倍数 (倒车慢、视野差: 掉个头就能前进时不选倒车，转不开或要绕很远才倒)
    DIR_SWITCH_COST = 1.0       # 前进/倒车切换一次 (停车换向) 折算的路程 (m)
    allow_reverse = True        # False: 始终按 plan_route (只前进)
    rot_blocked: List[Tuple[float, float]] = []   # 现场证实原地转不开的位置 (执行进程在转向受阻、挪车也失败后登记；新任务时清空)

    def mark_rot_blocked(self, x: float, y: float):
        self.rot_blocked = list(self.rot_blocked) + [(float(x), float(y))]

    def _rot_marked(self, p) -> bool:
        return any(math.hypot(p[0] - q[0], p[1] - q[1]) < 0.6 for q in self.rot_blocked)

    def _rot_penalty(self, p, h_from: float, h_to: float) -> float:
        """在点 p 车身从 h_from 原地转到 h_to (两个方向任选) 的净空是否足够；不够返回 CORNER_BLOCK_COST"""
        from planning import maneuver
        key = ("rot", self.active_scenario_id, round(p[0], 2), round(p[1], 2), round(h_from, 3), round(h_to, 3))
        cache = self.__dict__.setdefault("_corner_cache", {})
        if key not in cache:
            head, tail, hw, r, cmin, _ = self.footprint
            _, clr = maneuver.plan_corner(self._static_segments(), p, h_from, h_to, 1.0, 1.0, head, tail, hw, r,
                                          clear_min=cmin, mode="rotate")
            cache[key] = self.CORNER_BLOCK_COST if clr < cmin else 0.0
        return cache[key]

    def plan_route_dir(self, start_pt, goal_pt, obstacles=None, start_yaw: float = 0.0, goal_yaw: Optional[float] = None) -> Dict[str, Any]:
        """→ {"points", "labels", "length", "reverse": [每段是否倒车] (len = len(points) - 1),
              "blocked": [(x, y, 说明)] 仍然转不开的位置 (所有候选都被挡时才会有)}"""
        obstacles = obstacles or []
        blocked = set()
        if obstacles:
            for u, nbrs in self.edges.items():
                for v, _ in nbrs:
                    if self._is_edge_blocked(self.nodes[u], self.nodes[v], obstacles):
                        blocked.add(tuple(sorted((u, v))))

        def near(c):
            return [x for x in c if x[2] <= c[0][2] + self.ATTACH_SLACK][:3] if c else []
        sc = near(self._attach(start_pt, obstacles, blocked))
        gc = near(self._attach(goal_pt, obstacles, blocked))
        empty = {"points": [], "labels": [], "length": 0.0, "reverse": [], "blocked": []}
        if not sc or not gc:
            return empty
        START, GOAL = "__S__", "__G__"
        wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
        best = None
        for s_proj, s_links, s_d, s_edge in sc:
            for g_proj, g_links, g_d, g_edge in gc:
                pos = dict(self.nodes)
                pos[START], pos[GOAL] = s_proj, g_proj
                adj = {k: [(v, w) for v, w in vs if tuple(sorted((k, v))) not in blocked] for k, vs in self.edges.items()}
                adj[START], adj[GOAL] = [], []
                for n, w in s_links:
                    adj[START].append((n, w))
                for n, w in g_links:
                    adj[n] = adj.get(n, []) + [(GOAL, w)]
                if s_edge == g_edge:
                    adj[START].append((GOAL, math.hypot(s_proj[0] - g_proj[0], s_proj[1] - g_proj[1])))
                # 状态 = (节点, 来自节点, 到达时是否倒车, 到达时的车身朝向 [取整到 0.01 rad])；hb = 车身朝向的精确值。
                # 朝向本可由 (来自节点 → 节点, 是否倒车) 推出，但零长度段 (接入点与节点重合、起点终点同处) 上朝向是继承来的
                s0 = (START, None, False, round(start_yaw, 2))
                cost = {s0: s_d * self.OFF_NET_COST}
                hb = {s0: start_yaw}
                par, note = {}, {}
                pq = [(cost[s0], 0, s0)]
                tie = 1
                fin = None                      # (总代价, 状态, 终点转向说明)
                while pq:
                    c, _, st = heapq.heappop(pq)
                    if fin is not None and c >= fin[0]:
                        break
                    if c > cost.get(st, float("inf")) + 1e-9:
                        continue
                    n, prv, rev = st[:3]
                    h_body = hb[st]
                    if n == GOAL:
                        tc, why = 0.0, None
                        if goal_yaw is not None:
                            d = abs(wrap(goal_yaw - h_body))
                            if d > 0.05:
                                pen = self.CORNER_BLOCK_COST if self._rot_marked(goal_pt) else self._rot_penalty(goal_pt, h_body, goal_yaw)
                                tc = d * self.TURN_COST_PER_RAD + self.CORNER_STOP_COST + pen
                                why = (goal_pt[0], goal_pt[1], "终点对位转向") if pen else None
                        if fin is None or c + tc < fin[0]:
                            fin = (c + tc, st, why)
                        continue
                    for m, w in adj.get(n, []):
                        if m == START:
                            continue
                        if obstacles and m != GOAL and m in self.nodes and self._is_node_blocked(self.nodes[m], obstacles):
                            continue
                        zero = w < 1e-6
                        h_seg = None if zero else math.atan2(pos[m][1] - pos[n][1], pos[m][0] - pos[n][0])
                        for rev2 in ((rev,) if zero else (False, True)):
                            h2 = h_body if zero else wrap(h_seg + (math.pi if rev2 else 0.0))
                            d = abs(wrap(h2 - h_body))
                            tc, why = 0.0, None
                            if d > 0.02:
                                tc = d * self.TURN_COST_PER_RAD + self.CORNER_STOP_COST
                                geo = prv is not None and math.hypot(pos[n][0] - pos[prv][0], pos[n][1] - pos[prv][1]) > 1e-6 and \
                                    abs(wrap(math.atan2(pos[n][1] - pos[prv][1], pos[n][0] - pos[prv][0]) - h_body)) < 0.02
                                if geo and not rev and not rev2 and n in self.nodes and prv != m:
                                    pen = self._corner_penalty(pos[prv], pos[n], pos[m])      # 前进 → 前进: 可用圆弧过弯
                                else:
                                    at = start_pt if prv is None else pos[n]
                                    pen = self._rot_penalty(at, h_body, h2)
                                if self._rot_marked(start_pt if prv is None else pos[n]):
                                    pen = self.CORNER_BLOCK_COST          # 现场证实转不开 (地图上看不出来的障碍/定位偏差)
                                if pen:
                                    tc += pen
                                    why = ((start_pt if prv is None else pos[n])[0], (start_pt if prv is None else pos[n])[1],
                                           "起步转向" if prv is None else "拐点转向")
                            elif prv is not None and rev2 != rev:
                                tc = self.DIR_SWITCH_COST             # 不转车身，停车换向 (前进 ↔ 倒车)
                            elif prv is not None and m == prv:
                                continue                              # 同向原路返回没有意义
                            nc = c + w * (self.REVERSE_COST if rev2 else 1.0) + tc
                            st2 = (m, n, rev2, round(h2, 2))
                            if nc < cost.get(st2, float("inf")) - 1e-9:
                                cost[st2], hb[st2], par[st2], note[st2] = nc, h2, st, why
                                heapq.heappush(pq, (nc, tie, st2))
                                tie += 1
                if fin is None:
                    continue
                total = fin[0] + g_d * self.OFF_NET_COST
                if best is None or total < best[0]:
                    chain = []
                    st = fin[1]
                    while st in par:
                        chain.append(st)
                        st = par[st]
                    chain.reverse()
                    best = (total, chain, pos, [note[x] for x in chain if note.get(x)] + ([fin[2]] if fin[2] else []))
        if not best:
            return empty
        _, chain, pos, blk = best
        pts, labels, revs = [tuple(start_pt)], [None], []
        first_rev = chain[0][2] if chain else False
        if chain and math.hypot(pos[START][0] - pts[0][0], pos[START][1] - pts[0][1]) > 0.08:
            pts.append(tuple(pos[START])); labels.append(None); revs.append(first_rev)      # 接入段与第一段同向
        for n, _prv, rev, _h in chain:
            p = pos[n]
            lab = None if n == GOAL else n
            if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 0.08:
                pts.append(tuple(p)); labels.append(lab); revs.append(rev)
            elif lab and not labels[-1]:
                labels[-1] = lab
        if math.hypot(goal_pt[0] - pts[-1][0], goal_pt[1] - pts[-1][1]) > 0.08:
            pts.append(tuple(goal_pt)); labels.append(None); revs.append(revs[-1] if revs else False)
        if len(pts) > 2 and math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]) < 0.35 and revs[0] == revs[1]:
            del pts[0], labels[0], revs[0]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        return {"points": pts, "labels": labels, "length": round(length, 2), "reverse": revs,
                "blocked": [(round(x, 2), round(y, 2), w) for x, y, w in blk]}

    def plan_legacy(self, start_pt: Tuple[float, float], goal_pt: Tuple[float, float], obstacles: List[Tuple[float, float, float, float]] = None) -> List[Tuple[float, float]]:
        obstacles = obstacles or []
        start_node = self._find_nearest_node(start_pt, obstacles)
        goal_node = self._find_nearest_node(goal_pt, obstacles)

        if not start_node or not goal_node:
            return [start_pt, goal_pt]

        if start_node == goal_node:
            return [start_pt, self.nodes[start_node], goal_pt]

        # Dijkstra Min-Heap Priority Queue
        dist_map = {node: float('inf') for node in self.nodes}
        parent_map = {}
        dist_map[start_node] = 0.0

        pq = [(0.0, start_node)]

        while pq:
            d_curr, u = heapq.heappop(pq)
            if d_curr > dist_map[u]:
                continue

            if u == goal_node:
                break

            for v, weight in self.edges.get(u, []):
                if obstacles and (self._is_edge_blocked(self.nodes[u], self.nodes[v], obstacles)
                                  or (v != goal_node and self._is_node_blocked(self.nodes[v], obstacles))):
                    continue

                new_dist = d_curr + weight
                if new_dist < dist_map[v]:
                    dist_map[v] = new_dist
                    parent_map[v] = u
                    heapq.heappush(pq, (new_dist, v))

        if goal_node not in parent_map and start_node != goal_node:
            return []  # 无可行路径 (被障碍物阻断)：不再退化为穿越货架的直线

        # Reconstruct path
        node_path = []
        curr = goal_node
        while curr in parent_map:
            node_path.append(curr)
            curr = parent_map[curr]
        node_path.append(start_node)
        node_path.reverse()

        # Build clean, non-redundant waypoint list
        raw_coords = [start_pt]
        for nid in node_path:
            raw_coords.append(self.nodes[nid])
        raw_coords.append(goal_pt)

        clean_coords = [raw_coords[0]]
        for pt in raw_coords[1:]:
            if math.hypot(pt[0] - clean_coords[-1][0], pt[1] - clean_coords[-1][1]) > 0.08:
                clean_coords.append(pt)

        return clean_coords
