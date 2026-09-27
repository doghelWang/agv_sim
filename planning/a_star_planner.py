#!/usr/bin/env python3
"""
A* Grid Path Planner with Obstacle Inflation & Path Smoothing (A* 栅格启发式路径规划器)
"""

import math
import heapq
from typing import List, Tuple, Dict, Optional, Set


class AStarPlanner:
    def __init__(self, resolution: float = 0.1, inflation_radius: float = 0.5):
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        self.inflation_cells = int(math.ceil(self.inflation_radius / self.resolution))

    def _world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        return int(round(wx / self.resolution)), int(round(wy / self.resolution))

    def _grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return round(gx * self.resolution, 3), round(gy * self.resolution, 3)

    def _build_obstacle_grid(self, walls: List[Tuple[float, float, float, float]]) -> Set[Tuple[int, int]]:
        """Rasterize wall segments into grid cells with safety inflation."""
        occupied = set()

        for w in walls:
            x0, y0, x1, y1 = w
            dist = math.hypot(x1 - x0, y1 - y0)
            steps = max(1, int(dist / (self.resolution * 0.5)))

            for s in range(steps + 1):
                t = s / float(steps)
                wx = x0 + (x1 - x0) * t
                wy = y0 + (y1 - y0) * t
                cgx, cgy = self._world_to_grid(wx, wy)

                # Inflate around obstacle
                for dx in range(-self.inflation_cells, self.inflation_cells + 1):
                    for dy in range(-self.inflation_cells, self.inflation_cells + 1):
                        if math.hypot(dx, dy) <= self.inflation_cells:
                            occupied.add((cgx + dx, cgy + dy))

        return occupied

    @staticmethod
    def _heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        # Octile distance heuristic
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)

    def plan(self, start_w: Tuple[float, float], goal_w: Tuple[float, float], walls: List[Tuple[float, float, float, float]]) -> List[Tuple[float, float]]:
        start = self._world_to_grid(start_w[0], start_w[1])
        goal = self._world_to_grid(goal_w[0], goal_w[1])

        obstacles = self._build_obstacle_grid(walls)

        # 8-connected grid motions
        motions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)), (-1, -1, math.sqrt(2))
        ]

        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct raw path
                path = []
                curr = current
                while curr in came_from:
                    path.append(self._grid_to_world(curr[0], curr[1]))
                    curr = came_from[curr]
                path.append(self._grid_to_world(start[0], start[1]))
                path.reverse()
                return self._smooth_path(path, obstacles)

            for dx, dy, cost in motions:
                neighbor = (current[0] + dx, current[1] + dy)

                if neighbor in obstacles and neighbor != goal:
                    continue

                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        # Fallback direct line if blocked
        return [start_w, goal_w]

    def _smooth_path(self, path: List[Tuple[float, float]], obstacles: Set[Tuple[int, int]]) -> List[Tuple[float, float]]:
        """Line-of-sight shortcutting smoothing."""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        curr_idx = 0

        while curr_idx < len(path) - 1:
            best_next = curr_idx + 1
            # Look ahead as far as possible
            for test_idx in range(len(path) - 1, curr_idx, -1):
                if self._is_line_clear(path[curr_idx], path[test_idx], obstacles):
                    best_next = test_idx
                    break

            smoothed.append(path[best_next])
            curr_idx = best_next

        return smoothed

    def _is_line_clear(self, p0: Tuple[float, float], p1: Tuple[float, float], obstacles: Set[Tuple[int, int]]) -> bool:
        dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        steps = max(1, int(dist / (self.resolution * 0.5)))

        for s in range(steps + 1):
            t = s / float(steps)
            wx = p0[0] + (p1[0] - p0[0]) * t
            wy = p0[1] + (p1[1] - p0[1]) * t
            cell = self._world_to_grid(wx, wy)
            if cell in obstacles:
                return False
        return True
