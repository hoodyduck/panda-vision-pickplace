import numpy as np

class Strategy:
    TOP_DOWN = "TOP_DOWN"
    SIDE     = "SIDE"
    DIAGONAL = "DIAGONAL"
    ARC      = "ARC"

class MotionStrategy:
    def __init__(self, strategy_name):
        self.strategy_name = strategy_name

    def get_waypoints(self, object_pos, goal_pos):
        if self.strategy_name == Strategy.TOP_DOWN:
            return self._top_down(object_pos, goal_pos)
        elif self.strategy_name == Strategy.SIDE:
            return self._side(object_pos, goal_pos)
        elif self.strategy_name == Strategy.DIAGONAL:
            return self._diagonal(object_pos, goal_pos)
        elif self.strategy_name == Strategy.ARC:
            return self._arc(object_pos, goal_pos)

    def _top_down(self, obj, goal):
        """
        위에서 수직으로 내려오는 전략
        → 팔이 높이 올라갔다가 수직으로 내려옴
        """
        return [
            [obj[0], obj[1], obj[2] + 0.7],   # WP1: 아주 높이 올라감
            [obj[0], obj[1], obj[2] + 0.03],   # WP2: 수직으로 내려옴
        ]

    def _side(self, obj, goal):
        """
        완전히 옆에서 수평으로 접근하는 전략
        → 로봇 왼쪽 멀리서 수평으로 슥 밀고 들어옴
        """
        return [
            [obj[0] - 0.5, obj[1], obj[2] + 0.03],  # WP1: 왼쪽 멀리
            [obj[0] - 0.3, obj[1], obj[2] + 0.03],  # WP2: 수평 접근
            [obj[0],       obj[1], obj[2] + 0.03],  # WP3: 집기 위치
        ]

    def _diagonal(self, obj, goal):
        """
        대각선 아래로 접근하는 전략
        → 앞 대각선 위쪽에서 비스듬히 내려옴
        """
        return [
            [obj[0] - 0.4, obj[1] - 0.4, obj[2] + 0.6],  # WP1: 대각선 멀리 위
            [obj[0] - 0.2, obj[1] - 0.2, obj[2] + 0.3],  # WP2: 대각선 중간
            [obj[0],       obj[1],        obj[2] + 0.03], # WP3: 집기 위치
        ]

    def _arc(self, obj, goal):
        """
        반원 호를 그리며 접근하는 전략
        → 물체 뒤쪽에서 크게 돌아서 앞으로 들어옴
        """
        return [
            [obj[0],        obj[1] - 0.5, obj[2] + 0.5],  # WP1: 뒤쪽으로
            [obj[0] + 0.4,  obj[1] - 0.3, obj[2] + 0.6],  # WP2: 호의 정점
            [obj[0] + 0.4,  obj[1] + 0.3, obj[2] + 0.5],  # WP3: 반대편
            [obj[0],        obj[1],        obj[2] + 0.03], # WP4: 집기 위치
        ]