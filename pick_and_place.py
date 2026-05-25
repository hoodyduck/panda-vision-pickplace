import pybullet as p
from strategies import MotionStrategy, Strategy

class FSMState:
    PREPARE  = "PREPARE"
    GRASP    = "GRASP"
    LIFT     = "LIFT"
    MOVE     = "MOVE"
    PLACE    = "PLACE"
    RELEASE  = "RELEASE"
    DONE     = "DONE"

class PickAndPlace:
    def __init__(self, controller, strategy_name=Strategy.TOP_DOWN):
        self.controller      = controller
        self.strategy        = MotionStrategy(strategy_name)
        self.strategy_name   = strategy_name
        self.state           = FSMState.PREPARE
        self.step_count      = 0
        self.steps_per_state = 240
        self.waypoint_index  = 0
        self.waypoints       = []

    def _get_final_approach(self, object_pos):
        """전략별 최종 집기 직전 접근 동작"""
        obj = object_pos
        if self.strategy_name == Strategy.TOP_DOWN:
            # 바로 위에서 수직으로
            return [obj[0], obj[1], obj[2] + 0.15]
        elif self.strategy_name == Strategy.SIDE:
            # 옆에서 수평으로 마지막 접근
            return [obj[0] - 0.1, obj[1], obj[2] + 0.03]
        elif self.strategy_name == Strategy.DIAGONAL:
            # 대각선으로 마지막 접근
            return [obj[0] - 0.05, obj[1] - 0.05, obj[2] + 0.1]
        elif self.strategy_name == Strategy.ARC:
            # 호의 끝에서 위로 내려옴
            return [obj[0], obj[1], obj[2] + 0.15]

    def run(self, object_pos, goal_pos):
        if not self.waypoints:
            self.waypoints = self.strategy.get_waypoints(object_pos, goal_pos)
            print(f"\n전략: {self.strategy_name}")
            print(f"경유지 수: {len(self.waypoints)}")

        lift_pos   = [object_pos[0], object_pos[1], object_pos[2] + 0.35]
        above_goal = [goal_pos[0],   goal_pos[1],   goal_pos[2]   + 0.2]
        place_pos  = [goal_pos[0],   goal_pos[1],   goal_pos[2]   + 0.03]
        final_approach = self._get_final_approach(object_pos)
        grasp_pos  = [object_pos[0], object_pos[1], object_pos[2] + 0.03]

        print(f"현재 상태: {self.state}", end="\r")

        # ① 전략 경유지 이동
        if self.state == FSMState.PREPARE:
            self.controller.open_gripper()

            # 마지막 경유지 전에 최종 접근 동작 추가
            all_waypoints = self.waypoints[:-1] + \
                            [final_approach] + \
                            [grasp_pos]

            if self.waypoint_index < len(all_waypoints):
                target = all_waypoints[self.waypoint_index]
                self.controller.move_to(target)
                self.step_count += 1
                if self.step_count >= self.steps_per_state:
                    self.waypoint_index += 1
                    self.step_count = 0
            else:
                self.state = FSMState.GRASP
                self.step_count = 0

        # ② 집기
        elif self.state == FSMState.GRASP:
            self.controller.close_gripper()
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.LIFT
                self.step_count = 0

        # ③ 들기
        elif self.state == FSMState.LIFT:
            self.controller.move_to(lift_pos)
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.MOVE
                self.step_count = 0

        # ④ 목표 위치로 이동
        elif self.state == FSMState.MOVE:
            self.controller.move_to(above_goal)
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.PLACE
                self.step_count = 0

        # ⑤ 내려놓기
        elif self.state == FSMState.PLACE:
            self.controller.move_to(place_pos)
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.RELEASE
                self.step_count = 0

        # ⑥ 그리퍼 열기
        elif self.state == FSMState.RELEASE:
            self.controller.open_gripper()
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.DONE
                self.step_count = 0

        return self.state