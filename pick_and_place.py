import pybullet as p

class FSMState:
    APPROACH = "APPROACH"
    DESCEND  = "DESCEND"
    GRASP    = "GRASP"
    LIFT     = "LIFT"
    MOVE     = "MOVE"
    PLACE    = "PLACE"
    RELEASE  = "RELEASE"
    DONE     = "DONE"

class PickAndPlace:
    def __init__(self, controller):
        self.controller  = controller
        self.state       = FSMState.APPROACH
        self.step_count  = 0
        self.steps_per_state = 240

    def run(self, object_pos, goal_pos):
        # 각 상태별 목표 좌표
        above_object = [object_pos[0], object_pos[1], object_pos[2] + 0.2]
        grasp_pos    = [object_pos[0], object_pos[1], object_pos[2] + 0.03]
        lift_pos     = [object_pos[0], object_pos[1], object_pos[2] + 0.35]
        above_goal   = [goal_pos[0],   goal_pos[1],   goal_pos[2]   + 0.2]
        place_pos    = [goal_pos[0],   goal_pos[1],   goal_pos[2]   + 0.03]

        print(f"현재 상태: {self.state}", end="\r")

        if self.state == FSMState.APPROACH:
            self.controller.open_gripper()
            self.controller.move_to(above_object)
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.DESCEND
                self.step_count = 0

        elif self.state == FSMState.DESCEND:
            self.controller.move_to(grasp_pos)
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.GRASP
                self.step_count = 0

        elif self.state == FSMState.GRASP:
            self.controller.close_gripper()
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.LIFT
                self.step_count = 0

        elif self.state == FSMState.LIFT:
            self.controller.move_to(lift_pos)
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.MOVE
                self.step_count = 0

        elif self.state == FSMState.MOVE:
            self.controller.move_to(above_goal)
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.PLACE
                self.step_count = 0

        elif self.state == FSMState.PLACE:
            self.controller.move_to(place_pos)
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.RELEASE
                self.step_count = 0

        elif self.state == FSMState.RELEASE:
            self.controller.open_gripper()
            self.step_count += 1
            if self.step_count >= self.steps_per_state:
                self.state = FSMState.DONE
                self.step_count = 0

        return self.state