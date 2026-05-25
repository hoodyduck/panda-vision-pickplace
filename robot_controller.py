import pybullet as p
from utils import JointSmoother

class RobotController:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.smoother = JointSmoother(max_delta=0.05, alpha=0.85)

        self.joint_indices      = [0, 1, 2, 3, 4, 5, 6]
        self.gripper_indices    = [9, 10]
        self.end_effector_index = 11

        self.init_angles = [0.0, -1.0, 0.0, -2.8, 0.0, 2.0, 0.785]

        self.reset_pose()

    def reset_pose(self):
        # 1단계: 관절 강제 초기화
        for i, joint_index in enumerate(self.joint_indices):
            p.resetJointState(
                self.robot_id,
                joint_index,
                self.init_angles[i]
            )

        # 2단계: 모터 제어로 자세 유지
        for i, joint_index in enumerate(self.joint_indices):
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=self.init_angles[i],
                force=500
            )

        self.open_gripper()

        # 3단계: smoother 초기화
        self.smoother.prev_angles = None

        print("✅ 초기 자세 복귀 완료!")

    def move_to(self, target_position, target_orientation=None):
        if target_orientation is None:
            target_orientation = p.getQuaternionFromEuler([3.14159, 0, 0])

        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            self.end_effector_index,
            target_position,
            target_orientation,
            lowerLimits=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
            upperLimits=[ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973],
            jointRanges=[ 5.7946,  3.5256,  5.7946,  3.0020,  5.7946,  3.7700,  5.7946],
            restPoses=   [ 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            maxNumIterations=200,
            residualThreshold=1e-5
        )

        smoothed_angles = self.smoother.smooth(joint_angles[:7])

        for i, joint_index in enumerate(self.joint_indices):
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=smoothed_angles[i],
                force=500
            )

    def open_gripper(self):
        for i in self.gripper_indices:
            p.setJointMotorControl2(
                self.robot_id, i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.04,
                force=100
            )

    def close_gripper(self):
        for i in self.gripper_indices:
            p.setJointMotorControl2(
                self.robot_id, i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.0,
                force=100
            )