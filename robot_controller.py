import pybullet as p
import time
from utils import JointSmoother

class RobotController:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.smoother = JointSmoother(max_delta=0.05, alpha=0.85)

        # Franka Panda 관절 인덱스
        self.joint_indices = [0, 1, 2, 3, 4, 5, 6]  # 7개 관절
        self.gripper_indices = [9, 10]                # 그리퍼
        self.end_effector_index = 11                  # 그리퍼 끝점

    def move_to(self, target_position, target_orientation=None):
        """
        목표 좌표로 팔 이동
        target_position: [x, y, z]
        """
        if target_orientation is None:
            target_orientation = p.getQuaternionFromEuler([3.14, 0, 0])

        # IK 계산
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            self.end_effector_index,
            target_position,
            target_orientation,
            maxNumIterations=100,
            residualThreshold=1e-5
        )

        # Smoothing 적용
        smoothed_angles = self.smoother.smooth(joint_angles[:7])

        # 관절에 적용
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