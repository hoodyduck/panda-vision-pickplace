import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(1./240.)

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0.5, 0, 0.2]
)

robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
p.loadURDF("plane.urdf")

# 테스트할 orientation 목록
orientations = [
    ("테스트 1", p.getQuaternionFromEuler([0, 0, 0])),
    ("테스트 2", p.getQuaternionFromEuler([3.14159, 0, 0])),
    ("테스트 3", p.getQuaternionFromEuler([0, 1.5708, 0])),
    ("테스트 4", p.getQuaternionFromEuler([3.14159, 1.5708, 0])),
    ("테스트 5", p.getQuaternionFromEuler([1.5708, 0, 0])),
    ("테스트 6", p.getQuaternionFromEuler([0, 0, 1.5708])),
]

target_position = [0.5, 0.0, 0.3]

for name, orientation in orientations:
    print(f"\n{name}: {orientation}")

    # 해당 orientation으로 IK 계산
    joint_angles = p.calculateInverseKinematics(
        robot, 11,
        target_position,
        orientation,
        lowerLimits=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
        upperLimits=[ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973],
        jointRanges=[ 5.7946,  3.5256,  5.7946,  3.0020,  5.7946,  3.7700,  5.7946],
        restPoses=   [ 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        maxNumIterations=200,
        residualThreshold=1e-5
    )

    # 관절 적용
    for i in range(7):
        p.setJointMotorControl2(
            robot, i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_angles[i],
            force=500
        )

    # 2초간 유지하면서 확인
    for _ in range(480):
        p.stepSimulation()
        time.sleep(1./240.)

    print(f"{name} 확인 완료. 다음으로 넘어갑니다...")

p.disconnect()