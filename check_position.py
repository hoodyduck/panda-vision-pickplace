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

p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 물체 생성
box_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.03, 0.03, 0.03],
    rgbaColor=[1, 0, 0, 1]
)
box_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[0.03, 0.03, 0.03]
)
box = p.createMultiBody(
    baseMass=0.1,
    baseCollisionShapeIndex=box_collision,
    baseVisualShapeIndex=box_visual,
    basePosition=[0.5, -0.1, 0.03]
)

# 안정화
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

# 물체 실제 위치 확인
box_pos, _ = p.getBasePositionAndOrientation(box)
print(f"물체 실제 위치: {box_pos}")

# 그리퍼 끝점 현재 위치 확인
gripper_state = p.getLinkState(robot, 11)
print(f"그리퍼 끝점 위치: {gripper_state[0]}")

# 로봇 베이스 위치 확인
base_pos, _ = p.getBasePositionAndOrientation(robot)
print(f"로봇 베이스 위치: {base_pos}")

p.disconnect()