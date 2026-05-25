import pybullet as p
import pybullet_data
import time
from robot_controller import RobotController
from pick_and_place import PickAndPlace, FSMState

# =====================
# 1. 시뮬레이션 초기화
# =====================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(1. / 240.)
p.setRealTimeSimulation(0)

# =====================
# 2. 조명 및 카메라 설정
# =====================
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0.5, 0, 0.2]
)

# =====================
# 3. 환경 로드
# =====================
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF(
    "franka_panda/panda.urdf",
    basePosition=[0, 0, 0.05],
    useFixedBase=True
)

# =====================
# 4. 물체 배치 (빨간 박스)
# =====================
box_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.03, 0.03, 0.03],
    rgbaColor=[1, 0, 0, 1]  # 빨간색
)
box_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[0.03, 0.03, 0.03]
)
box = p.createMultiBody(
    baseMass=0.1,
    baseCollisionShapeIndex=box_collision,
    baseVisualShapeIndex=box_visual,
    basePosition=[0.5, 0, 0.03]  # 물체 초기 위치
)

print("✅ 환경 로드 완료!")

# =====================
# 5. FSM Pick & Place 실행
# =====================
controller = RobotController(robot)
# 기존
controller = RobotController(robot)
fsm = PickAndPlace(controller)

# 수정
controller = RobotController(robot)

# 초기 자세 안정화 (1초)
print("✅ 초기 자세 안정화 중...")
for _ in range(240):
    p.stepSimulation()
    import time
    time.sleep(1./240.)
print("✅ 안정화 완료!")

fsm = PickAndPlace(controller)

object_pos = [0.5, 0.0, 0.03]   # 물체 위치
goal_pos   = [0.5, 0.3, 0.03]   # 목표 위치

print("✅ Pick & Place 시작!")

while True:
    state = fsm.run(object_pos, goal_pos)
    p.stepSimulation()
    time.sleep(1. / 240.)

    if state == FSMState.DONE:
        print("✅ Pick & Place 완료!")
        break

print("종료하려면 Ctrl+C 를 누르세요.")
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)