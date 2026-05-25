import pybullet as p
import pybullet_data
import time
from robot_controller import RobotController
from pick_and_place import PickAndPlace, FSMState
from strategies import Strategy

# =====================
# 1. 시뮬레이션 초기화
# =====================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(1./240.)
p.setRealTimeSimulation(0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0.5, 0, 0.2]
)

# =====================
# 2. 환경 로드
# =====================
p.loadURDF("plane.urdf")
robot = p.loadURDF(
    "franka_panda/panda.urdf",
    basePosition=[0, 0, 0.05],
    useFixedBase=True
)

# =====================
# 3. 컨트롤러 초기화
# =====================
controller = RobotController(robot)

print("✅ 초기 자세 안정화 중...")
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)
print("✅ 안정화 완료!")

# =====================
# 4. 4가지 전략 순서대로 테스트
# =====================
strategies = [
    Strategy.TOP_DOWN,
    Strategy.SIDE,
    Strategy.DIAGONAL,
    Strategy.ARC,
]

object_pos = [0.5, 0.0, 0.03]
goal_pos   = [0.3, 0.3, 0.03]

for strategy_name in strategies:
    print(f"\n{'='*40}")
    print(f"전략 시작: {strategy_name}")
    print(f"{'='*40}")

    # 물체를 초기화 완료 후 생성 ← 여기로 이동
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
        basePosition=object_pos
    )

    # 물체 생성 후 잠깐 안정화
    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./240.)
    # FSM 초기화
    fsm = PickAndPlace(controller, strategy_name)

    # 실행
    while True:
        state = fsm.run(object_pos, goal_pos)
        p.stepSimulation()
        time.sleep(1./240.)

        if state == FSMState.DONE:
            print(f"\n✅ {strategy_name} 전략 완료!")
            break

    # 다음 전략 전 물체 제거 및 로봇 초기화
    p.removeBody(box)
    controller.reset_pose()

    # 안정화 시간 충분히 확보 (1초 → 2초)
    print("초기 자세로 복귀 중...")
    for _ in range(480):  # 240 → 480
        p.stepSimulation()
        time.sleep(1./240.)
    print("복귀 완료!")
print("\n✅ 모든 전략 테스트 완료!")
print("종료하려면 Ctrl+C 를 누르세요.")
while True:
    p.stepSimulation()
    time.sleep(1./240.)