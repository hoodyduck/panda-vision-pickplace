import pybullet as p
import pybullet_data
import time
from robot_controller import RobotController

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
    basePosition=[0, 0, 0],
    useFixedBase=True
)

print("환경 로드 완료")

# =====================
# 4. 로봇 컨트롤러 초기화
# =====================
controller = RobotController(robot)

# =====================
# 5. IK 테스트 — 목표 좌표로 이동
# =====================
target_positions = [
    [0.5, 0.0, 0.5],   # 위치 1
    [0.5, 0.3, 0.5],   # 위치 2
    [0.6, -0.2, 0.6],  # 위치 3
]

print("IK 테스트 시작")

for i, target in enumerate(target_positions):
    print(f"→ 목표 위치 {i+1}: {target}")

    # 목표 위치로 이동 (240프레임 = 1초)
    for _ in range(240):
        controller.move_to(target)
        p.stepSimulation()
        time.sleep(1. / 240.)

    print(f"✅ 위치 {i+1} 도달 완료!")

print("모든 IK 테스트 완료")
print("종료하려면 Ctrl+C 를 누르세요.")

while True:
    p.stepSimulation()
    time.sleep(1. / 240.)