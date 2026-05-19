import pybullet as p
import pybullet_data
import time

# =====================
# 1. 시뮬레이션 초기화
# =====================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 물리 엔진 파라미터 설정
p.setGravity(0, 0, -9.8)
p.setTimeStep(1. / 240.)
p.setRealTimeSimulation(0)  # 수동 스텝 모드

# =====================
# 2. 조명 설정
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
# 3. 바닥 및 로봇 로드
# =====================
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF(
    "franka_panda/panda.urdf",
    basePosition=[0, 0, 0],
    useFixedBase=True
)

print("기본 환경 로드 완료!")
print(f"로봇 관절 수: {p.getNumJoints(robot)}")
print("종료하려면 Ctrl+C 를 누르세요.")

# =====================
# 4. 시뮬레이션 실행
# =====================
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)