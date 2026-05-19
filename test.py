import pybullet as p
import pybullet_data
import time

# 시뮬레이션 시작
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 바닥 + 로봇 팔 로드
p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

print("✅ Franka Panda 로드 성공!")
print("창을 닫으려면 Ctrl+C 를 누르세요.")

# 수동으로 닫을 때까지 무한 실행
while True:
    p.stepSimulation()
    time.sleep(1./240.)