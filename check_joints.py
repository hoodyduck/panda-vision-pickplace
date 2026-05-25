import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 초기 자세로 잠깐 안정화
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

# 그리퍼 끝점(11번) 현재 위치 출력
state = p.getLinkState(robot, 11)
print(f"그리퍼 끝점 현재 위치: {state[0]}")
print(f"그리퍼 끝점 현재 방향: {state[1]}")

p.disconnect()