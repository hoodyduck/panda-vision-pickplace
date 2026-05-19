import numpy as np

class JointSmoother:
    def __init__(self, max_delta=0.05, alpha=0.85):
        """
        max_delta: 프레임당 최대 허용 각도 변화량 (라디안)
        alpha:     이전 각도 반영 비율 (높을수록 부드러움)
        """
        self.prev_angles = None
        self.max_delta = max_delta
        self.alpha = alpha

    def smooth(self, target_angles):
        # 최초 실행 시 초기화
        if self.prev_angles is None:
            self.prev_angles = list(target_angles)
            return list(target_angles)

        smoothed = []
        for prev, target in zip(self.prev_angles, target_angles):
            # 변화량 계산
            delta = target - prev

            # 최대 변화량 제한
            if abs(delta) > self.max_delta:
                delta = np.sign(delta) * self.max_delta

            # 지수 이동 평균 적용
            new_angle = self.alpha * prev + (1 - self.alpha) * (prev + delta)
            smoothed.append(new_angle)

        self.prev_angles = smoothed
        return smoothed