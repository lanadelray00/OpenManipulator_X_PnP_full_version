import numpy as np
import math
from ArUco_coord_extractor import MarkerPoseProcessor


# ===== Dummy robot class =====
class DummyRobot:
    current_position = [0, 0, 0]
    current_orientation = [0, 0, 0, 1]

# ===== Processor 생성 =====
processor = MarkerPoseProcessor(DummyRobot())

# recording 상태라고 가정
processor.recording = True

# ===== 가짜 30프레임 데이터 생성 =====
def make_fake_pose(x, y, z):
    # quaternion (0,0,0,1) 고정
    return (x, y, z, 0, 0, 0, 1)

# ID 1 - 그룹 0
processor.buffer[(1,0)] = [make_fake_pose(0.10, 0.00, 0.20) for _ in range(30)]

# ID 1 - 그룹 1 (같은 ID, 다른 위치)
processor.buffer[(1,1)] = [make_fake_pose(0.15, 0.05, 0.25) for _ in range(30)]

# ID 2 - 그룹 0
processor.buffer[(2,0)] = [make_fake_pose(0.30, 0.10, 0.20) for _ in range(30)]

# 준비 안된 그룹
processor.buffer[(3,0)] = [make_fake_pose(0.50, 0.20, 0.20) for _ in range(10)]

# ===== 테스트 =====
results = processor.get_refined_poses()

print("Returned poses:")
for r in results:
    marker_id, group_index, x, y, z, qx, qy, qz, qw = r
    print(
        f"ID={marker_id}, G={group_index} | "
        f"X={x:.3f}, Y={y:.3f}, Z={z:.3f}"
    )
print("\nRemaining buffer keys:")
print(processor.buffer.keys())
