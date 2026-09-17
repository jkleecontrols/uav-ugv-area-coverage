import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import json
import time
from zconfig import CONFIG
from zdataloader import DataLoader
import math
from matplotlib.patches import Rectangle, Polygon

# DataLoader 초기화
loader = DataLoader('map/tamu_map.csv')
grid_points = np.array(loader.get_grid_points())
boundary_points = loader.get_boundary_points()

# 방문 기록을 위한 그리드 초기화
visit_count = np.zeros(len(grid_points))
last_visited_time = np.zeros(len(grid_points))

# FOV 계산을 위한 상수
uav_height = CONFIG["uav_height"]
fov_angle = math.radians(CONFIG["uav_fov_angle"])
aspect_ratio = CONFIG["uav_aspect_ratio"]

# FOV 직사각형의 크기 계산
fov_width = 2 * uav_height * math.tan(fov_angle/2) * math.sqrt(1/(1 + aspect_ratio**2))
fov_height = fov_width * aspect_ratio

def is_point_in_rotated_rect(point, center, width, height, angle):
    """회전된 직사각형 내에 점이 있는지 확인"""
    dx = point[0] - center[0]
    dy = point[1] - center[1]
    
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    
    rotated_x = dx * cos_angle + dy * sin_angle
    rotated_y = -dx * sin_angle + dy * cos_angle
    
    return (abs(rotated_x) <= width/2) and (abs(rotated_y) <= height/2)

def get_rotated_rect_corners(center, width, height, angle):
    """회전된 직사각형의 모서리 좌표 계산"""
    half_width = width / 2
    half_height = height / 2
    
    corners = [
        [-half_width, -half_height],
        [half_width, -half_height],
        [half_width, half_height],
        [-half_width, half_height]
    ]
    
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    
    rotated_corners = []
    for corner in corners:
        x = corner[0] * cos_angle - corner[1] * sin_angle + center[0]
        y = corner[0] * sin_angle + corner[1] * cos_angle + center[1]
        rotated_corners.append([x, y])
    
    return rotated_corners

def update_visit_grid(uav_positions, current_time):
    """UAV 위치와 방향을 기반으로 방문 그리드 업데이트"""
    global visit_count, last_visited_time
    
    for uav_pos in uav_positions:
        if uav_pos is None or len(uav_pos) < 3:
            continue
            
        try:
            center = uav_pos[:2]
            heading = uav_pos[2]
            if heading is None:  # heading이 None인 경우 처리
                continue
            angle = math.radians(float(heading))
            
            for i, point in enumerate(grid_points):
                if is_point_in_rotated_rect(point, center, fov_width, fov_height, angle):
                    visit_count[i] += 1
                    last_visited_time[i] = current_time
        except (ValueError, TypeError, IndexError):
            continue

# Create figure and axis
fig, ax = plt.subplots(figsize=(12, 12))
ax.set_title("Coverage Visualization")
ax.set_xlabel("X Coordinate (m)")
ax.set_ylabel("Y Coordinate (m)")

# 경계선 그리기
boundary_x, boundary_y = zip(*boundary_points)
ax.plot(boundary_x, boundary_y, 'k-', linewidth=2)

# 그리드 포인트 초기화
grid_scatter = ax.scatter([], [], s=1, c='black', alpha=0.5)

# UAV와 UGV 위치 표시
uav_colors = ['magenta', 'green']
uav_scatters = []
uav_fov_patches = []
for i in range(CONFIG["num_uav"]):
    # UAV를 역삼각형으로 표시
    scatter = ax.scatter([], [], c=uav_colors[i], s=200, marker='v', label=f'UAV {i}')
    uav_scatters.append(scatter)
    
    # FOV 영역을 위한 패치 (초기 좌표 설정)
    initial_corners = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])  # 초기 사각형 좌표
    fov_patch = Polygon(initial_corners, alpha=0.2, color=uav_colors[i])
    ax.add_patch(fov_patch)
    uav_fov_patches.append(fov_patch)

# UGV를 파란색 사각형으로 표시
ugv_scatter = ax.scatter([], [], c='blue', s=200, marker='s', label='UGV')

# 범례 추가
ax.legend()

def init():
    """애니메이션 초기화"""
    empty_pos = np.array([[np.nan, np.nan]])
    grid_scatter.set_offsets(empty_pos)
    for scatter in uav_scatters:
        scatter.set_offsets(empty_pos)
    ugv_scatter.set_offsets(empty_pos)
    for patch in uav_fov_patches:
        patch.set_xy(np.array([[0, 0], [0, 0], [0, 0], [0, 0]]))  # 초기 FOV 좌표
    return [grid_scatter] + uav_scatters + [ugv_scatter] + uav_fov_patches

def update(frame):
    """프레임 업데이트"""
    try:
        with open('map/uav_simulation_data.json', 'r') as f:
            uav_data = json.load(f)
        with open('map/ugv_simulation_data.json', 'r') as f:
            ugv_data = json.load(f)
        
        current_time = frame * CONFIG["simulation_time_step"]
        uav_positions = []
        for i in range(CONFIG["num_uav"]):
            if frame < len(uav_data['uav_positions'][i]):
                pos = uav_data['uav_positions'][i][frame]
                if isinstance(pos, list) and len(pos) >= 3:
                    uav_positions.append(pos)
                else:
                    uav_positions.append(None)
            else:
                uav_positions.append(None)
        
        update_visit_grid(uav_positions, current_time)
        
        # 색상 매핑
        colors = []
        for i in range(len(grid_points)):
            if visit_count[i] == 0:
                colors.append('black')
            else:
                colors.append('yellow')  # 방문된 노드는 노란색으로 표시
        
        # 그리드 포인트 업데이트
        grid_scatter.set_offsets(grid_points)
        grid_scatter.set_color(colors)
        
        # UAV 위치와 FOV 업데이트
        for i, (scatter, patch) in enumerate(zip(uav_scatters, uav_fov_patches)):
            if frame < len(uav_data['uav_positions'][i]):
                pos = uav_data['uav_positions'][i][frame]
                if isinstance(pos, list) and len(pos) >= 3:
                    scatter.set_offsets([pos[:2]])
                    # FOV 영역 업데이트
                    heading = pos[2]
                    if heading is not None:  # heading이 None이 아닌 경우에만 FOV 업데이트
                        corners = get_rotated_rect_corners(pos[:2], fov_width, fov_height, 
                                                         math.radians(float(heading)))
                        patch.set_xy(corners)
                    else:
                        patch.set_xy(np.array([[0, 0], [0, 0], [0, 0], [0, 0]]))
                else:
                    scatter.set_offsets(np.array([[np.nan, np.nan]]))
                    patch.set_xy(np.array([[0, 0], [0, 0], [0, 0], [0, 0]]))
            else:
                scatter.set_offsets(np.array([[np.nan, np.nan]]))
                patch.set_xy(np.array([[0, 0], [0, 0], [0, 0], [0, 0]]))
        
        # UGV 위치 업데이트
        if frame < len(ugv_data['position']):
            ugv_scatter.set_offsets([ugv_data['position'][frame]])
        else:
            ugv_scatter.set_offsets(np.array([[np.nan, np.nan]]))
        
        # 제목 업데이트
        ax.set_title(f"Coverage Visualization (Time: {current_time:.1f}s)")
        
    except (FileNotFoundError, json.JSONDecodeError):
        pass
    
    return [grid_scatter] + uav_scatters + [ugv_scatter] + uav_fov_patches

# 애니메이션 생성 (60fps로 설정)
ani = FuncAnimation(fig, update, frames=None,
                   init_func=init, blit=True, interval=16.67,  # 1000ms/60fps ≈ 16.67ms
                   cache_frame_data=False)

# mp4로 저장 (60fps)
print("Saving animation to coverage_animation.mp4...")
ani.save('map/coverage_animation1.mp4', writer='ffmpeg', fps=120, dpi=300)
print("Animation saved successfully!")

plt.show() 