import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib as mpl
import matplotlib.patches as patches

# CSV 데이터 로드
df = pd.read_csv('interpolated_map.csv')

# UGV 경로 데이터 (feature_id==0, part_id==0)
df_ugv = df[(df['feature_id'] == 0) & (df['part_id'] == 0)].sort_values('coord_index')
polygon_points = list(zip(df_ugv['x'], df_ugv['y']))
start_point = np.array(polygon_points[0])

# UAV 매핑 영역 설정 (100x67m 직사각형)
rect_width, rect_height = 100, 67
rect_x_min, rect_y_min = start_point[0], start_point[1]
rect_x_max, rect_y_max = rect_x_min + rect_width, rect_y_min + rect_height

# 1m 격자 생성
x_grid = np.arange(rect_x_min, rect_x_max + 1, 1)
y_grid = np.arange(rect_y_min, rect_y_max + 1, 1)
X, Y = np.meshgrid(x_grid, y_grid)
heatmap = np.zeros_like(X, dtype=float)

# 폴리곤 내부 마스크 생성
polygon_path = Path(polygon_points)
grid_points = np.vstack((X.ravel(), Y.ravel())).T
inside_mask = polygon_path.contains_points(grid_points).reshape(X.shape)

# 시뮬레이션 파라미터 (5초 설정)
T = 5.0  # 총 시뮬레이션 시간 5초
num_samples = 100  # 샘플 수 증가 (부드러운 궤적)

# UGV 이동 (3m/s)
ugv_speed = 3.0
distances = np.sqrt(np.diff(df_ugv['x'])**2 + np.diff(df_ugv['y'])**2)
cum_dist = np.concatenate(([0], np.cumsum(distances)))
max_ugv_dist = min(ugv_speed * T, cum_dist[-1])
ugv_sample_dists = np.linspace(0, max_ugv_dist, num_samples)
ugv_x = np.interp(ugv_sample_dists, cum_dist, df_ugv['x'])
ugv_y = np.interp(ugv_sample_dists, cum_dist, df_ugv['y'])
ugv_traj = np.column_stack((ugv_x, ugv_y))
ugv_times = ugv_sample_dists / ugv_speed

# UAV 3대 경로 생성 (12m/s)
def generate_uav_path(start, pattern, speed, t_max):
    t = np.linspace(0, t_max, num_samples)
    if pattern == 'horizontal':
        x = start[0] + np.clip(speed * t * np.cos(np.deg2rad(0)), 0, 100)
        y = start[1] + np.zeros_like(t)
    elif pattern == 'vertical':
        x = start[0] + np.zeros_like(t)
        y = start[1] + np.clip(speed * t * np.sin(np.deg2rad(90)), 0, 67)
    elif pattern == 'diagonal':
        x = start[0] + np.clip(speed * t * np.cos(np.deg2rad(45)), 0, 100)
        y = start[1] + np.clip(speed * t * np.sin(np.deg2rad(45)), 0, 67)
    return np.column_stack((x, y)), t

uav1_traj, uav1_t = generate_uav_path(start_point, 'horizontal', 12, T)
uav2_traj, uav2_t = generate_uav_path(start_point, 'vertical', 12, T)
uav3_traj, uav3_t = generate_uav_path(start_point, 'diagonal', 12, T)

# 히트맵 계산 (최근 위치 가중치 강조)
sigma = 2.0  # Gaussian 반경

def add_heat(traj, times):
    global heatmap
    for pos, t in zip(traj, times):
        weight = (T - t)/T  # 시간에 따른 가중치 (최근 높음)
        heatmap += weight * np.exp(-((X - pos[0])**2 + (Y - pos[1])**2)/(2*sigma**2))

add_heat(ugv_traj, ugv_times)
add_heat(uav1_traj, uav1_t)
add_heat(uav2_traj, uav2_t)
add_heat(uav3_traj, uav3_t)

# 영역 외부 마스킹
heatmap_masked = np.ma.array(heatmap, mask=~inside_mask)

# 히트맵 시각화
plt.figure(figsize=(10, 8))
cmap = mpl.cm.get_cmap('hot').copy()
cmap.set_bad(color='black')  # 외부 영역 검정색

plt.imshow(heatmap_masked, origin='lower', 
           extent=[rect_x_min, rect_x_max, rect_y_min, rect_y_max],
           cmap=cmap, aspect='auto', vmin=0, vmax=1)
plt.colorbar(label='Recent Activity Intensity')
plt.scatter(start_point[0], start_point[1], c='cyan', s=100, marker='*', edgecolor='white', label='Start Point')

# UAV 경로 표시
for traj, color in zip([uav1_traj, uav2_traj, uav3_traj], ['lime', 'magenta', 'yellow']):
    plt.plot(traj[:,0], traj[:,1], color=color, alpha=0.4, lw=1)

plt.title(f'Heatmap after {T} seconds')
plt.xlabel('X Coordinate (m)')
plt.ylabel('Y Coordinate (m)')
plt.legend()
plt.xlim(rect_x_min-10, rect_x_max+10)
plt.ylim(rect_y_min-10, rect_y_max+10)
plt.grid(True, alpha=0.3)
plt.show()