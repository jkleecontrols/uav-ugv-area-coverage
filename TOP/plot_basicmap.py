import pandas as pd
import matplotlib.pyplot as plt

# CSV 파일 읽기
df = pd.read_csv('environments/corridor_scene/corridor_scene.csv')

# km를 m로 변환
df['X (m)'] = df['X (km)'] * 1000
df['Y (m)'] = df['Y (km)'] * 1000

# 그래프 설정
plt.figure(figsize=(8, 8))

# 이름 없는 좌표점들 그리기 (검은 점)
unnamed_points = df[df['Name'].isna()]
plt.scatter(unnamed_points['X (m)'], unnamed_points['Y (m)'], c='black', s=20, label='Route Points')

# Depot 포인트 그리기
depots = df[df['Name'].str.contains('Depot', na=False)]
plt.scatter(depots['X (m)'], depots['Y (m)'], c='red', s=100, label='Depots')

# AOI 포인트 그리기
aois = df[df['Name'].str.contains('AOI', na=False)]
plt.scatter(aois['X (m)'], aois['Y (m)'], c='blue', s=100, label='AOIs')

# # Route 경로 그리기 (Route 포인트와 이름 없는 포인트들을 모두 포함)
# route_points = df[df['Name'].isna() | (df['Name'] == 'Route')]
# plt.plot(route_points['X (m)'], route_points['Y (m)'], 'g-', label='Route', alpha=0.5)

# 그래프 설정
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Corridor Scene Map')
plt.grid(True)
plt.legend()

# 그래프 저장
# plt.savefig('corridor_scene_map.png')
plt.show()
