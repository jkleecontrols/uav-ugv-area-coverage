import pandas as pd
import matplotlib.pyplot as plt

# CSV 파일 읽기
df = pd.read_csv('interpolated_map.csv')

# 플롯 설정
plt.figure(figsize=(10, 10))  # 정사각형 비율 유지
plt.title("Interpolated Map (0-2100m Scale)")
plt.xlabel("X Coordinate (m)")
plt.ylabel("Y Coordinate (m)")

# 축 범위 고정
plt.xlim(0, 2100)
plt.ylim(0, 2100)

# feature_id별 색상 지정
colors = plt.cm.tab20.colors

# 각 feature_id 그룹별로 플로팅
for (feature_id, part_id), group in df.groupby(['feature_id', 'part_id']):
    plt.plot(
        group['x'], 
        group['y'], 
        marker='o', 
        markersize=3,
        linestyle='-', 
        linewidth=1,
        color=colors[feature_id % len(colors)],
        label=f'Feature {feature_id}'
    )

# 보조 설정
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
plt.gca().set_aspect('equal')  # 축 비율 동일하게 유지

# 저장 및 출력
plt.savefig('map_2100_scale.png', dpi=300, bbox_inches='tight')
plt.show()


