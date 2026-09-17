import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# CSV 파일 읽기
df = pd.read_csv('map/newtamu.csv')

plt.figure(figsize=(10, 10))
plt.title("Interpolated Map (0-2100m Scale)")
plt.xlabel("X Coordinate (m)")
plt.ylabel("Y Coordinate (m)")
plt.xlim(0, 4300)
plt.ylim(0, 4300)
colors = plt.cm.tab20.colors

for (feature_id, part_id), group in df.groupby(['feature_id', 'part_id']):
    x = group['x'].values
    y = group['y'].values

    # 누적 거리 계산 (각 점 사이의 거리를 더해서 매개변수 t 생성)
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    t = np.concatenate(([0], np.cumsum(distances)))
    
    # 보간할 점의 수를 정함 (예: 500개의 점)
    t_new = np.linspace(t[0], t[-1], 500)
    
    # x, y 각각에 대해 선형 보간
    x_new = np.interp(t_new, t, x)
    y_new = np.interp(t_new, t, y)
    
    plt.scatter(
        x_new, 
        y_new, 
        s=9,  # 점 크기; 필요에 따라 조절
        color=colors[feature_id % len(colors)],
        label=f'Feature {feature_id}'
    )

plt.grid(True, linestyle='--', alpha=0.5)
plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
plt.gca().set_aspect('equal')
plt.savefig('map_2100_scale.png', dpi=300, bbox_inches='tight')
plt.show()