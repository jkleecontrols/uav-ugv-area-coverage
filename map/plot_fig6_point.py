import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import seaborn as sns

# 가상의 AOI 영역 (예: 100x100 grid)
grid_size = (100, 100)
coverage_matrix = np.zeros(grid_size)

# 예시로 특정 영역에만 커버리지를 준다 (Point-Based)
# Point-based는 일부 영역만 진하게 칠함
coverage_matrix[30:40, 60:70] += 5
coverage_matrix[32:36, 62:68] += 3

# 색상 정의: rocket palette
cmap = sns.color_palette("rocket", as_cmap=True)

# 플롯
plt.figure(figsize=(8, 8))
sns.heatmap(
    coverage_matrix,
    cmap=cmap,
    cbar=True,
    square=True,
    xticklabels=False,
    yticklabels=False,
    linewidths=0.0,
    linecolor='gray'
)
plt.title("Coverage Heatmap - Point-Based Example")
plt.xlabel("X Grid Index")
plt.ylabel("Y Grid Index")
plt.tight_layout()
plt.show()