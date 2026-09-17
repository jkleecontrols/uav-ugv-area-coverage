import pandas as pd
import matplotlib.pyplot as plt

# 데이터 불러오기
interpolated_df = pd.read_csv("interpolated_map.csv")
branch_df = pd.read_csv("detected_branch_points.csv")

# 그림 설정
plt.figure(figsize=(7, 7))
plt.title("Map with Interpolated Paths and Branch Points", fontsize=14)
plt.xlabel("X Coordinate (m)", fontsize=12)
plt.ylabel("Y Coordinate (m)", fontsize=12)
plt.grid(True, linestyle="--", alpha=0.5)

# 1. interpolated_map 시각화 (선+점)
for (feature_id, part_id), group in interpolated_df.groupby(["feature_id", "part_id"]):
    plt.plot(
        group["x"],
        group["y"],
        marker="o",
        markersize=3,
        linestyle="-",
        linewidth=1,
        alpha=0.7,
        zorder=1,  # 하단 레이어
    )

# 2. branch_points 시각화 (강조 표시)
plt.scatter(
    branch_df["x"],
    branch_df["y"],
    s=150,  # 마커 크기
    c="gold",
    edgecolors="red",
    linewidths=1.5,
    marker="*",
    label="Branch Points",
    zorder=2,  # 상단 레이어
)

# 축 범위 설정
plt.xlim(0, 2100)
plt.ylim(0, 2100)
plt.legend(loc="upper right")
plt.savefig("map_with_layers.png", dpi=300, bbox_inches="tight")
plt.show()