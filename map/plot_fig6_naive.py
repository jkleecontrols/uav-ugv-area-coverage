import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# Create base coverage matrix
grid_size = (100, 100)
common_extent = [0, grid_size[1], 0, grid_size[0]]  # For consistent axis ranges

# Point-Based: compact target zone
point_matrix = np.zeros(grid_size)
point_matrix[49:51, 49:51] += 10

# Naive Area-Based: vertical overlapping stripes
coverage_matrix = np.zeros(grid_size)
for i in range(10, 90, 8):
    coverage_matrix[i:i+4, 20:80] += 1

# FOV-Aware: diagonally distributed coverage with broader spread
fov_matrix = np.zeros(grid_size)
for i in range(10, 90, 6):
    for j in range(10, 90, 6):
        fov_matrix[i:i+3, j:j+2] += 1

def summarize_coverage(matrix, name):
    covered_cells = np.sum(matrix > 0)
    total_cells = matrix.size
    avg_coverage = np.mean(matrix[matrix > 0]) if covered_cells > 0 else 0
    redundancy_ratio = np.sum(matrix > 1) / covered_cells if covered_cells > 0 else 0
    print(f"{name} — Covered: {covered_cells}, Avg Intensity: {avg_coverage:.2f}, Redundant: {redundancy_ratio*100:.1f}%")

summarize_coverage(point_matrix, "Point-Based")
summarize_coverage(coverage_matrix, "Naive Area-Based")
summarize_coverage(fov_matrix, "FOV-Aware")

# Define colormap similar to the provided example
cmap = sns.color_palette("rocket", as_cmap=True)

# Create subplots for side-by-side comparison
fig, axes = plt.subplots(1, 3, figsize=(18, 6), gridspec_kw={'width_ratios': [1, 1, 1]})

# Plot Point-Based
sns.heatmap(
    point_matrix,
    cmap=cmap,
    ax=axes[0],
    cbar=False,
    square=True,
    vmin=0, vmax=10,  # Normalize across all three
    xticklabels=False,
    yticklabels=False,
    linewidths=0.0,
    linecolor='gray'
)
axes[0].set_title("Point-Based")

# Plot Naive Area-Based
sns.heatmap(
    coverage_matrix,
    cmap=cmap,
    ax=axes[1],
    cbar=False,
    square=True,
    vmin=0, vmax=10,  # Normalize across all three
    xticklabels=False,
    yticklabels=False,
    linewidths=0.0,
    linecolor='gray'
)
axes[1].set_title("Naive Area-Based")

# Plot FOV-Aware
sns.heatmap(
    fov_matrix,
    cmap=cmap,
    ax=axes[2],
    cbar=True,
    square=True,
    vmin=0, vmax=10,  # Normalize across all three
    xticklabels=False,
    yticklabels=False,
    linewidths=0.0,
    linecolor='gray'
)
axes[2].set_title("FOV-Aware")

for ax in axes:
    ax.set_xlim(common_extent[0], common_extent[1])
    ax.set_ylim(common_extent[2], common_extent[3])
    ax.set_aspect('equal')

# Manually adjust position of each axis to match size
for ax in axes:
    pos = ax.get_position()
    ax.set_position([pos.x0, pos.y0, 0.25, pos.height])

plt.suptitle("Coverage Heatmap Comparison", fontsize=16)
# plt.tight_layout()
plt.savefig("figs/coverage_heatmap_comparison.png", dpi=300)
plt.show()
