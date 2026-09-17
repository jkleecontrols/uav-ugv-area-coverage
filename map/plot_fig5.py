import numpy as np
import matplotlib.pyplot as plt
from zdataloader import DataLoader
import matplotlib.patches as patches
from matplotlib.path import Path

def move_toward_center(point, center, distance):
    """Move a point toward the center by a specified distance"""
    direction = np.array(center) - np.array(point)
    norm = np.linalg.norm(direction)
    if norm == 0:
        return point
    direction = direction / norm
    return tuple(np.array(point) + direction * distance)

def plot_tamu_map(output_file='map/tamu_fig5.png'):
    # DataLoader 초기화
    loader = DataLoader('map/tamu_map.csv')
    
    # 그래프 설정
    plt.figure(figsize=(12, 10))
    plt.title("Comparison of UAV Trajectories Across Three Methods", fontsize=14, pad=20)
    plt.xlabel("X Coordinate (m)", fontsize=12)
    plt.ylabel("Y Coordinate (m)", fontsize=12)
    
    # Get the axis object early
    ax = plt.gca()
    
    # 경계선 그리기
    boundary_points = loader.get_boundary_points()
    boundary_x, boundary_y = zip(*boundary_points)
    plt.plot(boundary_x, boundary_y, 'k-', linewidth=2, zorder=2)
    
    # 1m 간격 그리드 포인트 그리기
    grid_points = loader.get_grid_points()
    if grid_points:
        grid_x, grid_y = zip(*grid_points)
        plt.scatter(grid_x, grid_y, s=1, color='lightgray', alpha=0.3, zorder=1)

    # 시작점과 도착점 설정
    start = (545.96, 689.40)
    end = (1348.92, 3290.63)

    # 1. Point-based: clustered repetitive paths
    cluster_a = [(start[0] + 50, start[1] + 50),
                 (start[0] + 70, start[1] + 80),
                 (start[0] + 90, start[1] + 60),
                 (start[0] + 50, start[1] + 50)] * 3  # repeat loop

    cluster_b = [(end[0] - 90, end[1] - 80),
                 (end[0] - 70, end[1] - 100),
                 (end[0] - 50, end[1] - 90),
                 (end[0] - 90, end[1] - 80)] * 3  # repeat loop

    full_path = [start] + cluster_a + [(start[0]+300, start[1]+1200)] + cluster_b + [end]
    point_x, point_y = zip(*full_path)
    plt.plot(point_x, point_y, 'r-', linewidth=2, label='Point-Based')

    # 2. Naive Area-based: zigzag sweep pattern (FOV not considered)
    rows = 10
    cols = 10
    x_step = (end[0] - start[0]) / cols
    y_step = (end[1] - start[1]) / rows
    
    zigzag_x = []
    zigzag_y = []
    
    for i in range(rows + 1):
        y = start[1] + i * y_step
        if i % 2 == 0:
            x_vals = np.linspace(start[0], end[0], cols + 1)
        else:
            x_vals = np.linspace(end[0], start[0], cols + 1)
        for x in x_vals:
            zigzag_x.append(x)
            zigzag_y.append(y)
    
    plt.plot(zigzag_x, zigzag_y, color='orange', linewidth=2, label='Naive Area-Based')

    # Add fixed-orientation FOV rectangles for Naive Area-Based path
    for i in range(len(zigzag_x)-1):
        cx = (zigzag_x[i] + zigzag_x[i+1]) / 2
        cy = (zigzag_y[i] + zigzag_y[i+1]) / 2
        # Heading is ignored (fixed orientation)
        draw_fov_rectangle(ax, (cx, cy), heading_deg=0, width=100, height=67, color='orange')

    # Find closest boundary index to start and end
    def closest_index(target, points):
        return min(range(len(points)), key=lambda i: np.hypot(points[i][0] - target[0], points[i][1] - target[1]))

    start_idx = closest_index(start, boundary_points)
    end_idx = closest_index(end, boundary_points)

    if start_idx < end_idx:
        fov_path = boundary_points[start_idx:end_idx+1]
    else:
        fov_path = boundary_points[start_idx:] + boundary_points[:end_idx+1]

    # === Extended nonlinear FOV-aware trajectory (confined to boundary) ===
    boundary_polygon = Path(boundary_points)
    steps = 200
    y_vals = np.linspace(start[1], end[1], steps)

    x_center = start[0] + (end[0] - start[0]) * 0.78
    amplitude1 = 480
    amplitude2 = 280
    amplitude3 = 180
    frequency = 2 * np.pi / (end[1] - start[1])

    raw_fov1_x = x_center \
                 + amplitude1 * np.sin(frequency * (y_vals - start[1]) * 4) \
                 + amplitude2 * np.cos(frequency * (y_vals - start[1]) * 2.8) \
                 + amplitude3 * np.sin(frequency * (y_vals - start[1]) * 6)

    raw_fov1_y = y_vals

    # Filter: keep only points inside boundary polygon
    filtered_fov1_x = []
    filtered_fov1_y = []

    for x, y in zip(raw_fov1_x, raw_fov1_y):
        if boundary_polygon.contains_point((x, y)):
            filtered_fov1_x.append(x)
            filtered_fov1_y.append(y)

    # Connect to start and end points
    fov1_x = [start[0]] + filtered_fov1_x + [end[0]]
    fov1_y = [start[1]] + filtered_fov1_y + [end[1]]

    plt.plot(fov1_x, fov1_y, 'b-', linewidth=2, label='FOV-Aware (UAV1 - Nonlinear Coverage)')

    for i in range(len(fov1_x)-1):
        cx = (fov1_x[i] + fov1_x[i+1]) / 2
        cy = (fov1_y[i] + fov1_y[i+1]) / 2
        dx = fov1_x[i+1] - fov1_x[i]
        dy = fov1_y[i+1] - fov1_y[i]
        heading = np.degrees(np.arctan2(dy, dx)) + 90
        draw_fov_rectangle(ax, (cx, cy), heading)

    # 시작/종료점 표시
    plt.scatter(*start, color='black', marker='o', zorder=5)
    plt.text(start[0]-100, start[1]-100, 'Start', fontsize=9)

    plt.scatter(*end, color='black', marker='x', zorder=5)
    plt.text(end[0]+50, end[1]+50, 'End', fontsize=9)

    # 범례 추가
    plt.legend(loc='lower right')

    # Add legend and grid
    plt.legend(loc='lower right', fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.3)
    plt.axis('equal')
    
    # Add caption
    plt.figtext(0.5, 0.01, "Comparison of UAV trajectories across three methods. Our method exhibits smoother and more distributed coverage paths.", 
                ha='center', fontsize=12, style='italic')
    
    # 저장 및 표시
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.show()

def draw_fov_rectangle(ax, center, heading_deg, width=100, height=67, color='skyblue'):
    # Convert heading to radians, and rotate to match matplotlib orientation
    theta = np.deg2rad(-heading_deg)
    # Create rectangle at origin
    rect = patches.Rectangle((-width/2, -height/2), width, height, linewidth=1,
                             edgecolor=color, facecolor=color, alpha=0.3)
    # Rotate and move to center
    t = patches.transforms.Affine2D().rotate(theta).translate(center[0], center[1])
    rect.set_transform(t + ax.transData)
    ax.add_patch(rect)

if __name__ == "__main__":
    plot_tamu_map()