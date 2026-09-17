import numpy as np
import matplotlib.pyplot as plt
from zdataloader import DataLoader

def plot_tamu_map(output_file='map/tamu_innerdot_map.png'):
    # DataLoader 초기화
    loader = DataLoader('map/tamu_map.csv')
    
    # 그래프 설정
    plt.figure(figsize=(10, 10))
    plt.title("Texas A&M University Map (1m interval grid)")
    plt.xlabel("X Coordinate (m)")
    plt.ylabel("Y Coordinate (m)")
    
    # 경계선 그리기
    boundary_points = loader.get_boundary_points()
    boundary_x, boundary_y = zip(*boundary_points)
    plt.plot(boundary_x, boundary_y, 'k-', linewidth=2, zorder=2)
    
    # 1m 간격 그리드 포인트 그리기
    grid_points = loader.get_grid_points()
    if grid_points:
        grid_x, grid_y = zip(*grid_points)
        plt.scatter(grid_x, grid_y, s=1, color='lightgray', alpha=0.5, zorder=1)
    
    # 그래프 설정 마무리
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.axis('equal')
    
    # 저장 및 표시
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    plot_tamu_map()