import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import json
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
import matplotlib.cm as cm
import time
from zconfig import CONFIG
from zdataloader import DataLoader

# DataLoader 초기화
loader = DataLoader('map/tamu_map.csv')
grid_points = loader.get_grid_points()
boundary_points = loader.get_boundary_points()

# Create figure and axis
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_title("Texas A&M University Map (1m interval grid)")
ax.set_xlabel("X Coordinate (m)")
ax.set_ylabel("Y Coordinate (m)")

# 경계선 그리기
boundary_x, boundary_y = zip(*boundary_points)
ax.plot(boundary_x, boundary_y, 'k-', linewidth=2, zorder=2)

# 1m 간격 그리드 포인트 그리기
if grid_points:
    grid_x, grid_y = zip(*grid_points)
    ax.scatter(grid_x, grid_y, s=1, color='lightgray', alpha=0.5, zorder=1)

# Initialize empty lines for trajectories
ugv_line, = ax.plot([], [], 'b-', label='UGV Path', linewidth=2, zorder=3)
uav_lines = []
uav_points = []
colors = ['r', 'g']
for i in range(CONFIG["num_uav"]):
    line, = ax.plot([], [], f'{colors[i]}-', label=f'UAV {i} Path', linewidth=2, zorder=3)
    point = ax.scatter([], [], c=colors[i], s=100, label=f'UAV {i}', zorder=4)
    uav_lines.append(line)
    uav_points.append(point)

# Initialize scatter points for current positions
ugv_point = ax.scatter([], [], c='blue', s=100, label='UGV', zorder=4)

# Initialize reward heatmap
x_grid = np.linspace(0, 4500, 100)
y_grid = np.linspace(0, 4500, 100)
reward_grid = np.zeros((len(y_grid), len(x_grid)))
heatmap = ax.imshow(reward_grid, extent=[0, 4500, 0, 4500], 
                   origin='lower', cmap='hot', alpha=0.3, zorder=1)
plt.colorbar(heatmap, ax=ax, label='Reward Value')

# Initialize empty lists to store trajectories
ugv_trajectory = []
uav_trajectories = [[] for _ in range(CONFIG["num_uav"])]
uav_states = [None for _ in range(CONFIG["num_uav"])]

# Add legend and grid
ax.legend()
ax.grid(True, linestyle='--', alpha=0.5)
ax.axis('equal')

def init():
    ugv_line.set_data([], [])
    for line in uav_lines:
        line.set_data([], [])
    # Initialize with empty 2D array
    empty_pos = np.array([[np.nan, np.nan]])
    ugv_point.set_offsets(empty_pos)
    for point in uav_points:
        point.set_offsets(empty_pos)
    heatmap.set_data(np.zeros((len(y_grid), len(x_grid))))
    return [ugv_line] + uav_lines + [ugv_point] + uav_points + [heatmap]

def update(frame):
    try:
        # Load latest simulation data
        with open('map/ugv_simulation_data1.json', 'r') as f:
            ugv_data = json.load(f)
        with open('map/uav_simulation_data1.json', 'r') as f:
            uav_data = json.load(f)
        with open('map/zz_result.json', 'r') as f:
            result_data = json.load(f)
        
        # Update UGV trajectory
        if frame < len(ugv_data['position']):
            ugv_pos = ugv_data['position'][frame]
            ugv_trajectory.append(ugv_pos)
            ugv_traj_array = np.array(ugv_trajectory)
            ugv_line.set_data(ugv_traj_array[:, 0], ugv_traj_array[:, 1])
            ugv_point.set_offsets([ugv_pos])
        
        # Update UAV trajectories and states
        for i in range(CONFIG["num_uav"]):
            if frame < len(uav_data['uav_positions'][i]):
                uav_pos = uav_data['uav_positions'][i][frame]
                uav_trajectories[i].append(uav_pos)
                uav_traj_array = np.array(uav_trajectories[i])
                uav_lines[i].set_data(uav_traj_array[:, 0], uav_traj_array[:, 1])
                uav_points[i].set_offsets([uav_pos])
                
                # Update UAV state
                if frame < len(uav_data['uav_states'][i]):
                    uav_states[i] = uav_data['uav_states'][i][frame]
        
        # Update reward heatmap
        if 'reward_history' in result_data:
            current_rewards = result_data['reward_history'][-1]
            reward_grid.fill(0)  # Reset grid
            
            for point, value in zip(current_rewards['points'], current_rewards['values']):
                x_idx = np.argmin(np.abs(x_grid - point[0]))
                y_idx = np.argmin(np.abs(y_grid - point[1]))
                reward_grid[y_idx, x_idx] = value
            
            heatmap.set_data(reward_grid)
            heatmap.set_clim(vmin=reward_grid.min(), vmax=reward_grid.max())
        
        # Update title with current time and UAV states
        current_time = frame * CONFIG["simulation_time_step"]
        state_text = " | ".join([f"UAV {i}: {uav_states[i]}" for i in range(CONFIG["num_uav"])])
        ax.set_title(f'Texas A&M University Map (Time: {current_time:.1f}s)\n{state_text}')
        
    except (FileNotFoundError, json.JSONDecodeError):
        # Handle case when files are not yet available or being written
        pass
    
    return [ugv_line] + uav_lines + [ugv_point] + uav_points + [heatmap]

# Create animation
ani = FuncAnimation(fig, update, frames=None,
                   init_func=init, blit=True, interval=100)

plt.show() 