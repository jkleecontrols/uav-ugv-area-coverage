import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import json
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
import matplotlib.cm as cm
import time
from config_settings import CONFIG

# Load simulation data
with open('ugv_simulation_data.json', 'r') as f:
    ugv_data = json.load(f)

with open('uav_simulation_data.json', 'r') as f:
    uav_data = json.load(f)

with open('top_final_results.json', 'r') as f:
    reward_data = json.load(f)

# Extract data
times = np.array(ugv_data['time'])
ugv_positions = np.array(ugv_data['ugv_positions'])

# Process UAV data
uav_positions = []
for uav_idx in range(CONFIG["num_uav"]):
    uav_path = []
    for i in range(len(uav_data['uav_start_points'][uav_idx])):
        start = uav_data['uav_start_points'][uav_idx][i]
        end = uav_data['uav_end_points'][uav_idx][i]
        if end is not None:  # Skip if end point is None
            uav_path.append(start)
            uav_path.append(end)
    uav_positions.append(uav_path)

reward_history = reward_data.get('reward_history', [])

# Get speed settings from config
uav_speed = CONFIG["uav_flight_speed"]  # m/s
ugv_drive_speed = CONFIG["ugv_drive_speed"]  # m/s
ugv_takeoffland_speed = CONFIG["ugv_takeoffland_speed"]  # m/s
time_step = 1  # 1초 단위로 애니메이션

# Create figure and axis
fig, ax = plt.subplots(figsize=(12, 12))
ax.set_xlim(0, 20000)
ax.set_ylim(0, 20000)
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_title('UGV and UAV Trajectories with Reward Heatmap')

# Initialize empty lines for trajectories
ugv_line, = ax.plot([], [], 'b-', label='UGV Path')
uav_lines = []
uav_points = []
colors = ['r', 'g', 'm']
for i in range(CONFIG["num_uav"]):
    line, = ax.plot([], [], f'{colors[i]}-', label=f'UAV {i+1} Path')
    point = ax.scatter([], [], c=colors[i], s=100, label=f'UAV {i+1}')
    uav_lines.append(line)
    uav_points.append(point)

# Initialize scatter points for current positions
ugv_point = ax.scatter([], [], c='blue', s=100, label='UGV')

# Initialize reward heatmap
x_grid = np.linspace(0, 20000, 100)
y_grid = np.linspace(0, 20000, 100)
reward_grid = np.zeros((len(y_grid), len(x_grid)))
heatmap = ax.imshow(reward_grid, extent=[0, 20000, 0, 20000], 
                   origin='lower', cmap='hot', alpha=0.5)
plt.colorbar(heatmap, ax=ax, label='Reward Value')

# Initialize empty lists to store trajectories
ugv_trajectory = []
uav_trajectories = [[] for _ in range(CONFIG["num_uav"])]

# Add legend
ax.legend()

def interpolate_path(path, speed, time_step):
    interpolated = []
    for i in range(len(path)-1):
        start = path[i]
        end = path[i+1]
        distance = np.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)
        steps = int(distance / (speed * time_step))
        for j in range(steps):
            ratio = j/steps
            x = start[0] + (end[0]-start[0]) * ratio
            y = start[1] + (end[1]-start[1]) * ratio
            interpolated.append((x, y))
    return interpolated

# Interpolate paths based on speeds
ugv_interpolated = interpolate_path(ugv_positions, ugv_drive_speed, time_step)
uav_interpolated = [interpolate_path(path, uav_speed, time_step) for path in uav_positions]

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
    # Update trajectories
    if frame < len(ugv_interpolated):
        ugv_trajectory.append(ugv_interpolated[frame])
    
    for i in range(CONFIG["num_uav"]):
        if frame < len(uav_interpolated[i]):
            uav_trajectories[i].append(uav_interpolated[i][frame])
    
    # Convert trajectories to numpy arrays for plotting
    if ugv_trajectory:
        ugv_traj_array = np.array(ugv_trajectory)
        ugv_line.set_data(ugv_traj_array[:, 0], ugv_traj_array[:, 1])
    
    # Update lines
    for i in range(CONFIG["num_uav"]):
        if uav_trajectories[i]:
            uav_traj_array = np.array(uav_trajectories[i])
            uav_lines[i].set_data(uav_traj_array[:, 0], uav_traj_array[:, 1])
    
    # Update current positions
    if frame < len(ugv_interpolated):
        ugv_point.set_offsets([ugv_interpolated[frame]])
    for i in range(CONFIG["num_uav"]):
        if frame < len(uav_interpolated[i]):
            uav_points[i].set_offsets([uav_interpolated[i][frame]])
    
    # Update reward heatmap
    if frame < len(reward_history):
        current_rewards = reward_history[frame]
        reward_grid.fill(0)  # Reset grid
        
        for point, value in zip(current_rewards['points'], current_rewards['values']):
            x_idx = np.argmin(np.abs(x_grid - point[0]))
            y_idx = np.argmin(np.abs(y_grid - point[1]))
            reward_grid[y_idx, x_idx] = value
        
        heatmap.set_data(reward_grid)
        heatmap.set_clim(vmin=reward_grid.min(), vmax=reward_grid.max())
    
    # Update title with current time
    current_time = frame * time_step
    ax.set_title(f'UGV and UAV Trajectories with Reward Heatmap (Time: {current_time:.1f}s)')
    
    return [ugv_line] + uav_lines + [ugv_point] + uav_points + [heatmap]

# Create animation
max_frames = max(len(ugv_interpolated), *[len(path) for path in uav_interpolated])
ani = FuncAnimation(fig, update, frames=max_frames,
                    init_func=init, blit=True, interval=50)

plt.show() 