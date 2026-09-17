import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation

def plot_ugv_path():
    # Load the CSV file
    df = pd.read_csv('map/tamu_map.csv')
    
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(12, 10))
    plt.title("UGV Path Progression in TAMU Map", fontsize=14, pad=20)
    plt.xlabel("X Coordinate (m)", fontsize=12)
    plt.ylabel("Y Coordinate (m)", fontsize=12)
    
    # Plot all points in black initially
    scatter = ax.scatter(df['x'], df['y'], color='black', s=10, alpha=0.5)
    
    # Function to update the plot for each frame
    def update(frame):
        # Create a color array where points up to the current frame are red
        colors = ['red' if i <= frame else 'black' for i in range(len(df))]
        scatter.set_color(colors)
        return scatter,
    
    # Create animation
    ani = FuncAnimation(fig, update, frames=len(df), interval=100, blit=True)
    
    # Save the animation
    ani.save('map/ugv_path_progression.gif', writer='pillow', fps=10)
    
    plt.grid(True, linestyle='--', alpha=0.3)
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    plot_ugv_path()
