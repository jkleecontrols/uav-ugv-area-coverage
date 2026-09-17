from data_loader import DataLoader
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import patches

class StepZero:
    def __init__(self, env, start, end, config, csv_path):
        self.env = env
        self.start = start
        self.end = end
        self.config = config
        self.csv_path = csv_path
        self.data = None
        self.x_data = None
        self.y_data = None
        self.i_cutoff = None
        self.s_point = None
        self.e_point = None
        self.center = None
        self.angle_deg = None
        self.reward_nodes = {}
        
        # Get UAV parameters from config
        self.drone_speed = config["uav_flight_speed"]  # m/s
        self.drone_time_limit = config["uav_flight_time"]  # seconds

    def load_data(self):
        loader = DataLoader(self.csv_path)
        self.data = loader.get_coordinates()
        self.x_data = 1000 * np.array([pt[0] for pt in self.data])  # km to m
        self.y_data = 1000 * np.array([pt[1] for pt in self.data])  # km to m

    def euclidean_distance(self, x1, y1, x2, y2):
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def determine_cutoff_index(self):
        total_distance = 0
        # Find the index of the start point in the coordinates
        start_idx = 0
        min_dist = float('inf')
        for i, (x, y) in enumerate(zip(self.x_data, self.y_data)):
            dist = self.euclidean_distance(x, y, self.start[0], self.start[1])
            if dist < min_dist:
                min_dist = dist
                start_idx = i
        
        # Calculate cutoff based on UAV flight time
        for i in range(start_idx, len(self.x_data)-1):
            length = self.euclidean_distance(self.x_data[i+1], self.y_data[i+1],
                                             self.x_data[i], self.y_data[i])
            total_distance += length
            if total_distance >= self.drone_speed * self.drone_time_limit:
                self.i_cutoff = i
                return

    def calculate_geometry(self):
        s_point = (self.x_data[self.start_index], self.y_data[self.start_index])
        e_point = (self.x_data[self.start_index + self.i_cutoff - 7],
                   self.y_data[self.start_index + self.i_cutoff - 7])
        self.s_point = s_point
        self.e_point = e_point
        mid_x = (s_point[0] + e_point[0]) / 2
        mid_y = (s_point[1] + e_point[1]) / 2
        self.center = (mid_x, mid_y)
        angle_rad = math.atan2((s_point[1] - e_point[1]), (s_point[0] - e_point[0]))
        self.angle_deg = angle_rad * (180 / math.pi)

    def populate_reward_nodes(self):
        for p in zip(self.x_data[0:3], self.y_data[0:3]):
            self.reward_nodes[p] = 10
        for p in zip(self.x_data[3:7], self.y_data[3:7]):
            self.reward_nodes[p] = 1000

    def draw_figure(self):
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.set_title('First Iteration', fontsize=16)
        ax.set_xlim(0, 20000)
        ax.set_ylim(0, 20000)

        ax.scatter(self.x_data[7:], self.y_data[7:], s=30, facecolors='none', edgecolors='black')
        ax.scatter(self.x_data[0:3], self.y_data[0:3], c='red', marker='D')
        ax.scatter(self.x_data[3:7], self.y_data[3:7], c='blue', marker='^')
        ax.scatter(*self.s_point, s=150, c='r', marker='*', label='Starting point on 1st iteration')
        ax.scatter(*self.e_point, s=150, c='y', marker='*', label='Ending point on 1st iteration')

        ellipse = patches.Ellipse(self.center, 19200, 18425, self.angle_deg,
                                  fc='none', linestyle='solid', ec='g', lw=3)
        ax.add_patch(ellipse)

        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.show()

    def run(self):
        self.load_data()
        self.determine_cutoff_index()
        self.calculate_geometry()
        self.populate_reward_nodes()
        self.draw_figure()
        self.env.set_vertices(...)  # csv 로딩 or 전처리
        return {
            "start_point": self.start,
            "end_point": self.end,
            "reward_nodes": self.reward_nodes,
            "cutoff_index": self.i_cutoff
        }