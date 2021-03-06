import logging
import logger as lg
import math
import numpy as np
import scipy.io
import scipy.stats
import matplotlib.pyplot as plt
import heapq
import pickle
from scipy.ndimage.morphology import binary_dilation


class Mapper:
    """
    Implements environment mapping
    """
    def __init__(self, robot, handle=None):
        lg.message(logging.INFO, 'Initialising ' + self.__class__.__name__)
        self.robot = robot
        self.robot_handle = handle

        # Defines grid size mapping the environment
        self.grid_res = 1
        self.x_size = int(1500 * self.grid_res)
        self.y_size = int(1500 * self.grid_res)
        self.map_grid = np.zeros((self.x_size, self.y_size))
        self.map_grid_binary = None

        self.alpha = 0.2  # Default obstacle thickness
        self.beta = 5.0 * np.pi / 180.0  # Sensor beam aperture

        self.r_to_c_angle_grid = None  # Grid of robot to cell angles
        self.r_to_c_distance_grid = None  # Grid to robot to cell distances

        self.sensor_distance = 0
        self.sensor_angle = 0

        self.grid_coords_3d = np.array([np.tile(np.arange(0, self.x_size, 1)[:, None], (1, self.y_size)),
                                        np.tile(np.arange(0, self.y_size, 1)[:, None].T, (self.x_size, 1))])

        self.bearing = 0
        self.pose = []

        # Probabilities for free and occupied space to increment/decrement a environment map grid cell by
        self.occupied = 0.367
        self.free = -0.693
        self.grid_to_pose = None

        # Grid refs for beacon and HP centre location
        self.beacon_loc = (250, 430)
        self.hp_centre_loc = (750, 750)

        self.planned_route = []

    @staticmethod
    def astar_gscore_heuristic(a, b):
        """
        Heuristic element of A* that adds Euclidean distance to target destination cost to cell
        """
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def update_map(self):
        """
        Note the OG mapping here is a refactored version of the original source code found here, by user superjax
        superjax/occupancy_grid_mapping_example.py on gist.github.com
        https://gist.github.com/superjax/33151f018407244cb61402e094099c1d

        """
        self.world_to_grid_bearing()
        if not self.world_to_grid_pose():
            return
        self.grid_to_pose = self.grid_coords_3d.copy()
        self.grid_to_pose[0, :, :] -= self.pose[0]  # x coordinates to pose
        self.grid_to_pose[1, :, :] -= self.pose[1]  # y coordinates to pose
        self.r_to_c_angle_grid = self.robot_to_cell_angle_grid()
        self.r_to_c_distance_grid = self.robot_to_cell_distance_grid()

        for i, sen in enumerate(self.robot.state['int']['prox_s'].last_read):
            if i > 7:  # Only process the front ultrasonic sensor array for performance reasons
                continue
            if sen[1] is False:  # Throw away abnormal readings not detecting explicit obstacle distance
                continue

            self.sensor_distance = sen[0]

            # Transform snesor distance to cm units as per grid
            self.sensor_distance *= (100 * self.grid_res)
            self.sensor_angle = self.robot.state['int']['prox_s'].sensor_angle[i]

            # Build free mask of empty space in front of sensor
            fm = self.free_mask()

            # Build occupied mask of obstacles detected at sensor distance
            om = self.occupied_mask()

            # Use masks to project probabilities onto OG
            self.map_grid[om] += self.occupied
            self.map_grid[fm] += self.free

    def world_to_grid_bearing(self):
        """
        Transform VREP robot Euler bearing orientation to grid
        """
        self.bearing = self.robot.state['int']['compass'].last_read_euler
        if self.bearing > 0:
            self.bearing -= math.pi
        else:
            self.bearing += math.pi

    def world_to_grid_pose(self):
        """
        Transform VREP absolute location system to grid grid reference system
        """
        self.pose = self.robot.state['ext']['abs_pos_n'].copy()
        if len(set(self.pose)) == 1:
            return False
        self.pose[0] = int((self.x_size / 2) + int(self.pose[0] * -100) * self.grid_res)
        self.pose[1] = int((self.y_size / 2) + int(self.pose[1] * -100) * self.grid_res)
        return True

    def robot_to_cell_angle_grid(self):
        """
        Calculate angles from robot to all OG cells
        """
        return np.arctan2(self.grid_to_pose[1, :, :], self.grid_to_pose[0, :, :]) - self.bearing

    def robot_to_cell_distance_grid(self):
        """
        Calculate distances from robot to all OG cells
        """
        return scipy.linalg.norm(self.grid_to_pose, axis=0)

    def free_mask(self):
        """
        Assemble free mask of empty space in front of sensor, taking into account mounted sensor angle, aperture and
        sensor distance
        """
        return (np.abs(self.r_to_c_angle_grid - self.sensor_angle) <= self.beta / 2.0) & \
               (self.r_to_c_distance_grid < (self.sensor_distance - self.alpha / 2.0))

    def occupied_mask(self):
        """
        Assemble occupied mask of detected obstacles detected at returned ping range
        """
        return (np.abs(self.r_to_c_angle_grid - self.sensor_angle) <= self.beta / 2.0) & \
               (np.abs(self.r_to_c_distance_grid - self.sensor_distance) <= self.alpha / 2.0)

    def render_map(self):
        """
        Show OG grid
        """
        plt.clf()
        plt.imshow(1.0 - 1. / (1. + np.exp(self.map_grid)), 'Greys')
        plt.show()

    def save_map_to_disk(self):
        np.save('data/og_map', self.map_grid)

    def load_map_from_disk(self):
        self.map_grid = np.load('data/og_map.npy')

    def save_planned_route_to_disk(self):
        """
        Serialize route to disk
        """
        with open('data/planned_route', 'wb') as fp:
            pickle.dump(self.planned_route, fp)  # Serialise route which is list of tuples

    def load_planned_route_from_disk(self):
        with open('data/planned_route', 'rb') as fp:
            self.planned_route = pickle.load(fp)

    def path_planner(self):
        """
        Look for optimal, lowest cost path from beacon to home point using occupancy grid
        """
        # Load occupancy grid
        self.load_map_from_disk()

        # Transform occupancy grid in array of binary values where 0 represents passable cell and 1 impassable cell
        self.map_grid_binary = np.zeros((self.x_size, self.y_size))
        for iy, ix in np.ndindex(self.map_grid.shape):
            if self.map_grid[iy, ix] == 0:  # Flag unknown cell as impassable
                self.map_grid_binary[iy, ix] = 1
            if -0.5 < self.map_grid[iy, ix] < 0:
                self.map_grid_binary[iy, ix] = 1

        marker_size = 4500
        marker_beacon = '$BEACON$'
        marker_home = '$HOME$'
        color_map = plt.cm.Accent

        #  Save plot of binary map before dilation
        fig, ax = plt.subplots(figsize=(20, 20))
        ax.imshow(self.map_grid_binary, cmap=color_map)
        ax.scatter(self.beacon_loc[1], self.beacon_loc[0], marker=marker_beacon, color='white', s=marker_size)
        ax.scatter(self.hp_centre_loc[1], self.hp_centre_loc[0], marker=marker_home, color='white', s=marker_size)
        plt.savefig(fname='plots/binary map from og.png', dpi=300, format='png')
        plt.clf()

        # Use binary dilation technique to expand occupied cells to build in safety margin for robot, as A* does not
        # account for robot dimensions
        self.map_grid_binary = binary_dilation(self.map_grid_binary, structure=np.ones((35, 35))).astype(int)

        # Save binary map now enhanced with binary dilation
        fig, ax = plt.subplots(figsize=(20, 20))
        ax.imshow(self.map_grid_binary, cmap=color_map)
        ax.scatter(self.beacon_loc[1], self.beacon_loc[0], marker=marker_beacon, color='white', s=marker_size)
        ax.scatter(self.hp_centre_loc[1], self.hp_centre_loc[0], marker=marker_home, color='white', s=marker_size)
        plt.savefig(fname='plots/binary dilation map.png', dpi=300, format='png')
        plt.clf()

        # Find optimal path from beacon to home using A* algorithm developed by Christian Careaga
        # http://code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/
        self.planned_route = self.astar_pathfinding()
        self.planned_route = self.planned_route + [self.beacon_loc]
        self.planned_route = self.planned_route[::-1]

        # Save route to disk for later use
        self.save_planned_route_to_disk()

        # Plot map and path
        path_x = []
        path_y = []

        for i in (range(0, len(self.planned_route))):
            x = self.planned_route[i][0]
            y = self.planned_route[i][1]
            path_x.append(x)
            path_y.append(y)

        fig, ax = plt.subplots(figsize=(20, 20))
        ax.imshow(self.map_grid_binary, cmap=color_map)
        ax.scatter(self.beacon_loc[1], self.beacon_loc[0], marker=marker_beacon, color='red', s=marker_size)
        ax.scatter(self.hp_centre_loc[1], self.hp_centre_loc[0], marker=marker_home, color='red', s=marker_size)
        ax.plot(path_y, path_x, color='white', linewidth=2.0)
        plt.savefig(fname='plots/binary dilation map with path plan.png', dpi=300, format='png')
        plt.clf()

    def astar_pathfinding(self):
        """
        Author: Christian Careaga
        http://code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/
        """
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        close_set = set()
        came_from = {}

        gscore = {self.beacon_loc: 0}
        fscore = {self.beacon_loc: self.astar_gscore_heuristic(self.beacon_loc, self.hp_centre_loc)}

        oheap = []
        heapq.heappush(oheap, (fscore[self.beacon_loc], self.beacon_loc))

        while oheap:
            current = heapq.heappop(oheap)[1]

            if current == self.hp_centre_loc:
                data = []

                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.astar_gscore_heuristic(current, neighbor)
                if 0 <= neighbor[0] < self.map_grid_binary.shape[0]:
                    if 0 <= neighbor[1] < self.map_grid_binary.shape[1]:
                        if self.map_grid_binary[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        continue  # array bound y walls
                else:
                    continue  # array bound x walls

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.astar_gscore_heuristic(neighbor, self.hp_centre_loc)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        return False
