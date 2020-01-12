import logging
import logger as lg
import math
import numpy as np
import scipy.io
import scipy.stats
import matplotlib.pyplot as plt


class Mapper:
    """
    Implements environment mapping
    """
    def __init__(self, robot, handle=None):
        lg.message(logging.INFO, 'Initialising ' + self.__class__.__name__)
        self.robot = robot
        self.robot_handle = handle
        self.grid_res = 1
        self.x_size = int(1500 * self.grid_res)
        self.y_size = int(1500 * self.grid_res)
        self.map_grid = np.zeros((self.x_size, self.y_size))

        self.alpha = 0.2
        self.beta = 5.0 * np.pi / 180.0

        self.r_to_c_angle_grid = None
        self.r_to_c_distance_grid = None

        self.sensor_distance = 0
        self.sensor_angle = 0

        self.grid_coords_3d = np.array([np.tile(np.arange(0, self.x_size, 1)[:, None], (1, self.y_size)),
                                        np.tile(np.arange(0, self.y_size, 1)[:, None].T, (self.x_size, 1))])

        self.bearing = 0
        self.pose = []

        self.occupied = 0.367
        self.free = -0.693
        self.grid_to_pose = None

    def update_map(self):
        self.world_to_grid_bearing()
        if not self.world_to_grid_pose():
            return
        self.grid_to_pose = self.grid_coords_3d.copy()
        self.grid_to_pose[0, :, :] -= self.pose[0]  # x coordinates to pose
        self.grid_to_pose[1, :, :] -= self.pose[1]  # y coordinates to pose
        self.r_to_c_angle_grid = self.robot_to_cell_angle_grid()
        self.r_to_c_distance_grid = self.robot_to_cell_distance_grid()

        for i, sen in enumerate(self.robot.state['int']['prox_s'].last_read):
            if sen[1] is False:
                continue
            else:
                self.sensor_distance = sen[0]

            self.sensor_distance *= (100 * self.grid_res)
            self.sensor_angle = self.robot.state['int']['prox_s'].sensor_angle[i]

            fm = self.free_mask()
            om = self.occupied_mask()

            self.map_grid[om] += self.occupied
            self.map_grid[fm] += self.free

    def world_to_grid_bearing(self):
        self.bearing = self.robot.state['int']['compass'].last_read_euler
        if self.bearing > 0:
            self.bearing -= math.pi
        else:
            self.bearing += math.pi

    def world_to_grid_pose(self):
        self.pose = self.robot.state['ext']['abs_pos_n']
        if len(set(self.pose)) == 1:
            return False
        self.pose[0] = int((self.x_size / 2) + int(self.pose[0] * -100) * self.grid_res)
        self.pose[1] = int((self.y_size / 2) + int(self.pose[1] * -100) * self.grid_res)
        return True

    def robot_to_cell_angle_grid(self):
        return np.arctan2(self.grid_to_pose[1, :, :], self.grid_to_pose[0, :, :]) - self.bearing

    def robot_to_cell_distance_grid(self):
        return scipy.linalg.norm(self.grid_to_pose, axis=0)

    def free_mask(self):
        return (np.abs(self.r_to_c_angle_grid - self.sensor_angle) <= self.beta / 2.0) & \
               (self.r_to_c_distance_grid < (self.sensor_distance - self.alpha / 2.0))

    def occupied_mask(self):
        return (np.abs(self.r_to_c_angle_grid - self.sensor_angle) <= self.beta / 2.0) & \
               (np.abs(self.r_to_c_distance_grid - self.sensor_distance) <= self.alpha / 2.0)

    def render_map(self):
        plt.clf()
        plt.imshow(1.0 - 1. / (1. + np.exp(self.map_grid)), 'Greys')
        plt.show()



