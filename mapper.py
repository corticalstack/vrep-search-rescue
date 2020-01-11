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
        #self.x_size = 1500
        #self.y_size = 1500
        self.map = np.zeros((self.x_size, self.y_size))

        self.alpha = 0.2
        self.beta = 5.0 * np.pi / 180.0

        self.r_to_c_angle_grid = None
        self.r_to_c_distance_grid = None

        self.sensor_distance = 0
        self.sensor_angle = 0

        # Pre-allocate the x and y positions of all grid positions into a 3D tensor
        # (pre-allocation = faster)
        self.grid_position_m = np.array([np.tile(np.arange(0, self.x_size, 1)[:, None], (1, self.y_size)),
                                         np.tile(np.arange(0, self.y_size, 1)[:, None].T, (self.x_size, 1))])

        self.bearing = 0
        self.pose = []
        self.log_occupied = np.log(0.65 / 0.35)
        self.log_free = np.log(0.35 / 0.65)
        self.dx = None

    def update_map(self):
        self.world_to_grid_bearing()
        if not self.world_to_grid_pose():
            return
        self.dx = self.grid_position_m.copy()  # A tensor of coordinates of all cells
        self.dx[0, :, :] -= self.pose[0]   # A matrix of all the x coordinates of the cell
        self.dx[1, :, :] -= self.pose[1]  # A matrix of all the y coordinates of the cell
        self.r_to_c_angle_grid = self.robot_to_cell_angle_grid()
        self.r_to_c_distance_grid = self.robot_to_cell_distance_grid()

        for i, z_i in enumerate(self.robot.state['int']['prox_s'].last_read):
            if z_i[1] is False:
                # r = 4
                continue
            else:
                self.sensor_distance = z_i[0]  # range measured

            # #if i == 1 or i == 2 or i == 2 or i == 3 or i == 4 or i == 5 or i == 5 or i == 6:
            # if i == 1 or i == 2 or i == 3 or i == 4 or i == 5 or i ==6:
            #     pass
            # else:
            #     continue

            self.sensor_distance *= (100 * self.grid_res)
            self.sensor_angle = self.robot.state['int']['prox_s'].sensor_angle[i]

            fm = self.free_mask()
            om = self.occupied_mask()

            self.map[om] += self.log_occupied
            self.map[fm] += self.log_free

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
        return np.arctan2(self.dx[1, :, :], self.dx[0, :, :]) - self.bearing  # matrix of all bearings from robot to cell

    def robot_to_cell_distance_grid(self):
        return scipy.linalg.norm(self.dx, axis=0)  # matrix of L2 distance to all cells from robot

    def free_mask(self):
        return (np.abs(self.r_to_c_angle_grid - self.sensor_angle) <= self.beta / 2.0) & \
               (self.r_to_c_distance_grid < (self.sensor_distance - self.alpha / 2.0))

    def occupied_mask(self):
        return (np.abs(self.r_to_c_angle_grid - self.sensor_angle) <= self.beta / 2.0) & \
               (np.abs(self.r_to_c_distance_grid - self.sensor_distance) <= self.alpha / 2.0)

    def render_map(self):
        plt.clf()
        #circle = plt.Circle((pose[1], pose[0]), radius=3.0, fc='y')
        #plt.gca().add_patch(circle)

        #arrow = pose[0:2] + np.array([3.5, 0]).dot(
        #    np.array([[np.cos(bearing), np.sin(bearing)], [-np.sin(bearing), np.cos(bearing)]]))
        #plt.plot([pose[1], arrow[1]], [pose[0], arrow[0]])
        plt.imshow(1.0 - 1. / (1. + np.exp(self.map)), 'Greys')
        #plt.pause(0.005)
        import seaborn as sns

        plt.show()
        plt.clf()
        ax = sns.heatmap(self.map)
        plt.show()



