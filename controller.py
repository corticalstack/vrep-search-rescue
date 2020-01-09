import sys
import time
from helper import Helper
from robots import PioneerP3dx
import logging
import logger as lg
import csv
import math
# This is an implementation of Occupancy Grid Mapping as Presented
# in Chapter 9 of "Probabilistic Robotics" By Sebastian Thrun et al.
# In particular, this is an implementation of Table 9.1 and 9.2


import scipy.io
import scipy.stats
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

class Map():
    def __init__(self, xsize, ysize, grid_size):
        self.xsize = xsize
        self.ysize = ysize
        self.grid_size = grid_size # save this off for future use
        self.log_prob_map = np.zeros((self.xsize, self.ysize)) # set all to zero

        self.alpha = 1 # The assumed thickness of obstacles
        #self.beta = 6
        #self.beta = 0.04
        self.beta = 0.0350
        self.z_max = 4

        # Pre-allocate the x and y positions of all grid positions into a 3D tensor
        # (pre-allocation = faster)
        self.grid_position_m = np.array([np.tile(np.arange(0, self.xsize*self.grid_size, self.grid_size)[:,None], (1, self.ysize)),
                                         np.tile(np.arange(0, self.ysize*self.grid_size, self.grid_size)[:,None].T, (self.xsize, 1))])

        # Log-Probabilities to add or remove from the map
        #self.l_occ = np.log(0.65/0.35)
        #self.l_free = np.log(0.35/0.65)

        self.l_occ = np.log(0.65 / 0.35)
        self.l_free = np.log(0.35 / 0.85)

    def update_map(self, pose, bearing, z):
        sen_angle = {0: -1.5708,
                     1: -0.872665,
                     2: -0.523599,
                     3: 0.174533,
                     4: 0.174533,
                     5: 0.523599,
                     6: 0.872665,
                     7: 1.5708,
                     8: 1.5708,
                     9: 2.26893,
                     10: 2.61799,
                     11: 2.96706,
                     12: -2.96706,
                     13: -2.61799,
                     14: -2.26893,
                     15: -1.5708}

        #bearing = bearing * math.pi/180
        print('bearing before', bearing)
        if bearing > 0:
            bearing = bearing - math.pi
        else:
            bearing = bearing + math.pi

        print('bearing ', bearing)
        dx = self.grid_position_m.copy() # A tensor of coordinates of all cells
        dx[0, :, :] -= pose[0] # A matrix of all the x coordinates of the cell
        dx[1, :, :] -= pose[1] # A matrix of all the y coordinates of the cell
        theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) - bearing # matrix of all bearings from robot to cell

        # Wrap to +pi / - pi
        theta_to_grid[theta_to_grid > np.pi] -= 2. * np.pi
        theta_to_grid[theta_to_grid < -np.pi] += 2. * np.pi

        dist_to_grid = scipy.linalg.norm(dx, axis=0) # matrix of L2 distance to all cells from robot

        # For each laser beam
        for i, z_i in enumerate(z):
            if z_i[1] is False:
                continue
            #if i != 3:
            #    continue
            r = z_i[0] # range measured
            r = r * 100
            print('Range ', r)
            b = z_i[1] # bearing measured
            b = b * math.pi/180  # Convert to rads
            b = sen_angle[i]
            # Calculate which cells are measured free or occupied, so we know which cells to update
            # Doing it this way is like a billion times faster than looping through each cell (because vectorized numpy is the only way to numpy)
            free_mask = (np.abs(theta_to_grid - b) <= self.beta) & (dist_to_grid < (r))
            occ_mask = (np.abs(theta_to_grid - b) <= self.beta) & (np.abs(dist_to_grid - r) <= self.alpha)
            # Adjust the cells appropriately
            self.log_prob_map[occ_mask] += self.l_occ
            self.log_prob_map[free_mask] += self.l_free


class Controller:
    """
    Controller managing connection to V-REP simulator, executing the physics loop etc
    """
    def __init__(self, world):
        lg.message(logging.INFO, 'Initialising ' + self.__class__.__name__)
        self.world_props = world['props']
        self.world_events = world['events']
        self.start_time = 0

        # Instantiate helper and connect to VREP
        self.h = Helper()
        if not self.h.connected:
            sys.exit()

        # Get all objects from running VREP scene
        self.full_scene_object_list, self.simple_object_list = self.h.get_objects()

        # Instantiate robot
        self.robot = PioneerP3dx(self.h, self.simple_object_list)

        self.loop()

        self.stats()

        # Disconnect from VREP
        self.h.disconnect_client()

    def loop(self):
        """
        Sequentially process planned events in a sense>think>act control cycle
        """
        lg.message(logging.INFO, 'Starting controller loop')

        self.start_time = time.time()

        stop_exceptions = ['wall_follow', 'room_centre', 'clear_ahead']
        step_status = {'start_t': time.time(),
                       'complete': None}

        grid_size = 1.0
        map = Map(int(1500 / grid_size), int(1500 / grid_size), grid_size)



        for step in self.world_events:
            lg.message(logging.INFO, 'Starting event - ' + step['task'])

            task_args = None

            step_status['start_t'] = time.time()  # Log start, as some actions have lifetime defined in route plan
            step_status['complete'] = None  # Step tristate - not started (None), in progress (False), complete (True)
            while not step_status['complete']:
                # Sense
                self.sense()

                # state is e.g. array([10., 15.,  0.])
                # meas = array of measure e.g. [14.        , -1.57079633]
                import vrep
                res, pos = vrep.simxGetObjectPosition(self.h.client_id, self.robot.handle, -1,
                                                      vrep.simx_opmode_streaming)
                bearing = self.robot.state['int']['compass'].last_read
                pose = pos
                pose[1] = 750 + int(pose[1] * -100)
                pose[0] = 750 + int(pose[0] * -100)

                map.update_map(pose, bearing, self.robot.state['int']['prox_s'].last_read)  # update the map

                # Think
                # Possible impending collision - trigger stop unless excepted actions
                if self.robot.is_less_min_prox_dir_travel() and step['task'] not in stop_exceptions:
                    lg.message(logging.INFO, 'Stop task triggered')
                    self.robot.stop(step_status, self.world_props, task_args)
                    continue

                if self.robot.is_prox_to_beacon() and step['task'] not in stop_exceptions:
                    print('Proximity to beacon')
                    lg.message(logging.INFO, 'Stop task triggered')
                    self.robot.stop(step_status, self.world_props, task_args)
                    continue

                # Act
                if step_status['complete'] is None:
                    # Execute current route plan step
                    task_args = {arg: step[arg] for arg in step if arg not in 'task'}
                    lg.message(logging.DEBUG, 'Executing method ' + str(step['task']) + ' args ' + str(task_args))
                    getattr(self.robot, step['task'])(step_status, self.world_props, task_args)
                    continue

                getattr(self.robot, step['task'])(step_status, self.world_props, task_args)

                #time.sleep(0.002)

        plt.clf()


        #circle = plt.Circle((pose[1], pose[0]), radius=3.0, fc='y')
        #plt.gca().add_patch(circle)

        #arrow = pose[0:2] + np.array([3.5, 0]).dot(
        #    np.array([[np.cos(bearing), np.sin(bearing)], [-np.sin(bearing), np.cos(bearing)]]))
        #plt.plot([pose[1], arrow[1]], [pose[0], arrow[0]])
        plt.imshow(1.0 - 1. / (1. + np.exp(map.log_prob_map)), 'Greys')
        #plt.pause(0.005)
        import seaborn as sns
        #ax = sns.heatmap(map.log_prob_map)
        plt.show()


    def sense(self):
        """
        Update state on how robot perceives world
        """
        self.robot.update_state_position()
        self.robot.update_state_proximity(self.world_props)
        self.robot.update_state_compass()
        self.robot.update_state_odometry()
        self.robot.update_state_beacon()

    def stats(self):
        """
        Show simulation statistics
        """
        time_taken = round(time.time() - self.start_time, 2)

        avg_joint_dist = 0
        for m in self.robot.state['int']['motors']:
            avg_joint_dist += self.robot.state['int']['jpos'][str(self.robot.state['int']['motors'][m] + '_dist')]

        avg_joint_dist = round(avg_joint_dist / 2, 2)
        #avg_speed_ms = round(avg_joint_dist / time_taken, 2)
        lg.message(logging.INFO, 'Distance travelled - {}m'.format(avg_joint_dist))
        #lg.message(logging.INFO, 'Average speed - {}m/s'.format(avg_speed_ms))
        lg.message(logging.INFO, 'Nav dist diff error - {}cm'.format(round(self.robot.state['int']['err_corr_count'], 2)))
        lg.message(logging.INFO, 'Controller loop complete - time taken - {}s'.format(round(time.time() -
                                                                                            self.start_time, 2)))

        with open('output/abs_pos_all.csv', 'w', newline='') as out:
            csv_output = csv.writer(out)
            csv_output.writerows(self.robot.state['ext']['abs_pos_all'])

        with open('output/motor_all_v', 'w', newline='') as out:
            csv_output = csv.writer(out)
            csv_output.writerows(self.robot.state['int']['motor_all_v'])

        with open('output/error_history', 'w', newline='') as out:
            csv_output = csv.writer(out)
            csv_output.writerows(self.robot.state['int']['error_history'])





# class Map():
#     def __init__(self, xsize, ysize, grid_size):
#         self.xsize = xsize+2 # Add extra cells for the borders
#         self.ysize = ysize+2
#         self.grid_size = grid_size # save this off for future use
#         self.log_prob_map = np.zeros((self.xsize, self.ysize)) # set all to zero
#
#         self.alpha = 1.0 # The assumed thickness of obstacles
#         self.beta = 5.0*np.pi/180.0 # The assumed width of the laser beam
#         self.z_max = 150.0 # The max reading from the laser
#
#         # Pre-allocate the x and y positions of all grid positions into a 3D tensor
#         # (pre-allocation = faster)
#         self.grid_position_m = np.array([np.tile(np.arange(0, self.xsize*self.grid_size, self.grid_size)[:,None], (1, self.ysize)),
#                                          np.tile(np.arange(0, self.ysize*self.grid_size, self.grid_size)[:,None].T, (self.xsize, 1))])
#
#         # Log-Probabilities to add or remove from the map
#         self.l_occ = np.log(0.65/0.35)
#         self.l_free = np.log(0.35/0.65)
#
#     def update_map(self, pose, z):
#
#         dx = self.grid_position_m.copy() # A tensor of coordinates of all cells
#         dx[0, :, :] -= pose[0] # A matrix of all the x coordinates of the cell
#         dx[1, :, :] -= pose[1] # A matrix of all the y coordinates of the cell
#         theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) - pose[2] # matrix of all bearings from robot to cell
#
#         # Wrap to +pi / - pi
#         theta_to_grid[theta_to_grid > np.pi] -= 2. * np.pi
#         theta_to_grid[theta_to_grid < -np.pi] += 2. * np.pi
#
#         dist_to_grid = scipy.linalg.norm(dx, axis=0) # matrix of L2 distance to all cells from robot
#
#         # For each laser beam
#         for z_i in z:
#             r = z_i[0] # range measured
#             b = z_i[1] # bearing measured
#
#             # Calculate which cells are measured free or occupied, so we know which cells to update
#             # Doing it this way is like a billion times faster than looping through each cell (because vectorized numpy is the only way to numpy)
#             free_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (dist_to_grid < (r - self.alpha/2.0))
#             occ_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (np.abs(dist_to_grid - r) <= self.alpha/2.0)
#
#             # Adjust the cells appropriately
#             self.log_prob_map[occ_mask] += self.l_occ
#             self.log_prob_map[free_mask] += self.l_free

# if __name__ == '__main__':
#
#     # https://gist.githubusercontent.com/superjax/33151f018407244cb61402e094099c1d/raw/e31868f4297fa0eab612178876025512a87a26c4/occupancy_grid_mapping_example.py
#     # load matlab generated data (located at http://jamessjackson.com/files/index.php/s/sdKzy9nnqaVlKUe)
#     data = scipy.io.loadmat('state_meas_data.mat')
#     state = data['X']
#     meas = data['z']
#
#     # Define the parameters for the map.  (This is a 100x100m map with grid size 1x1m)
#     #grid_size = 1.0
#     #map = Map(int(100/grid_size), int(100/grid_size), grid_size)
#     grid_size = 1.0
#     map = Map(int(15 / grid_size), int(15 / grid_size), grid_size)
#
#     plt.ion() # enable real-time plotting
#     plt.figure(1) # create a plot
#     for i in tqdm(range(len(state.T))):
#         map.update_map(state[:,i], meas[:,:,i].T) # update the map
#
#         # Real-Time Plotting
#         # (comment out these next lines to make it run super fast, matplotlib is painfully slow)
#         plt.clf()
#         pose = state[:,i]
#         circle = plt.Circle((pose[1], pose[0]), radius=3.0, fc='y')
#         plt.gca().add_patch(circle)
#         arrow = pose[0:2] + np.array([3.5, 0]).dot(np.array([[np.cos(pose[2]), np.sin(pose[2])], [-np.sin(pose[2]), np.cos(pose[2])]]))
#         plt.plot([pose[1], arrow[1]], [pose[0], arrow[0]])
#         plt.imshow(1.0 - 1./(1.+np.exp(map.log_prob_map)), 'Greys')
#         plt.pause(0.005)
#
#     # Final Plotting
#     plt.ioff()
#     plt.clf()
#     plt.imshow(1.0 - 1./(1.+np.exp(map.log_prob_map)), 'Greys') # This is probability
#     plt.imshow(map.log_prob_map, 'Greys') # log probabilities (looks really cool)
#     plt.show()