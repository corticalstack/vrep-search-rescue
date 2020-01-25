import os
import sys
import logging
import logger as lg
from datetime import datetime
import random
from controller import Controller

script_name = os.path.basename(sys.argv[0]).split('.')


class Scenario:
    """
    Scenario objectives: Task 1 - Turn random degrees
                         Task 2 - Move in straight line until within distance of wall
                         Task 3 - Follow perimeter wall for 1 loop
                         Task 4 - Stop
    """
    def __init__(self):
        lg.message(logging.INFO, 'Staring scenario ' + str(script_name[0]))

        # Properties
        # min_dist - stop robot at minimum distance from collidable object - UoM is metres
        # max_dist - max distance from object - UoM is metres
        # velocity - locomotive speed in metres per second
        # robot_dir_travel - 1=Forward, -1=Reverse
        # dir - 1=clockwise, -1=counter clockwise
        # method - algo controlling navigation & motor velocity - min=takes min from 1 sensor, multi=multiple sensors
        #        - multi (fast) - coeff = 7.5 (162s)
        #        - multi (slow) - coeff = 6.1 (206s)
        #        - min   (fast) - coeef = 1.6 (164s)
        #        - min   (slow) - coeef = 1.3 (207s)

        # self.world = {'props': {'min_dist_enabled': True, 'min_dist': 0.23, 'max_dist': 0.28},
        #               'events': [{'task': 'room_centre'},
        # #                          {'task': 'clear_ahead'},
        # #                          {'task': 'move', 'robot_dir_travel': 1, 'dist': 15, 'velocity': 0.4},
        #                          {'task': 'stop'}]}

        self.world = {'props': {'min_dist_enabled': True, 'min_dist': 0.23, 'max_dist': 0.28},
                      'events': [{'task': 'room_centre', 'mapping_enabled': True},
                                 {'task': 'clear_ahead', 'mapping_enabled': True},
                                 {'task': 'move', 'robot_dir_travel': 1, 'distm': 2.0, 'velocity': 0.2,
                                  'mapping_enabled': False},
                                 {'task': 'set_waypoint', 'wp': 'HP Doorway', 'mapping_enabled': False},
                                 {'task': 'random_wander', 'mapping_enabled': True},
                                 {'task': 'set_waypoint', 'wp': 'Beacon', 'mapping_enabled': False},
                                 {'task': 'stop',  'mapping_enabled': False},
                                 {'task': 'path_plan', 'mapping_enabled': False},
                                 {'task': 'go_home', 'mapping_enabled': False}]}

        # self.world = {'props': {'min_dist_enabled': True, 'min_dist': 0.05, 'max_dist': 0.28},
        #               'events': [{'task': 'turn', 'fixed': 90, 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 0, 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 270, 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 180, 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 3.0, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 270, 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 5.2, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 0, 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 1.0, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 170, 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 0, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 3.0, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 170, 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 0, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 45, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 3.5, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 320, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 185, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 90, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 2, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 290, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 90, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 3.3, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 290, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 180, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 4, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 350, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 130, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 3, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 270, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 180, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 2.3, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 340, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 270, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 3.0, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 340, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 90, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 330, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 3.0, 'velocity': 0.05, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 90, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True},
        #                          {'task': 'turn', 'fixed': 272, 'mapping_enabled': True},
        #                          {'task': 'stop', 'mapping_enabled': True}]}

        #""" Mapping resolution test with move speed=0.15
        # self.world = {'props': {'min_dist_enabled': True, 'min_dist': 0.05, 'max_dist': 0.28},
        #               'events': [{'task': 'turn', 'degrees': +359},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 5, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +91},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 5.1, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 8.75, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 8, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 8.75, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 3.5, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +45},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 8.25, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +45},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 5.75, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 13.25, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 13.15, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 13.5, 'velocity': 0.05},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 13, 'velocity': 0.05},
        #                          {'task': 'stop'}]}
        #"""

        #""" Mapping resolution test with move speed=0.2
        # self.world = {'props': {'min_dist_enabled': True, 'min_dist': 0.10, 'max_dist': 0.28},
        #               'events': [{'task': 'turn', 'degrees': +359},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 5, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +91},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 5, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 8.75, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 8, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 8.75, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 3.3, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +45},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 8, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +45},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 5.75, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 12.75, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 12.75, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 13, 'velocity': 0.2},
        #                          {'task': 'stop'},
        #                          {'task': 'turn', 'degrees': +90},
        #                          {'task': 'move', 'robot_dir_travel': 1, 'distm': 13, 'velocity': 0.2},
        #                          {'task': 'stop'}]}
        #"""
                                 # {'task': 'turn', 'degrees': +87},
                                 # {'task': 'move', 'robot_dir_travel': 1, 'dist': 14, 'velocity': 0.1},
                                 # {'task': 'stop'},
                                 # {'task': 'turn', 'degrees': +87},
                                 # {'task': 'move', 'robot_dir_travel': 1, 'dist': 6, 'velocity': 0.1},
                                 # {'task': 'stop'},
                                 # {'task': 'turn', 'degrees': +87},
                                 # {'task': 'move', 'robot_dir_travel': 1, 'dist': 5, 'velocity': 0.1},


        # self.world = {'props': {'min_dist_enabled': True, 'min_dist': 0.23, 'max_dist': 0.28},
        #               'events': [{'task': 'clear_ahead'}]}

        # self.world = {'props': {'min_dist_enabled': True, 'min_dist': 0.23, 'max_dist': 0.28},
        #               'events': [{'task': 'turn', 'degrees': +350}]}

        lg.message(logging.INFO, 'World props ' + str(self.world['props']))
        lg.message(logging.INFO, 'World events ' + str(self.world['events']))

        Controller(self.world)

        lg.message(logging.INFO, 'Scenario ' + str(script_name[0]) + ' complete')


if __name__ == "__main__":
    log_filename = str(script_name[0] +('_') + datetime.now().strftime("%Y%m%d-%H%M%S") + '.txt')

    logging.basicConfig(filename='logs/' + log_filename, level=logging.DEBUG,
                        format='%(asctime)s - %(levelname)s - %(message)s')

    scenario = Scenario()
