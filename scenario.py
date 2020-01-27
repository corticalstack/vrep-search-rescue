import os
import sys
import logging
import logger as lg
from datetime import datetime
import random
from controller import Controller

script_name = os.path.basename(sys.argv[0]).split('.')


class Scenario:
    def __init__(self):
        lg.message(logging.INFO, 'Staring scenario ' + str(script_name[0]))

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

        lg.message(logging.INFO, 'World props ' + str(self.world['props']))
        lg.message(logging.INFO, 'World events ' + str(self.world['events']))

        Controller(self.world)

        lg.message(logging.INFO, 'Scenario ' + str(script_name[0]) + ' complete')


if __name__ == "__main__":
    log_filename = str(script_name[0] +('_') + datetime.now().strftime("%Y%m%d-%H%M%S") + '.txt')

    logging.basicConfig(filename='logs/' + log_filename, level=logging.DEBUG,
                        format='%(asctime)s - %(levelname)s - %(message)s')

    scenario = Scenario()
