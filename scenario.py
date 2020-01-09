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

        self.world = {'props': {'min_dist_enabled': True, 'min_dist': 0.23, 'max_dist': 0.28},
                      'events': [{'task': 'room_centre'},
                                 {'task': 'clear_ahead'},
                                 {'task': 'move', 'robot_dir_travel': 1, 'dist': 15, 'velocity': 0.4},
                                 {'task': 'stop'}]}

        lg.message(logging.INFO, 'World props ' + str(self.world['props']))
        lg.message(logging.INFO, 'World events ' + str(self.world['events']))

        Controller(self.world)

        lg.message(logging.INFO, 'Scenario ' + str(script_name[0]) + ' complete')


if __name__ == "__main__":
    log_filename = str(script_name[0] +('_') + datetime.now().strftime("%Y%m%d-%H%M%S") + '.txt')

    logging.basicConfig(filename='logs/' + log_filename, level=logging.DEBUG,
                        format='%(asctime)s - %(levelname)s - %(message)s')

    scenario = Scenario()
