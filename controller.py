import sys
import time
from helper import Helper
from robots import PioneerP3dx
import logging
import logger as lg
import csv

import cProfile, pstats, io
from pstats import SortKey
pr = cProfile.Profile()


class Controller:
    """
    Controller managing connection to V-REP simulator, executing the physics loop etc
    """
    def __init__(self, world):
        lg.message(logging.INFO, 'Initialising ' + self.__class__.__name__)
        self.world_props = world['props']
        self.world_events = world['events']
        self.start_time = 0
        self.scenario_mapping_enabled = False
        self.step_mapping_enabled = False

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

        stop_exceptions = ['room_centre', 'turn', 'random_wander', 'go_home']
        beacon_tasks = ['random_wander']
        step_status = {'start_t': time.time(),
                       'start_m': self.robot.get_distance(),
                       'complete': None}

        for step in self.world_events:
            lg.message(logging.INFO, 'Starting event - ' + step['task'])

            task_args = None

            step_status['start_t'] = time.time()  # Log start, as some actions have lifetime defined in route plan
            step_status['start_m'] = self.robot.get_distance()
            step_status['complete'] = None  # Step tristate - not started (None), in progress (False), complete (True)

            while not step_status['complete']:
                # Sense
                self.sense()

                # Think
                # Possible impending collision - trigger stop unless excepted actions
                if self.robot.is_less_min_prox_dir_travel() and step['task'] not in stop_exceptions:
                    lg.message(logging.INFO, 'Stop task triggered')
                    self.robot.stop(step_status, self.world_props, task_args)
                    continue

                # Act
                if step_status['complete'] is None:
                    # Execute current route plan step
                    task_args = {arg: step[arg] for arg in step if arg not in 'task'}

                    # Check if mapping enabled for specific task. If any tasks enable mapping then flag this scenario
                    # does mapping in at least one step so the OG can be saved to disk etc
                    if 'mapping_enabled' in task_args:
                        if task_args['mapping_enabled'] is True:
                            self.step_mapping_enabled = True
                            self.scenario_mapping_enabled = True
                        else:
                            self.step_mapping_enabled = False

                    lg.message(logging.DEBUG, 'Executing method ' + str(step['task']) + ' args ' + str(task_args))
                    getattr(self.robot, step['task'])(step_status, self.world_props, task_args)
                    continue

                getattr(self.robot, step['task'])(step_status, self.world_props, task_args)

                if step['task'] in beacon_tasks:
                    if self.robot.is_prox_to_beacon():
                        self.random_wander_housekeeping_stats(step_status, task_args)

                if step['task'] == 'go_home' and step_status['complete'] is True:
                    self.go_home_stats(step_status, task_args)

        # Mapping done in at least one step so render and save
        if self.scenario_mapping_enabled:
            self.robot.state['ext']['mapper'].render_map()
            self.robot.state['ext']['mapper'].save_map_to_disk()

    def sense(self):
        """
        Update state on how robot perceives world
        """
        self.robot.update_state_position()
        self.robot.update_state_proximity(self.world_props)
        self.robot.update_state_compass()
        self.robot.update_state_odometry()
        self.robot.update_state_beacon()
        if self.step_mapping_enabled:
            self.robot.update_state_map()

    def random_wander_housekeeping_stats(self, ss, ta):
        self.robot.state['int']['rw_status']['complete'] = True
        lg.message(logging.INFO, 'Proximity to beacon triggered')

        if self.step_mapping_enabled:
            self.robot.state['ext']['mapper'].save_map_to_disk()
            lg.message(logging.INFO, 'OG map saved to disk')

        lg.message(logging.INFO, 'Stop task triggered')
        self.robot.stop(ss, self.world_props, ta)
        lg.message(logging.INFO, 'RW Distance travelled - {}m'.format(
            self.robot.get_distance() - self.robot.state['int']['rw_status']['start_m']))
        lg.message(logging.INFO, 'RW time taken - {}s'.format(
            round(time.time() - self.robot.state['int']['rw_status']['start_t'], 2)))
        lg.message(logging.INFO, 'RW number of turns - {}'.format(
            self.robot.state['int']['rw_status']['turns_count']))

    def go_home_stats(self, ss, ta):
        lg.message(logging.INFO, 'Completed path to home')
        lg.message(logging.INFO, 'Stop task triggered')
        self.robot.stop(ss, self.world_props, ta)
        lg.message(logging.INFO, 'GH Distance travelled - {}m'.format(
            self.robot.get_distance() - self.robot.state['int']['gh_status']['start_m']))
        lg.message(logging.INFO, 'GH time taken - {}s'.format(
            round(time.time() - self.robot.state['int']['gh_status']['start_t'], 2)))
        lg.message(logging.INFO, 'GH number of route steps taken - {}'.format(
            self.robot.state['int']['gh_status']['steps_count']))

    def stats(self):
        """
        Show simulation statistics
        """
        avg_joint_dist = self.robot.get_distance()
        lg.message(logging.INFO, 'Distance travelled - {}m'.format(avg_joint_dist))
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
