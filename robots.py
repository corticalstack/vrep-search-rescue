import logging
import logger as lg
import vrep
import math
import time
from sensors import ProximitySensor, Compass, Beacon
from mapper import Mapper
import numpy as np
from operator import itemgetter
import statistics
import random
import scipy.io


class Robot:
    """
    Super class implementing robot
    """
    def __init__(self, name, simple_object_list, helper):
        self.h = helper

        # Robot objects
        self.name = name
        self.handle = self.h.get_object_handle_by_name(self.name)
        self.component_tree = self.set_object_tree()

        # Robot facing direction 1=forward, -1=back
        self.robot_dir_travel = 1

    def set_object_tree(self):
        """
        Get object tree for robot
        """
        self.h.get_object_tree(self.handle, 0)
        return self.h.object_tree

    def get_components(self, tag, simple_object_list):
        """
        Get component tree for vrep object
        """
        lg.message(logging.DEBUG, 'Getting V-REP components for ' + tag)
        components = {}
        for comp in self.component_tree:
            for child in self.component_tree[comp].children:
                if tag in simple_object_list[child]:
                    components[child] = simple_object_list[child]
        return components

    def set_joint_v(self, joint, velocity):
        """
        Set joint velocity
        """
        vrep.simxSetJointTargetVelocity(self.h.client_id, joint, velocity, vrep.simx_opmode_streaming)


class PioneerP3dx(Robot):
    """
    Pioneer P3DX robot implementation
    """
    def __init__(self, helper, simple_object_list):
        Robot.__init__(self, 'Pioneer_p3dx', simple_object_list, helper)
        lg.message(logging.INFO, 'Initialising ' + self.__class__.__name__)

        # Pioneer specific properties
        self.props = {'wheel_rad_m': 0.195,
                      'wheel_circ_m': 0.61261,
                      'chassis_l_mm': 455,
                      'chassis_w_mm': 381,
                      'chassis_h_mm': 237,
                      'weight_kg': 9,
                      'operating_payload_kg': 17,
                      'def_ms_v': 0.15,
                      'min_speed_ms': 0.12,
                      'max_speed_ms': 1.2,
                      'sensor_us_weights': [0.75 / 180.0 * math.pi, 0.55 / 180.0 * math.pi, 0.5 / 180.0 * math.pi,
                                            0.25 / 180.0 * math.pi, 0.25 / 180.0 * math.pi, 0.5 / 180.0 * math.pi,
                                            0.55 / 180.0 * math.pi, 0.75 / 180.0 * math.pi]}
        self.turn_dir = 0

        # Pioneer specific internal and external state
        self.state = {'int': {'motors': [],
                              'motor_l_v': 0.0,
                              'motor_r_v': 0.0,
                              'motor_all_v': [],
                              'pid': {'kp': 7.29, 'ki': 0.0, 'kd': -7.0},
                              'pid_integral_clamp': 0.09,
                              'pid_prev_error': 0.0,
                              'pid_sum_error_short_term': [],
                              'pid_sum_error': 0,
                              'jpos': {},
                              'prox_history': [],
                              'prox_s_arr': [],
                              'prox_s': None,
                              'prox_is_less_min_dist_f': False,
                              'prox_is_less_min_dist_r': False,
                              'rw_status': {'complete': None, 'turns_count': 0, 'start_t': 0, 'start_m': 0},
                              'rw_turn_status': {'complete': None, 'degrees': 0, 'args': {}},
                              'rw_move_status': {'complete': None, 'startm': 0, 'args': {}},
                              'gh_status': {'complete': None, 'steps_count': 0, 'start_t': 0, 'start_m': 0, 'pose': [],
                                            'route_x': 0, 'route_y': 0},
                              'gh_turn_status': {'complete': None, 'degrees': 0, 'args': {}},
                              'gh_move_status': {'complete': None, 'startm': 0, 'args': {}},
                              'prox_min_dist_f': 0,
                              'prox_min_dist_r': 0,
                              'compass': None,
                              'err_corr_count': 0,
                              'cycle_i': 0,
                              'error_history': []},
                      'ext': {'abs_pos_s': None,
                              'abs_pos_n': None,
                              'abs_pos_all': [],
                              'waypoints': {},
                              'beacon': None,
                              'mapper': None}}

        # Initialise internal state for robot components
        self.state['int']['motors'] = self.get_components('Motor', simple_object_list)
        self.state['int']['prox_s_arr'] = self.get_components('ultrasonic', simple_object_list)

        # Sort sensor in ascending order by name - check the sensor ordering by name in VREP!!!
        self.state['int']['prox_s_arr'] = {k: v for k, v in sorted(self.state['int']['prox_s_arr'].items(), key=lambda
            item: item[1])}

        self.state['int']['prox_s'] = ProximitySensor(helper.client_id, sensor_array=self.state['int']['prox_s_arr'])
        self.state['int']['compass'] = Compass(helper.client_id, handle=self.handle)
        self.state['ext']['beacon'] = Beacon(helper.client_id, handle=self.handle)
        self.state['ext']['mapper'] = Mapper(self, handle=self.handle)
        self.set_joint_pos()

    def set_state_proximity(self, min_distance):
        """
        Boolean state indicating whether robot within specified minimum distance as detected front or rear array
        """
        distance_readings = [dr[0] for dr in self.state['int']['prox_s'].last_read[3:5]]
        self.state['int']['prox_min_dist_f'] = min(distance_readings)

        distance_readings = [dr[0] for dr in self.state['int']['prox_s'].last_read[11:13]]
        self.state['int']['prox_min_dist_r'] = min(distance_readings)

        if self.state['int']['prox_min_dist_f'] < min_distance:
            self.state['int']['prox_is_less_min_dist_f'] = True
        else:
            self.state['int']['prox_is_less_min_dist_f'] = False

        if self.state['int']['prox_min_dist_r'] < min_distance:
            self.state['int']['prox_is_less_min_dist_r'] = True
        else:
            self.state['int']['prox_is_less_min_dist_r'] = False

    def is_less_min_prox_dir_travel(self):
        """
        Returns boolean state indicating if robot proximity less than minimum distance for direction of travel
        """
        if self.robot_dir_travel == 1:
            return self.state['int']['prox_is_less_min_dist_f']
        else:
            return self.state['int']['prox_is_less_min_dist_r']

    def prox_dist_dir_travel(self):
        """
        Returns minimum distance between robot and collidable object for direction of travel
        """
        if self.robot_dir_travel:
            return self.state['int']['prox_min_dist_f']
        else:
            return self.state['int']['prox_min_dist_r']

    def is_prox_to_beacon(self):
        """
        Returns boolean state indicating if robot proximity less than minimum distance to beacon
        """
        if self.state['ext']['beacon'].last_read < 0.5:
            return True
        else:
            return False

    def update_state_proximity(self, props):
        """
        Read sensor array and update associated proximity states
        """
        self.state['int']['prox_s'].read(vrep.simx_opmode_buffer, sensor_array=self.state['int']['prox_s_arr'])
        if 'min_dist_enabled' in props:
            if props['min_dist_enabled']:
                self.set_state_proximity(props['min_dist'])

    def update_state_compass(self):
        """
        Get current object bearing and update state
        """
        self.state['int']['compass'].read(vrep.simx_opmode_buffer, handle=self.handle)

    def update_state_beacon(self):
        """
        Get current distance to beacon
        """
        self.state['ext']['beacon'].read(vrep.simx_opmode_buffer, handle=self.handle)

    def update_state_map(self):
        """
        Update map
        """
        self.state['ext']['mapper'].update_map()

    def set_motor_v(self):
        """
        Set motor velocity for each motor joint after updating state with conversion m/s to rad/s
        """
        self.state['int']['cycle_i'] += 1

        if self.state['int']['motor_l_v'] > self.props['max_speed_ms']:  # Cap velocity to top speed per data sheet
            self.state['int']['motor_l_v'] = self.props['max_speed_ms']
            lg.message(logging.WARNING, 'Max speed left motor limited to ' + str(self.props['max_speed_ms']))

        if self.state['int']['motor_r_v'] > self.props['max_speed_ms']:  # Cap velocity to top speed per data sheet
            self.state['int']['motor_r_v'] = self.props['max_speed_ms']
            lg.message(logging.WARNING, 'Max speed right motor limited to ' + str(self.props['max_speed_ms']))

        lg.message(logging.DEBUG, 'Setting motor left velocity (m/s) - ' + str(self.state['int']['motor_l_v']))
        lg.message(logging.DEBUG, 'Setting motor right velocity (m/s) - ' + str(self.state['int']['motor_r_v']))

        self.state['int']['motor_l_v'] = self.h.ms_to_radss(self.state['int']['motor_l_v'], self.props['wheel_rad_m'])
        self.state['int']['motor_r_v'] = self.h.ms_to_radss(self.state['int']['motor_r_v'], self.props['wheel_rad_m'])

        self.state['int']['motor_all_v'].append((self.state['int']['cycle_i'], self.state['int']['motor_l_v'],
                                                 self.state['int']['motor_r_v']))

        for m in self.state['int']['motors']:
            if 'left' in self.state['int']['motors'][m]:
                self.set_joint_v(m, self.state['int']['motor_l_v'])
            else:
                self.set_joint_v(m, self.state['int']['motor_r_v'])

        lg.message(logging.DEBUG, 'Motor left velocity now set at (rad/s) - ' + str(self.state['int']['motor_l_v']))
        lg.message(logging.DEBUG, 'Motor right velocity now set at (rad/s) - ' + str(self.state['int']['motor_r_v']))

    def get_distance(self):
        """
        Aggregate distance rotated by motor revolute joints
        """
        avg_joint_dist = 0
        try:
            for m in self.state['int']['motors']:
                avg_joint_dist += self.state['int']['jpos'][str(self.state['int']['motors'][m] + '_dist')]
            return round(avg_joint_dist / 2, 2)
        except KeyError:
            return 0

    def set_state_pos_start(self):
        """
        Set starting position as external, absolute position
        """
        res, self.state['ext']['abs_pos_s'] = vrep.simxGetObjectPosition(self.h.client_id, self.handle, -1,
                                                                         vrep.simx_opmode_buffer)
        lg.message(logging.DEBUG, 'Start point set ' + str(self.state['ext']['abs_pos_s']))

    def update_state_position(self):
        """
        Set current (now) position as external, absolute position
        """
        res, self.state['ext']['abs_pos_n'] = vrep.simxGetObjectPosition(self.h.client_id, self.handle, -1,
                                                                         vrep.simx_opmode_streaming)
        self.state['ext']['abs_pos_all'].append(self.state['ext']['abs_pos_n'][0:2])

    def set_waypoint(self, step_status, world_props, args):
        """
        Set waypoint
        """
        res, self.state['ext']['waypoints'][args['wp']] = vrep.simxGetObjectPosition(self.h.client_id, self.handle, -1,
                                                                                   vrep.simx_opmode_buffer)
        lg.message(logging.DEBUG, 'Waypoint ' + args['wp'] + ' set to ' + str(self.state['ext']['waypoints'][args['wp']]))
        step_status['complete'] = True

    def stop(self, step_status, world_props, args):
        """
        Stop robot locomotion
        """
        self.state['int']['motor_l_v'] = 0
        self.state['int']['motor_r_v'] = 0
        self.set_motor_v()
        step_status['complete'] = True

    def move(self, step_status, world_props, args):
        """
        Move robot as locomotion task
        """

        # Set p controller gain and error
        kp = 1
        distance = self.prox_dist_dir_travel()
        if distance > 1:
            distance = 1
        error = distance * kp

        if 'robot_dir_travel' in args:
            self.robot_dir_travel = args['robot_dir_travel']

        velocity = self.props['def_ms_v']  # Set default velocity
        if 'velocity' in args:
            velocity = args['velocity']

        baseline_v = velocity * error

        self.state['int']['motor_l_v'] = baseline_v * self.robot_dir_travel
        self.state['int']['motor_r_v'] = baseline_v * self.robot_dir_travel
        self.set_motor_v()
        step_status['complete'] = False

        # Velocity as a distance is applied for specified time. Continue moving until "distt" satisfied or stop if
        # proximity detected
        if 'distt' in args:
            if (time.time() - step_status['start_t'] > args['dist']) or self.is_less_min_prox_dir_travel():
                step_status['complete'] = True
                lg.message(logging.INFO, 'Move distt event complete')

        # Metres as a distance. Continue moving until "distm" satisfied or stop if proximity detected
        if 'distm' in args:
            if (self.get_distance() - step_status['start_m'] > args['distm']) or self.is_less_min_prox_dir_travel():
                step_status['complete'] = True
                lg.message(logging.INFO, 'Move distm event complete')

    def turn(self, step_status, world_props, args):
        """
        Turn robot task - calculate new bearing relative to current orientation and turn cw or ccw at constant speed
        """
        if step_status['complete'] is None:
            if 'degrees' in args:
                self.state['int']['compass'].set_to_bearing_add_deg(args['degrees'])
                lg.message(logging.DEBUG, 'Turn bearing from {} to {}'.format(self.state['int']['compass'].last_read,
                                                                              self.state['int']['compass'].to_bearing))
            if 'fixed' in args:
                self.state['int']['compass'].set_to_bearing_fixed(args['fixed'])
                lg.message(logging.DEBUG, 'Fixed to bearing {}'.format(self.state['int']['compass'].to_bearing))

            # Determine shortest way to turn between 2 bearings
            if (self.state['int']['compass'].to_bearing - self.state['int']['compass'].last_read_mag_deg + 360) % 360 > 180:
                self.turn_dir = 1
            else:
                self.turn_dir = 0

            step_status['complete'] = False

        kp = 0.3  # p-controller gain
        radius_threshold = 0.25
        if 'radius_threshold' in args:
            radius_threshold = args['radius_threshold']

        # Set p-controller error
        error = (self.state['int']['compass'].to_bearing -
                 self.state['int']['compass'].last_read_mag_deg + 540) % 360 - 180

        if abs(error) < radius_threshold:
            self.stop(step_status, world_props, {})
            step_status['complete'] = True
            lg.message(logging.INFO, 'Turn event complete')
            return

        #  Cap error (based on bearing diff which can be large) for controlled rotational speed
        if self.turn_dir == 0:
            error = abs(error)
            if error > 30:  # Cap diff
                error = 30
            self.state['int']['motor_l_v'] = 0.003 * (error * kp)
            self.state['int']['motor_r_v'] = -0.003 * (error * kp)
        else:
            error = abs(error)
            if error > 30:
                error = 30
            self.state['int']['motor_l_v'] = -0.003 * (error * kp)
            self.state['int']['motor_r_v'] = 0.003 * (error * kp)

        self.set_motor_v()

    def room_centre(self, step_status, world_props, args):
        kp = 0.155
        clip_distance = 3
        sensors = [s[0] if s[0] < clip_distance else clip_distance for s in self.state['int']['prox_s'].last_read]

        # Used sensors numbered in VREP/Pioneer specs as 0 (front-left), 3 (front), 7 (front right) and 11 (rear)
        sensor_index = [0, 3, 7, 11]

        # Get subset of sensor distances using specified sensor indexes
        sensor_list = list(itemgetter(*sensor_index)(sensors))

        # Get standard deviation for set of sensor values to determine diff between them
        error = statistics.stdev(sensor_list)

        # Task is complete if all sensors read acceptable similar distance to each other (standard deviation)
        if abs(error) < 0.20:
            step_status['complete'] = True
            lg.message(logging.INFO, 'Room centre event complete')
            self.set_waypoint({}, {}, {'wp': 'HP Centre'})
            return

        # Errors as diff between opposing sensors
        left_right_error = sensor_list[0] - sensor_list[2]
        front_rear_error = sensor_list[1] - sensor_list[3]

        # Drive motors using horizontal and vertical error
        if left_right_error < 0:
            self.state['int']['motor_l_v'] = left_right_error * kp
            self.state['int']['motor_r_v'] = front_rear_error * kp
        else:
            self.state['int']['motor_l_v'] = front_rear_error * kp
            self.state['int']['motor_r_v'] = left_right_error * kp

        self.set_motor_v()

    def clear_ahead(self, step_status, world_props, args):
        if self.state['int']['prox_s'].last_read[4][0] > 3:
            step_status['complete'] = True
            lg.message(logging.INFO, 'Clear ahead event complete')
            return

        self.state['int']['motor_l_v'] = -0.01
        self.state['int']['motor_r_v'] = 0.01

        self.set_motor_v()

    def random_wander(self, step_status, world_props, args):
        # Save start distance and time to internal state
        if self.state['int']['rw_status']['complete'] is None:
            self.state['int']['rw_status']['start_m'] = self.get_distance()
            self.state['int']['rw_status']['start_t'] = time.time()
            self.state['int']['rw_status']['complete'] = False

        # Check if move in progress is near HP doorway by checking distance from current location  to HP centre
        if self.state['int']['rw_move_status']['complete'] is False \
                and (self.h.within_dist(self.state['ext']['waypoints']['HP Centre'], self.state['ext']['abs_pos_n'],
                                        dist_threshold=2.3)) \
                and 'degrees' in self.state['int']['rw_turn_status']['args']:
            # Stop robot as close to doorway
            self.stop(self.state['int']['rw_move_status'], world_props, {})

            # Reset, ready to start a new turn & move combo
            self.state['int']['rw_turn_status']['complete'] = None
            self.state['int']['rw_move_status']['complete'] = None

        # Get sensor distances, can now use maximum range when pinging scene
        clip_distance = self.state['int']['prox_s'].max_detection_dist
        sensors = [s[0] if s[0] < clip_distance else clip_distance for s in self.state['int']['prox_s'].last_read]

        # Prepare a new random turn
        if self.state['int']['rw_turn_status']['complete'] is None:
            self.state['int']['rw_status']['turns_count'] += 1

            # Get ranges for left and right sensors
            sensor_index = [0, 7]
            sensor_list = list(itemgetter(*sensor_index)(sensors))

            # Set bearing direction southerly if robot within proximity of HP Centre waypoint
            if (self.h.within_dist(self.state['ext']['waypoints']['HP Centre'], self.state['ext']['abs_pos_n'],
                                   dist_threshold=2.3)):
                random_degree = random.randrange(120, 240, 2)
                self.state['int']['rw_turn_status']['degrees'] = random_degree

                # Prepare args for turn action
                self.state['int']['rw_turn_status']['args'] = {
                    'fixed': self.state['int']['rw_turn_status']['degrees'],
                    'radius_threshold': 0.3}
            else:
                random_degree = random.randrange(2, 180, 2)  # Otherwise wider arc for stochastic turn
                if sensor_list[0] > sensor_list[1]:
                    random_degree *= -1  # Turn ccw

                self.state['int']['rw_turn_status']['degrees'] = random_degree

                # Prepare args for turn action
                self.state['int']['rw_turn_status']['args'] = {'degrees': self.state['int']['rw_turn_status']['degrees'],
                                                               'radius_threshold': 0.3}

        # Execute turn action until it is signalled complete internally within method
        if self.state['int']['rw_turn_status']['complete'] is not True:
            self.turn(self.state['int']['rw_turn_status'], world_props, self.state['int']['rw_turn_status']['args'])
            return

        # Prepare move action
        if self.state['int']['rw_move_status']['complete'] is None:
            self.state['int']['rw_move_status']['start_m'] = self.get_distance()  # Current dist travelled as start dist

            # Prepare args for move action
            direction = 1
            velocity = 0.26
            if 'fixed' in self.state['int']['rw_turn_status']['args']:
                velocity = 0.32
            self.state['int']['rw_move_status']['args'] = {'velocity': velocity, 'distm': 20,
                                                           'robot_dir_travel': direction}

        # Execute move action until it is signalled complete internally within method
        self.move(self.state['int']['rw_move_status'], world_props, self.state['int']['rw_move_status']['args'])

        # If move is complete then reset status for turn & move combo, ready for another cycle
        if self.state['int']['rw_move_status']['complete']:
            self.state['int']['rw_turn_status']['complete'] = None
            self.state['int']['rw_move_status']['complete'] = None

    def path_plan(self, step_status, world_props, args):
        self.state['ext']['mapper'].path_planner()
        step_status['complete'] = True

    def go_home(self, step_status, world_props, args):
        # On initiating return home, get pre-planned route and set task start metrics
        if step_status['complete'] is None:
            self.state['int']['gh_status']['pose'] = self.state['ext']['abs_pos_n'].copy()
            if self.state['int']['gh_status']['pose'][0] == 0.0 and self.state['int']['gh_status']['pose'][1] == 0.0:
                return
            self.state['ext']['mapper'].load_planned_route_from_disk()
            self.state['int']['gh_status']['start_m'] = self.get_distance()
            self.state['int']['gh_status']['start_t'] = time.time()
            step_status['complete'] = False

        if step_status['complete'] is False:
            if len(self.state['ext']['mapper'].planned_route) == 0:
                step_status['complete'] = True
                return

        # Prepare a new turn to next route point
        if self.state['int']['gh_turn_status']['complete'] is None:
            self.state['int']['gh_status']['pose'] = self.state['ext']['abs_pos_n'].copy()
            self.state['int']['gh_status']['steps_count'] += 1
            self.state['int']['gh_status']['route_x'] = self.state['ext']['mapper'].planned_route[0][1]
            self.state['int']['gh_status']['route_y'] = self.state['ext']['mapper'].planned_route[0][0]

            # Transform world to grid reference units, as route uses grid system from occupancy map
            self.state['int']['gh_status']['pose'][0] = (1500 / 2) + \
                                                        (self.state['int']['gh_status']['pose'][0] * -100) * 1
            self.state['int']['gh_status']['pose'][1] = (1500 / 2) + \
                                                        (self.state['int']['gh_status']['pose'][1] * -100) * 1

            # Calculate angle from robot bearing to current planned route point
            delta_y = self.state['int']['gh_status']['route_y'] - self.state['int']['gh_status']['pose'][0]
            delta_x = self.state['int']['gh_status']['route_x'] - self.state['int']['gh_status']['pose'][1]
            angle_robot_to_route_point = math.degrees(math.atan2(delta_y, delta_x))
            angle_robot_to_route_point += 90
            if angle_robot_to_route_point > 360:
                angle_robot_to_route_point -= 360

            # Set arguments for turn action
            self.state['int']['gh_turn_status']['args'] = {'fixed': angle_robot_to_route_point, 'radius_threshold': 1}

        if self.state['int']['gh_turn_status']['complete'] is not True:
            self.turn(self.state['int']['gh_turn_status'], world_props, self.state['int']['gh_turn_status']['args'])
            return

        # Turn element is complete so can prepare move directive
        if self.state['int']['gh_move_status']['complete'] is None:
            # Stop previous turn
            self.stop(self.state['int']['gh_turn_status'], world_props, {})

            # Calculate distance in metres from robot to current planned route point
            dist = (math.hypot(self.state['int']['gh_status']['pose'][1] - self.state['int']['gh_status']['route_x'],
                               self.state['int']['gh_status']['pose'][0] - self.state['int']['gh_status']['route_y']) / 100)

            # Set arguments for move action
            self.state['int']['gh_move_status']['args'] = {'velocity': 0.15, 'distm': dist, 'robot_dir_travel': 1}
            self.state['int']['gh_move_status']['start_m'] = self.get_distance()

        # Issue move directive
        if self.state['int']['gh_move_status']['complete'] is not True:
            self.move(self.state['int']['gh_move_status'], world_props, self.state['int']['gh_move_status']['args'])
            return

        self.stop(self.state['int']['gh_move_status'], world_props, {})
        self.state['int']['gh_turn_status']['complete'] = None
        self.state['int']['gh_move_status']['complete'] = None

        # Remove n route points from the route as too granular for robot movement
        n = 20
        del self.state['ext']['mapper'].planned_route[:n]

    def wall_follow_pid(self, min_dist, max_dist, gain, motor_index):
        """
        This wall follow algorithm uses a PID controller approach
        """
        # Get PV
        distance = self.prox_dist_dir_travel()

        # e(t) is SP - PV
        error = min_dist - distance

        # Accumulate navigation error
        self.state['int']['err_corr_count'] += abs(error)

        # Integral short-term memory
        self.state['int']['pid_sum_error_short_term'].insert(0, error)  # Add error to top of list
        if len(self.state['int']['pid_sum_error_short_term']) > 100:
            self.state['int']['pid_sum_error_short_term'].pop()  # Remove last item

        # Traditional integral implementation
        #self.state['int']['pid_sum_error'] += error

        p = 0.00002 * error

        #i = 0
        #i = self.state['int']['pid']['ki'] * self.pid_integral_clamp(self.state['int']['pid_sum_error'])
        i = self.state['int']['pid']['ki'] * self.pid_integral_clamp(sum(self.state['int']['pid_sum_error_short_term']))

        #d = 0
        d = self.state['int']['pid']['kd'] * (error - self.state['int']['pid_prev_error'])

        output = p + i + d
        self.state['int']['error_history'].append((self.state['int']['cycle_i'], output))  # Supports telemetry

        #baseline_v = 0.39  # Fixed baseline speed
        baseline_v = 0.47 - math.sqrt(abs(p))  # Dynamic baseline speed with P control gain

        if error < 0:
            self.state['int']['motor_l_v'] = baseline_v + abs(output)
            self.state['int']['motor_r_v'] = baseline_v - abs(output)
        else:
            self.state['int']['motor_l_v'] = baseline_v - abs(output)
            self.state['int']['motor_r_v'] = baseline_v + abs(output)

        self.state['int']['pid_prev_error'] = error  # Log error supporting derivative

    def pid_integral_clamp(self, integral):
        if integral > self.state['int']['pid_integral_clamp']:
            return self.state['int']['pid_integral_clamp']
        elif integral < -self.state['int']['pid_integral_clamp']:
            return -self.state['int']['pid_integral_clamp']
        else:
            return integral

    def set_joint_pos(self):
        """
        Set starting positions for revolute joints
        """
        for m in self.state['int']['motors']:
            res, current_pos = vrep.simxGetJointPosition(self.h.client_id, m, vrep.simx_opmode_streaming)
            self.state['int']['jpos'][str(self.state['int']['motors'][m] + '_prev')] = current_pos
            self.state['int']['jpos'][str(self.state['int']['motors'][m] + '_total')] = current_pos

    def update_state_odometry(self):
        """
        Update odometry state by reading the motor revolute joint positions
        """
        for m in self.state['int']['motors']:
            self.update_joint_pos(m)

    def update_joint_pos(self, handle):
        """
        Update motor revolute joint positions. Determine rotation amount since last read
        """

        res, current_pos = vrep.simxGetJointPosition(self.h.client_id, handle, vrep.simx_opmode_streaming)
        difference_pos = current_pos - self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_prev')]

        self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_prev')] = current_pos

        difference_pos = self.get_pos_ang(difference_pos)
        self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_total')] += difference_pos

        self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_turns')] = \
            self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_total')] / (math.pi * 2)

        self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_dist')] = \
            self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_turns')] * self.props['wheel_circ_m']

    @staticmethod
    def get_pos_ang(pos):
        """
        Get revolute position as turn angle
        """
        if pos >= 0:
            pos = math.fmod(pos + math.pi, 2 * math.pi) - math.pi
        else:
            pos = math.fmod(pos - math.pi, 2 * math.pi) + math.pi
        return pos



