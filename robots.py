import logging
import logger as lg
import vrep
import math
import time
from sensors import ProximitySensor, Compass, Beacon
from mapper import Mapper
import numpy as np
import matplotlib.pyplot as plt


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

        self.count = 0
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
                              'prox_min_dist_f': 0,
                              'prox_min_dist_r': 0,
                              'compass': None,
                              'err_corr_count': 0,
                              'cycle_i': 0,
                              'error_history': []},
                      'ext': {'abs_pos_s': None,
                              'abs_pos_n': None,
                              'abs_pos_all': [],
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
        distance_readings = [dr[0] for dr in self.state['int']['prox_s'].last_read[0:8]]
        self.state['int']['prox_min_dist_f'] = min(distance_readings)

        distance_readings = [dr[0] for dr in self.state['int']['prox_s'].last_read[8:16]]
        self.state['int']['prox_min_dist_r'] = min(distance_readings)

        if self.state['int']['prox_min_dist_f'] < min_distance:
            print('Min dist', self.state['int']['prox_min_dist_f'])
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
        if self.state['ext']['beacon'].last_read < 1:
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
        velocity = self.props['def_ms_v']  # Set default velocity

        if 'robot_dir_travel' in args:
            self.robot_dir_travel = args['robot_dir_travel']

        if 'velocity' in args:
            velocity = args['velocity']

        if step_status['complete'] is None:
            # Direction travel - forward = 1, reverse = -1
            self.state['int']['motor_l_v'] = velocity * self.robot_dir_travel
            self.state['int']['motor_r_v'] = velocity * self.robot_dir_travel
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
                self.state['int']['compass'].set_to_bearing(args['degrees'])
                lg.message(logging.DEBUG, 'Turn bearing from {} to {}'.format(self.state['int']['compass'].last_read,
                                                                              self.state['int']['compass'].to_bearing))
            step_status['complete'] = False

        kp = 0.2  # p-controller gain
        radius_threshold = 0.15
        error = (self.state['int']['compass'].to_bearing -
                 self.state['int']['compass'].last_read_mag_deg + 540) % 360 - 180

        if abs(error) < radius_threshold:
            step_status['complete'] = True
            lg.message(logging.INFO, 'Turn event complete')
            return

        if error < 0:
            if error < -30:  # Cap diff
                error = -30
            self.state['int']['motor_l_v'] = -0.003 * (error * kp)
            self.state['int']['motor_r_v'] = 0.003 * (error * kp)
        else:
            if error > 30:
                error = 30
            self.state['int']['motor_l_v'] = 0.003 * (error * 0.2)
            self.state['int']['motor_r_v'] = -0.003 * (error * 0.2)

        self.set_motor_v()

    def room_centre(self, step_status, world_props, args):
        from operator import itemgetter
        import statistics

        mdd = self.state['int']['prox_s'].max_detection_dist
        sensors = [s[0] if s[0] < mdd else mdd for s in self.state['int']['prox_s'].last_read]

        # Used sensors numbered in VREp/Pioneer specs as 1 (front-left), 4 (front), 8 (front right) and 12 (rear)
        sensor_index = [0, 3, 7, 11]

        # Get subset of sensor distances using specified sensor indexes
        sensor_list = list(itemgetter(*sensor_index)(sensors))
        sensor_list = np.clip(sensor_list, 0, 3)
        # e(t) is SP - PV
        #error = sensor_list[0] - (sum(sensor_list) / len(sensor_list))
        error = statistics.stdev(sensor_list)
        if len(set(sensor_list)) == 1 and sensor_list[0] != 1:
            print('list all the same')

        print('Sensor list ', sensor_list)
        print('Error ', error)

        error1 = sensor_list[0] - sensor_list[2]
        error2 = sensor_list[1] - sensor_list[3]
        if abs(error) < 0.2 and self.state['int']['cycle_i'] > 30:
            step_status['complete'] = True
            lg.message(logging.INFO, 'Wall Follow event complete')
            return

        baseline_v1 = error1 * 0.15
        baseline_v2 = error2 * 0.15
        print('Baseline v1 ', baseline_v1)
        print('Baseline v2 ', baseline_v2)

        error3 = baseline_v1 - baseline_v2 * 2
        print('Error 3 ', error3)
        if error1 < 0:
            self.state['int']['motor_l_v'] = baseline_v1
            self.state['int']['motor_r_v'] = baseline_v2
        else:
            self.state['int']['motor_l_v'] = baseline_v2
            self.state['int']['motor_r_v'] = baseline_v1
        self.set_motor_v()
        return

        p = 0.12 * error
        p2 = 2.5
        output = p
        baseline_v = 0.01 - math.sqrt(abs(p))  # Dynamic baseline speed with P control gain
        print('Baseline ', baseline_v)
        if sensor_list[0] < sensor_list[2]:
            self.state['int']['motor_l_v'] = baseline_v + abs(output * p2)
            self.state['int']['motor_r_v'] = baseline_v - abs(output * p2)
        else:
            self.state['int']['motor_l_v'] = baseline_v - abs(output * p2)
            self.state['int']['motor_r_v'] = baseline_v + abs(output * p2)

        self.set_motor_v()

    def clear_ahead(self, step_status, world_props, args):
        from operator import itemgetter
        import statistics

        sensors = [s[1] if s[0] == 6 else 1 for s in self.state['int']['prox_s'].last_read]
        sensor_index = [0, 3, 7, 11]
        sensor_list = list(itemgetter(*sensor_index)(sensors))

        print("Ahead ", sensor_list[1])
        # e(t) is SP - PV
        #error = sensor_list[0] - (sum(sensor_list) / len(sensor_list))
        error = statistics.stdev(sensor_list)
        if self.state['int']['prox_s'].last_read[3][1] is False:
            step_status['complete'] = True
            lg.message(logging.INFO, 'Wall Follow event complete')
            return

        self.state['int']['motor_l_v'] = -0.01
        self.state['int']['motor_r_v'] = 0.01

        self.set_motor_v()

    def search_beacon(self, min_dist, max_dist, gain, motor_index):
        """
        This wall follow algorithm uses a PID controller approach
        """
        # Get PV

        attractice_gain = 5
        repulsive_gain = 5

        ensors = [s[0] if s[0] < 6 else 1 for s in self.state['int']['prox_s'].last_read]
        sensor_index = [0, 3, 7, 11]
        sensor_list = list(itemgetter(*sensor_index)(sensors))

        # Remember sensors not in order JP - can I resolve this?
        # Calculate repulse
        # repulse_left = 0
        # for s in sensor_list:
        #     repulse_left = repulse_left + (repulse_gain / left sensor)
        #
        # repulse_right = 0
        # for s in sensor_list:
        #     repulse_right = repulse_right + (repulse_gain / right_sensor)




        # Calculate gain





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



