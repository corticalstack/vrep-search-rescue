import logging
import logger as lg
import vrep
import math
import numpy as np


class Sensor:
    """
    Implements sensors supporting perception of environment
    """
    def __init__(self, client_id, handle=None, sensor_array=None):
        lg.message(logging.INFO, 'Initialising ' + self.__class__.__name__)
        self.client_id = client_id
        self.sensor_array = sensor_array
        self.last_read = None


class ProximitySensor(Sensor):
    """
    Implements and manages a proximity sensor array
    """
    def __init__(self, client_id, handle=None, sensor_array=None):
        Sensor.__init__(self, client_id, sensor_array=sensor_array)
        self.max_detection_dist = 4.0
        self.last_read = self.read(vrep.simx_opmode_streaming)

        self.sensor_angle = {0: 1.57079,
                             1: 0.872665,
                             2: 0.523599,
                             3: 0.174533,
                             4: -0.174533,
                             5: -0.523599,
                             6:-0.872665,
                             7: -1.57079,
                             8: -1.5708,
                             9: -2.26893,
                             10: -2.61799,
                             11: -2.96706,
                             12: 2.96706,
                             13: 2.61799,
                             14: 2.26893,
                             15: 1.5708}

    def read(self, mode, handle=None, sensor_array=None):
        """
        Update value for each sensor in array
        """
        self.last_read = []
        for i, s in enumerate(self.sensor_array):
            res, ds, dp, doh, dsnv = vrep.simxReadProximitySensor(self.client_id, s, mode)
            distance = math.sqrt(sum(i**2 for i in dp))
            if not ds:
                distance = self.max_detection_dist  # If bad read set distance to default max range
            self.last_read.append((distance, ds))
        return self.last_read


class Compass(Sensor):
    """
    Implements a magnetic compass virtually, using V-REP object orientation angles
    """
    def __init__(self, client_id, handle=None, sensor_array=None):
        Sensor.__init__(self, client_id, handle=handle)
        self.to_bearing = None
        self.last_read_euler, self.last_read_mag_deg = self.read(vrep.simx_opmode_streaming, handle)

    def read(self, mode, handle=None):
        """
        Get absolute orientation of object to scene
        """
        res, bearing = vrep.simxGetObjectOrientation(self.client_id, handle, -1, mode)

        self.last_read_euler = round(bearing[2], 2)
        self.last_read_mag_deg = (180/math.pi) * bearing[2]
        if self.last_read_mag_deg > 0:
            self.last_read_mag_deg = 360 - self.last_read_mag_deg
        else:
            self.last_read_mag_deg *= -1

        return self.last_read_euler, self.last_read_mag_deg

    def set_to_bearing_add_deg(self, degrees):
        """
        Calculate target magnetic bearing
        """
        self.to_bearing = round(self.last_read_mag_deg + degrees, 0)
        if self.to_bearing >= 360:
            self.to_bearing -= 360
        elif self.to_bearing < 0:
            self.to_bearing += 360

    def set_to_bearing_fixed(self, degrees):
        """
        Set bearing to passed target magnetic bearing
        """
        self.to_bearing = round(degrees, 0)


class Beacon(Sensor):
    def __init__(self, client_id, handle=None, sensor_array=None):
        Sensor.__init__(self, client_id, handle=handle)
        res, self.handle = vrep.simxGetDistanceHandle(self.client_id, 'beacon', vrep.simx_opmode_blocking)
        self.last_read = self.read(vrep.simx_opmode_streaming, self.handle)

    def read(self, mode, handle=None):
        """
        Get absolute orientation of object to scene
        """
        res, self.last_read = vrep.simxReadDistance(self.client_id, self.handle, vrep.simx_opmode_streaming)
        return self.last_read
