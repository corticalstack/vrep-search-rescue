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
        self.detection_points = []
        self.max_detection_dist = 6
        self.last_read = self.read(vrep.simx_opmode_streaming)

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
        self.last_read = self.read(vrep.simx_opmode_streaming, handle)

    def read(self, mode, handle=None):
        """
        Get absolute orientation of object to scene
        """
        res, bearing = vrep.simxGetObjectOrientation(self.client_id, handle, -1, mode)

        # Bearing to magnetic degree
        deg = math.degrees(bearing[2])
        if deg > 0:
            mag_deg = 360 - deg
        else:
            mag_deg = deg * -1

        #self.last_read = round(mag_deg, 0)
        self.last_read = round(bearing[2], 2)
        return self.last_read

    def set_to_bearing(self, degrees):
        """
        Calculate target magnetic bearing
        """
        self.to_bearing = (self.last_read + degrees)
        if self.to_bearing > 360:
            self.to_bearing -= 360
        elif self.to_bearing < 0:
            self.to_bearing += 360


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
