import logging
import logger as lg
import vrep
import math


class Node(object):
    """
    Class for building N-ary tree of V-REP object with descendants
    """
    def __init__(self, data):
        self.data = data
        self.children = []

    def add_child(self, obj):
        """
        Add descendant
        """
        self.children.append(obj)


class Helper:
    """
    Generic helper class supporting simulation in V-REP
    """
    def __init__(self):
        lg.message(logging.INFO, 'Initialising ' + self.__class__.__name__)
        self.object_types = {0: 'shape',
                             1: 'joint',
                             2: 'graph',
                             3: 'camera',
                             4: 'dummy',
                             5: 'proximitysensor',
                             6: 'reserved1',
                             7: 'reserved2',
                             8: 'path_type',
                             9: 'visionsensor',
                             10: 'volume',
                             11: 'mill',
                             12: 'forcesensor',
                             13: 'light',
                             14: 'mirror'}

        self.api_server = {'connectionAddress': '127.0.0.1',
                           'connectionPort': 19999,
                           'waitUntilConnected': True,
                           'doNotReconnectOnceDisconnected': True,
                           'timeOutInMs': 5000,
                           'commThreadCycleInMs': 5}

        self.client_id = -1
        self.connected = self.connect_client()
        self.object_tree = {}

    def connect_client(self):
        """
        Connect this client to V-REP server
        """
        self.client_id = vrep.simxStart(self.api_server['connectionAddress'],
                                        self.api_server['connectionPort'],
                                        self.api_server['waitUntilConnected'],
                                        self.api_server['doNotReconnectOnceDisconnected'],
                                        self.api_server['timeOutInMs'],
                                        self.api_server['commThreadCycleInMs'])

        if self.client_id != -1:  # check if client connection successful
            lg.message(logging.INFO, 'Connected to V-REP')
            return True
        else:
            lg.message(logging.INFO, 'Failed connecting to V-REP')
            return False

    def disconnect_client(self):
        """
        Disconnect this client from the V-REP server
        """
        # Close the connection to VREP
        vrep.simxStopSimulation(self.client_id, vrep.simx_opmode_blocking)
        vrep.simxFinish(self.client_id)
        lg.message(logging.INFO, 'Disconnected from V-REP')

    def get_object_handle_by_name(self, object_name):
        """
        Return V-REP object handle for a given object name
        """
        lg.message(logging.DEBUG, 'Getting V-REP object handle for ' + object_name)
        error_code, handle = vrep.simxGetObjectHandle(self.client_id, object_name, vrep.simx_opmode_oneshot_wait)
        if error_code == 0:
            return handle
        else:
            return -1

    def get_object_tree(self, handle, index):
        """
        Build N-ary tree for V-REP object
       """
        if index == 0:
            self.object_tree[handle] = Node(handle)
        child_handle = self.get_object_child(handle, index)

        if child_handle != -1:
            self.object_tree[handle].add_child(child_handle)
            index += 1
            self.get_object_tree(handle, index)
        else:
            for c in self.object_tree[handle].children:
                self.get_object_tree(c, 0)

    def get_object_child(self, handle, index):
        """
        Get V-REP handle for child object
        """
        error_code, child_handle = vrep.simxGetObjectChild(self.client_id, handle, index, vrep.simx_opmode_blocking)
        return child_handle

    @staticmethod
    def get_object_data(client_id, object_type, data_type):
        """
        Get V-REP object group data by type
        """
        res, handles, int_data, float_data, string_data = vrep.simxGetObjectGroupData(client_id, object_type, data_type,
                                                                                      vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            return handles, int_data, float_data, string_data

    def get_objects(self):
        """
        Get V-REP object data
        """
        object_names = self.get_object_data(self.client_id, vrep.sim_appobj_object_type, 0)

        # Get object types
        object_types = self.get_object_data(self.client_id, vrep.sim_appobj_object_type, 1)

        # Get parent object handles
        object_parent_handles = self.get_object_data(self.client_id, vrep.sim_appobj_object_type, 2)

        objects = list(zip(object_names[0], object_names[3], object_types[1], object_parent_handles[1]))

        full_scene_object_list = []
        simple_object_list = {}
        for object_handle, object_name, object_type, object_parent_handle in objects:
            full_scene_object_list.append((object_handle, object_name, object_type, self.object_types[object_type],
                                           object_parent_handle))
            simple_object_list[object_handle] = object_name

        return full_scene_object_list, simple_object_list

    @staticmethod
    def ms_to_radss(metres_per_s, radius):
        """
        Convert metres per second to radius per second
        """
        return metres_per_s / radius
        #return (metres_per_s * 2) / (radius * 2)

    def within_dist(self, point_a, point_b, dist_threshold=0.08):
        """
        Determine if point A within distance threshold of point B
        """
        distance = self.get_euclidean_distance(point_a, point_b)
        lg.message(logging.DEBUG, 'Distance between point A and B is ' + str(distance))
        if distance <= dist_threshold:
            return True
        else:
            return False

    @staticmethod
    def get_euclidean_distance(point_a, point_b):
        """
        Euclidean distance between point A and B
        """
        return math.sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)
