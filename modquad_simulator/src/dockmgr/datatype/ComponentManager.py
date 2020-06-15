import networkx as nx
import numpy as np
import rospy
#from dock_manager.srv import ManualDock, ManualDockRequest
from std_msgs.msg import Int8MultiArray
from itertools import combinations
from modsim.datatype.structure import Structure

from modsim.util.docking import docking_array_to_graph, components_locations
from dockmgr.datatype.OdometryManager import OdometryManager

class ComponentManager(object):
    def __init__(self, n, dock_listener=None):
        self.n = n # Number of modules
        self.num_struc = n# Number of structures being managed initially = # mods
        # No components in the beginning, they are recognized after docking detector msg.
        # A component is a Structure object
        self.components = []  
        # {Component:locations (matrix n'x2)} with respect to the structure
        self.locations = [[0., 0.] for _ in range(n)]  
        self.dock_listener = dock_listener
        self.G = None

    def init_component(structure):
        """
        Mainly for testing. Allows for creation of structure that is already docked. 
        Used to test undocking.
        """
        self.components = [structure]
        self.num_struc = 1

    def subscribe(self, topic_name='/dockings'):
        # Subscribe to component listener
        rospy.Subscriber(topic_name, Int8MultiArray, self._component_listener)

    def _component_listener(self, comp_msg):
        """
        Receives the component message and storage it.
        :param comp_msg: components message from topic 
        """
        # Component array
        comp_array = np.array(comp_msg.data)
        # convert to graph
        self.G = docking_array_to_graph(comp_array, self.n)
        self.locations = components_locations(self.G, self.n)
        # For each detected component, compute its structure
        new_components = [sorted(c) for c in nx.connected_components(self.G)]

        # Notify new components
        if self.dock_listener is not None:
            for nc in new_components:
                # Validate if this component is a new component. Then notify the listener.
                if nc not in self.components:
                    self.dock_listener(nc)

        # update components
        self.components = new_components

    def get_neighbors(self, i):
        return self.G.edge[i].keys()

    def get_centriod(self, C):
        """
        Get the centroid of a component
        :param C: Component: vector of robot indexes. 
        """
        return np.mean(self.get_component_locations(C), axis=0)

    def get_component_locations(self, C):
        """
        Get component locations
        :rtype: array of locations
        """
        return np.array(self.locations)[C]

    def get_component_locations_centroid(self, C):
        """
        Get component locations based on the center of mass as origin.
        :rtype: array of locations
        """
        X = np.array(self.locations)[C]
        # average (center of mass)
        X[:, 0] -= np.mean(X[:, 0])
        X[:, 1] -= np.mean(X[:, 1])
        return X

    def component_with_robot(self, r1):
        """
        Return the component that contains robot r1.
        :param r1: robot id
        :return: component
        """
        for c in self.components:
            if r1 in c:
                return c

        return None

    def get_location_relative_centroid(self, r1):
        """
        Get location of the robot r1 relative to the centroid of the component
        :param r1: 
        """
        C = self.component_with_robot(r1)
        centriod = self.get_centriod(C)
        # robot location with respect to robot 1
        lloc1 = np.array(self.locations)[r1]
        return lloc1 - centriod

    def get_principal(self, r1):
        """
        Get the principal robot id which controls the component.
        :param r1: robot in the component
        """
        C = self.component_with_robot(r1)
        return min(C)

    def split_components(self, r1, r2):
        locations = np.array(self.locations)

        # Find the perpendicular line to separate cmpts from r1 and r2. E.g. r1|r2
        x1, x2 = locations[r1], locations[r2]
        # midpoint
        xm = (x2 + x1) / 2.
        dx = x2 - x1  # direction vector
        # slope of the perpendicular line
        s = [-dx[1], dx[0]]  # the perpendicular line goes from xm to xm+s

        ### Separate the components
        C1 = []  # Elements on one side of the perpendicular line
        C2 = []  # Elements on the other side perpendicular line

        # r1 belongs to component C. It is assumed that r2 belongs to the same component.
        C = self.component_with_robot(r1)

        for i, xi in zip(C, locations[C]):
            # position with respect to the mid point
            xim = [xi[0] - xm[0], xi[1] - xm[1]]

            # cross product
            cross = s[0] * xim[1] - s[1] * xim[0]
            if cross > 0:
                C2.append(i)
            else:
                C1.append(i)

        return C1, C2

    def manual_undocking_request(self, r1, r2):
        """
        Check all the breaking connections and manually notify the docking_detector to undock the implicated robots.
        :param r1: robot 1 (enumerating from 0 to n-1)
        :param r2: robot 2
        """
        Cr, Cl = self.split_components(r1, r2)

        n = self.n

        ### check the breaking connections
        manual_docking = []

        for i, j in combinations(range(n), 2):
            # if the edge connects both components
            if (i in Cr and j in Cl) or (j in Cr and i in Cl):
                # if self.G.has_edge(i,j):
                manual_docking.append(0)
            else:
                manual_docking.append(-1)

        ### Send to docking detector
        service_name = '/manual_dock'
        msg = ManualDockRequest()
        msg.attachments = manual_docking

        try:
            call_manual_dock = rospy.ServiceProxy(service_name, ManualDock)
            call_manual_dock(msg)
        except rospy.ServiceException as e:
            rospy.logerr("Service manual dock failed: {}".format(e))
