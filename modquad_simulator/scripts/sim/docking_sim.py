#!/usr/bin/env python
import networkx as nx
import numpy as np
import rospy
from modquad_simulator.srv import Dislocation
from std_msgs.msg import Int8MultiArray

from modsim.util.docking import docking_array_to_graph, components_locations

zero = np.array([0., 0.])
n = rospy.get_param('num_robots', 1)
old_components = [[i] for i in range(n)]
old_centroids = {str(sorted(c)): zero for c in old_components}  
# {Component:centroid=[0,0]}


def component_listener(comp_msg):
    global old_centroids, old_components
    # Component array
    comp_array = np.array(comp_msg.data)
    # convert to graph
    G = docking_array_to_graph(comp_array, n)
    locs = components_locations(G, n)

    # For each detected component, compute its structure
    new_components = [sorted(c) for c in nx.connected_components(G)]
    new_centroids = {}

    # Same components, do nothing
    if len(old_components) <= len(new_components):
        return

    for C in new_components:
        # compute new centroid
        new_centroid = np.mean(np.array(locs)[C], axis=0)
        # save it
        new_centroids[str(C)] = new_centroid

        # Check if component C is a new component
        if C not in old_components:
            ### old component
            i = min(C)
            # find the old component that has i
            for old_C in old_components:
                if i in old_C:
                    break
            # old centroid
            old_centroid = old_centroids[str(old_C)]
            # compute dislocation
            disloc = new_centroid - old_centroid
            print disloc

            ### Send dislocation to robot i.
            #robot_sufij = '/crazy'
            robot_sufij = '/modquad'
            srv_name = robot_sufij + '%02d/dislocate_robot' % (i + 1)
            rospy.wait_for_service(srv_name)

            try:
                dislocate_robot = rospy.ServiceProxy(srv_name, Dislocation)
                dislocate_robot(disloc[0], disloc[1])
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)

    # save as old
    old_components = new_components
    old_centroids = new_centroids


def simulate():
    rospy.init_node('docking_simulator', anonymous=True)

    # Subscribe to component listener
    rospy.Subscriber('/dockings', Int8MultiArray, component_listener)

    rospy.spin()


if __name__ == '__main__':
    simulate()
