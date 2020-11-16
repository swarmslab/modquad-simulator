import rospy
import tf
from nav_msgs.msg import Odometry

class WorldPosManager(object):
    def __init__(self, n, start_id=0, robot_sufij='/modquad'):
        self.n = n
        self.robot_sufij = robot_sufij
        self.start_id = start_id
        ### Position in world frame for all robots. Dic{topic: (x,y,z)}
        self._locations = [None for _ in range(n)]

    def subscribe(self):
        for i in range(self.n):
            # subscriber
            #rospy.Subscriber(self.robot_sufij + '%02d/world_pos' % (i + 1), Odometry, self._callback_odom)
            rospy.Subscriber(self.robot_sufij + '%02d/odom' % (i + self.start_id + 1), Odometry, self._callback_odom)
            rospy.loginfo('<WorldPosManager> subscribe to {}'.format(self.robot_sufij + '%02d/odom' % (i + self.start_id + 1)))

    def _callback_odom(self, odom):
        # state vector
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z

        ### Use topic name to get Robot ID
        topic = odom._connection_header['topic']

        # extract robot id from the topic name
        robot_id = int(topic[len(self.robot_sufij):len(self.robot_sufij) + 2]) - 1

        self._locations[robot_id - self.start_id] = (x, y, z)  # store the position in world frame

    def get_locations(self):
        """
        :return: robot locations [(x1,y1,z1),...,(xn, yn,zn)] 
        """
        return self._locations

    def get_robot_loc(self, r2):
        return self.get_locations()[r2]
