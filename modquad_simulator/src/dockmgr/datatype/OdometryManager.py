import rospy
import tf
from nav_msgs.msg import Odometry

class OdometryManager(object):
    def __init__(self, n, robot_sufij='/modquad', start_id=0):
        self.n = n
        self.robot_sufij = robot_sufij
        ### Odometry for all robots. Dic{topic: (x,y,z)}
        # self.states = {robot_sufij + '%02d/odom' % (i + 1): None for i in range(n)}
        self._locations = [None for _ in range(n)]
        self._velocities = [None for _ in range(n)]
        self._ang_velocities = [None for _ in range(n)]
        self._orientations = [None for _ in range(n)]

        self._quats = [None for _ in range(n)]

        self._poses = [None for _ in range(n)]
        self._twists = [None for _ in range(n)]

        self._start_id = start_id

    def subscribe(self):
        for i in range(self.n):
            # subscriber
            rospy.Subscriber(self.robot_sufij + '%02d/odom' % (i + self._start_id + 1), Odometry, self._callback_odom)
            rospy.loginfo("Subscribe to {}{}/odom".format(self.robot_sufij, i + self._start_id + 1))

    def _callback_odom(self, odom):
        # state vector
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z

        # orientation
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # Linear velocities
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        vz = odom.twist.twist.linear.z

        # Ang velocities
        ax = odom.twist.twist.angular.x
        ay = odom.twist.twist.angular.y
        az = odom.twist.twist.angular.z

        ### Extract id from topic name
        topic = odom._connection_header['topic']
        # extract robot id from the topic name
        robot_id = int(topic[len(self.robot_sufij):len(self.robot_sufij) + 2]) - 1 - self._start_id

        self._locations[robot_id] = (x, y, z)  # store the recent location
        self._orientations[robot_id] = euler
        self._quats[robot_id] = quaternion
        self._velocities[robot_id] = (vx, vy, vz)  # store the recent location
        self._ang_velocities[robot_id] = (ax, ay, az)  # store the recent location
        self._poses[robot_id] = odom.pose.pose
        self._twists[robot_id] = odom.twist.twist

    def get_new_state(self, rid):
        if self._locations[0] == None:
            return [0,0,0,0,0,0,0,0,0,0,0,0,0]
        # :param state_vector: 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
        return [self._locations[rid][0], self._locations[rid][1], self._locations[rid][2], 
                self._velocities[rid][0], self._velocities[rid][1], self._velocities[rid][2], 
                self._quats[rid][0], self._quats[rid][1], self._quats[rid][2], self._quats[rid][3], 
                self._ang_velocities[rid][0], self._ang_velocities[rid][1], self._ang_velocities[rid][2]] 

    def get_poses(self):
        return self._poses

    def get_twists(self):
        return self._twists

    def get_locations(self):
        """
        :return: robot locations [(x1,y1,z1),...,(xn, yn,zn)] 
        """
        return self._locations

    def get_velocities(self):
        return self._velocities

    def get_robot_loc(self, r2):
        return self.get_locations()[r2]

    def get_orientations(self):
        return self._orientations
