import rospy
import tf
from geometry_msgs.msg import PoseStamped

class PoseManager(object):
    def __init__(self, n, robot_sufij='/vrpn_client_node/modquad', start_id=0):
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
            topic_name = '{}{:02d}/pose'.format(
                        self.robot_sufij, i + self._start_id + 1,
                    )
            rospy.Subscriber(topic_name, PoseStamped, self._callback_pose)
            rospy.loginfo("Subscribe to {}".format(topic_name))

    def _callback_pose(self, pose):
        # state vector
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z

        # orientation
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        ### Extract id from topic name
        topic = pose._connection_header['topic']
        # extract robot id from the topic name
        robot_id = int(topic[len(self.robot_sufij):len(self.robot_sufij) + 2]) - 1 - self._start_id

        self._locations[robot_id] = (x, y, z)  # store the recent location
        self._orientations[robot_id] = euler
        self._quats[robot_id] = quaternion
        self._poses[robot_id] = pose.pose

    def get_new_states(self):
        """
        Returns list of full state for all modules being tracked
        """
        return [self.get_new_state(mid) for mid in range(self.n)]

    def get_new_state(self, rid):
        """
        Return the current state for a specific module
        """
        if self._locations[0] == None:
            return [0,0,0,0,0,0,0,0,0,0,0,0,0]
        # :param state_vector: 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
        return [self._locations[rid][0], self._locations[rid][1], self._locations[rid][2], 
                self._quats[rid][0], self._quats[rid][1], self._quats[rid][2], self._quats[rid][3]]

    def get_poses(self):
        return self._poses

    def get_locations(self):
        """
        :return: robot locations [(x1,y1,z1),...,(xn, yn,zn)] 
        """
        return self._locations

    def get_robot_loc(self, r2):
        return self.get_locations()[r2]

    def get_orientations(self):
        return self._orientations
