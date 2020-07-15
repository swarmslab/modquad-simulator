import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from transforms3d import euler as trans

def publish_pos(x, pub):
    #TODO Remove the redundancy of using Odometry here
    #  as we don't use most of the vector
    """
    Publish absolute position in world so that docking detector 
    correctly identifies docking actions
    :param x: Vector of values (state)
    :param pub: The publisher for a specific modquad module
    """
    # Roll pitch yaw trust
    odom = Odometry()
    odom.pose.pose.position.x = x[0]
    odom.pose.pose.position.y = x[1]
    odom.pose.pose.position.z = x[2]

    # Velocities
    odom.twist.twist.linear.x = 0.0 # x[3]
    odom.twist.twist.linear.y = 0.0 # x[4]
    odom.twist.twist.linear.z = 0.0 # x[5]

    # Orientation
    odom.pose.pose.orientation.x = 0.0 # x[6]
    odom.pose.pose.orientation.y = 0.0 # x[7]
    odom.pose.pose.orientation.z = 0.0 # x[8]
    odom.pose.pose.orientation.w = 0.0 # x[9]

    odom.pose.pose.orientation.x = 0.0 # x[7]
    odom.pose.pose.orientation.y = 0.0 # x[8]
    odom.pose.pose.orientation.z = 0.0 # x[9]
    odom.pose.pose.orientation.w = 0.0 # x[6]

    # Angular velocities
    odom.twist.twist.angular.x = 0.0 # x[10]
    odom.twist.twist.angular.y = 0.0 # x[11]
    odom.twist.twist.angular.z = 0.0 # x[12]

    odom.child_frame_id = 'modquad'
    odom.header.frame_id = 'world'

    pub.publish(odom)

def publish_odom(x, pub):
    """
    Convert quad state into an odometry message and publish it.
    :param x: 
    :param pub: 
    """
    # Roll pitch yaw trust
    odom = Odometry()
    odom.pose.pose.position.x = x[0]
    odom.pose.pose.position.y = x[1]
    odom.pose.pose.position.z = x[2]

    # Velocities
    odom.twist.twist.linear.x = x[3]
    odom.twist.twist.linear.y = x[4]
    odom.twist.twist.linear.z = x[5]

    # Orientation
    odom.pose.pose.orientation.x = x[6]
    odom.pose.pose.orientation.y = x[7]
    odom.pose.pose.orientation.z = x[8]
    odom.pose.pose.orientation.w = x[9]

    odom.pose.pose.orientation.x = x[7]
    odom.pose.pose.orientation.y = x[8]
    odom.pose.pose.orientation.z = x[9]
    odom.pose.pose.orientation.w = x[6]


    # Angular velocities
    odom.twist.twist.angular.x = x[10]
    odom.twist.twist.angular.y = x[11]
    odom.twist.twist.angular.z = x[12]

    odom.child_frame_id = 'modquad'
    odom.header.frame_id = 'world'

    pub.publish(odom)


def publish_odom_relative(structure_x, structure_y, id_frame, relative_to_frame,pub):
    """
    Convert quad state into an odometry message and publish it.
    :param x:
    :param pub:
    """
    # Roll pitch yaw trust
    odom = Odometry()
    odom.pose.pose.position.x = structure_x
    odom.pose.pose.position.y = structure_y
    odom.pose.pose.position.z = 0

    # Velocities
    odom.twist.twist.linear.x = 0.
    odom.twist.twist.linear.y = 0.
    odom.twist.twist.linear.z = 0.

    # Orientation
    odom.pose.pose.orientation.x = 0
    odom.pose.pose.orientation.y = 0
    odom.pose.pose.orientation.z = 0
    odom.pose.pose.orientation.w = 1

    # Angular velocities
    odom.twist.twist.angular.x = 0.
    odom.twist.twist.angular.y = 0.
    odom.twist.twist.angular.z = 0.

    odom.child_frame_id = id_frame
    odom.header.frame_id = relative_to_frame

    pub.publish(odom)


from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion


def publish_transform_stamped(model_name, x, pub):
    ts = TransformStamped()
    ts.child_frame_id = model_name

    # Header
    ts.header.stamp = rospy.Time.now()
    ts.header.frame_id = "world"

    # Translation
    translation = Vector3()
    translation.x = x[0]
    translation.y = x[1]
    translation.z = x[2]

    # Rotation
    quat = Quaternion()
    quat.x = x[6]
    quat.y = x[7]
    quat.z = x[8]
    quat.w = x[9]

    # Message
    transform = Transform()
    transform.translation = translation
    transform.rotation = quat
    ts.transform = transform

    # Publish a transform stamped message
    pub.sendTransform(ts)


def publish_transform_stamped_relative(model_name, parent_name, struct_x, struct_y, pub):
    ts = TransformStamped()
    ts.child_frame_id = model_name

    # Header
    ts.header.stamp = rospy.Time.now()
    ts.header.frame_id = parent_name

    # Translation
    translation = Vector3()
    translation.x = struct_x
    translation.y = struct_y
    translation.z = 0

    # Rotation
    quat = Quaternion()
    quat.x = 0
    quat.y = 0
    quat.z = 0
    quat.w = 1

    # Message
    transform = Transform()
    transform.translation = translation
    transform.rotation = quat
    ts.transform = transform

    # Publish a transform stamped message
    pub.sendTransform(ts)

def publish_acc(state_vec, lin_acc, pub):
    """
	Publish linear acceleration for a main module
    :param x: 
    :param pub: 
    """
    # Roll pitch yaw trust
    imu = Imu()

    imu.header.stamp = rospy.Time.now()
    #imu.child_frame_id = 'modquad'
    imu.header.frame_id = 'world'

	# Not using seq nums
    #imu_data.header.seq = seq

	# Not using orientation from this msg
    imu.orientation.x = state_vec[0]
    imu.orientation.y = state_vec[1]
    imu.orientation.z = state_vec[2]
    imu.orientation.w = state_vec[9]

	# Linear acceleration
    imu.linear_acceleration.x = lin_acc[0]
    imu.linear_acceleration.y = lin_acc[1]
    imu.linear_acceleration.z = lin_acc[2]
    imu.linear_acceleration_covariance[0] = -1

	# Not using ang vel
    imu.angular_velocity.x = 0
    imu.angular_velocity.y = 0
    imu.angular_velocity.z = 0
    imu.angular_velocity_covariance[0] = -1

    pub.publish(imu)
