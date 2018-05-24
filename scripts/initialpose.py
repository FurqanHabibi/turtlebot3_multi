from math import pi
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

rospy.init_node("initialpose")
pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latched=True)
x = rospy.get_param("~x", 0)
y = rospy.get_param("~y", 0)
theta = rospy.get_param("~theta", 0)

pose = PoseWithCovarianceStamped()
pose.header.frame_id = "map"
pose.pose.pose.position.x=x
pose.pose.pose.position.y=y
pose.pose.pose.position.z=0
pose.pose.covariance[6*0+0] = 0.5 * 0.5
pose.pose.covariance[6*1+1] = 0.5 * 0.5
pose.pose.covariance[6*5+5] = pi/12.0 * pi/12.0
quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
pose.pose.orientation.x = quaternion[0]
pose.pose.orientation.y = quaternion[1]
pose.pose.orientation.z = quaternion[2]
pose.pose.orientation.w = quaternion[3]

pub.publish(pose)
rospy.spin()
