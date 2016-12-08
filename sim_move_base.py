#!/usr/bin/env python
import roslib; roslib.load_manifest('stage_ros')
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, Pose2D
from tf import TransformListener

pose_pub = None
tf_listener = None

def callback(msg):
    global pose_pub
    global tf_listener
    
    try:
        rospy.loginfo("Received at goal message!")
        rospy.loginfo("Timestamp: " + str(msg.header.stamp))
        rospy.loginfo("frame_id: " + str(msg.header.frame_id))

        # Copying for simplicity
        msg = tf_listener.transformPose("map", msg)
        position = msg.pose.position
        quat = msg.pose.orientation
        rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
        rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))

        # Also print Roll, Pitch, Yaw
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        rospy.loginfo("Euler Angles: %s"%str(euler))
        
        pos = Pose2D()
        pos.x = position.x
        pos.y = position.y
        pos.theta = euler[2]
        pose_pub.publish(pos)
    except:
        rospy.loginfo("error")

def listener():
    global pose_pub
    global tf_listener
    rospy.init_node('goal_listener', anonymous=True)
    tf_listener = TransformListener()
    pose_pub = rospy.Publisher("/pose", Pose2D, queue_size=1)
    rospy.Subscriber("/sim_move_base_simple/goal", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
