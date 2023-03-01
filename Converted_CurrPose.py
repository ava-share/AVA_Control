import rospy
#not sure where the /geometry_msgs is going to be passed from
from geometry_msgs.msg import PoseStamped, Pose

def current_pose_callback(msg):
    # init the pose message
    pose = Pose()
    pose.position.x = msg.pose.position.x
    pose.position.y = msg.pose.position.y
    pose.position.z = msg.pose.position.z
    pose.orientation.x = msg.pose.orientation.x
    pose.orientation.y = msg.pose.orientation.y
    pose.orientation.z = msg.pose.orientation.z
    pose.orientation.w = msg.pose.orientation.w

    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    # Publish posestamped message
    pub.publish(pose_stamped)

if __name__ == '__main__':
    rospy.init_node('current_pose_converter')
    rospy.Subscriber('/current_pose', PoseStamped, current_pose_callback)
    pub = rospy.Publisher('/converted_pose', PoseStamped, queue_size=10)

    rospy.spin()