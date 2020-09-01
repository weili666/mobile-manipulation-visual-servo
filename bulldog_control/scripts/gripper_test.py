import rospy
from robotiq_85_msgs.msg import GripperCmd 

def talker():
    pub = rospy.Publisher("/gripper/cmd", GripperCmd, queue_size=10)
    rospy.init_node("taker", anonymous=True)

    if not rospy.is_shutdown():
        gripper_cmd = GripperCmd()
        gripper_cmd.position = 0.0
        gripper_cmd.speed = 0.2
        gripper_cmd.force = 2.0
        pub.publish(gripper_cmd)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInternalException:
        pass

