#!/usr/bin/env python

import tf
import sys
import rospy
import moveit_commander

import moveit_msgs.msg
import geometry_msgs.msg

from robotiq_85_msgs.msg import GripperCmd 
from ar_track_alvar_msgs.msg import AlvarMarkers

class URControl():
    def __init__(self):

        rospy.init_node('ur_control')
        self.target_flag = False
        self.target_pose = geometry_msgs.msg.Pose()
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_pose_cb)
        pub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)

        moveit_commander.roscpp_initialize(sys.argv)   
        arm_group = moveit_commander.MoveGroupCommander("ur5_arm")

        print 'waiting for marker...'
        while self.target_flag is False:
            rospy.sleep(0.05)

        print 'got marker pose, planning...'
        arm_group.set_pose_target(self.target_pose)
        plan1 = arm_group.plan()
        print plan1
        arm_group.execute(plan1)


        rospy.sleep(5.0)
        while not rospy.is_shutdown():
            gripper_cmd = GripperCmd()
            gripper_cmd.position = 0
            gripper_cmd.speed = 0.2
            gripper_cmd.force = 2.0
            pub.publish(gripper_cmd)

        moveit_commander.roscpp_shutdown()

    def ar_pose_cb(self, msg):
        if len(msg.markers) > 0:
            self.target_flag = True
            self.target_pose.position.x = msg.markers[0].pose.pose.position.x+0.1
            self.target_pose.position.y = msg.markers[0].pose.pose.position.y+0.10
            self.target_pose.position.z = 0.08
            
            q = tf.transformations.quaternion_from_euler(-1.57, 1.57, 0)
            self.target_pose.orientation.x = q[0]
            self.target_pose.orientation.y = q[1]
            self.target_pose.orientation.z = q[2]
            self.target_pose.orientation.w = q[3]
    
if __name__ == "__main__":
    URControl()


