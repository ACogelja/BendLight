#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger, SetBool, SetBoolRequest

from sensor_msgs.msg import JointState, Joy

class Controller:

    def __init__(self):
        
        print("waiting for control services")
        rospy.wait_for_service('/open_gripper')
        rospy.wait_for_service('/close_gripper')
        

        self.open_gripper = rospy.ServiceProxy('/open_gripper', Trigger)
        self.close_gripper = rospy.ServiceProxy('/close_gripper', Trigger)
        
        rospy.Subscriber('joy', Joy, self.joy_callback)
        self.joystick_active = False

    def joy_callback(self,msg):

        if not self.joystick_active:
            return
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = "base_link"
        
        if msg.buttons[2]:  #'B' button
            self.open_gripper()
        elif msg.buttons[1]:  #'A' button
            self.close_gripper()

        self.twist_pub.publish(twist_msg) 


    def gripper(self):
        print("Taking control of robot")
        #self.enable_servo(SetBoolRequest(data=True))
        self.joystick_active = True
        try:
            while not rospy.is_shutdown():
                izlaz = input("Press q to exit controller mode: ")
                if izlaz.lower() == "q":
                    self.joystick_active = False
                    print("Exiting")
                    break
                rospy.sleep(0.1)
        except KeyboardInterrupt:
            self.joystick_active = False


if __name__ == "__main__":
    rospy.init_node('ur5_controller')
    rate = rospy.Rate(1)
    
    process = Controller()
    process.gripper()
    rospy.spin()