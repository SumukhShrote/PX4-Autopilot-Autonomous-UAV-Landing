#!/usr/bin/python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
import time

class OffboardController:
    def __init__(self):
        rospy.init_node('offboard_controller')
        
        # Publishers and subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.target_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # Service proxies
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # State and initial position
        self.current_state = State()
        self.pose = PoseStamped()
        self.pose.pose.position.x = 1.8
        self.pose.pose.position.y = 2.0
        self.pose.pose.position.z = 3.5

        # Wait for FCU connection
        rate = rospy.Rate(20)  # 20 Hz publishing rate
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("Waiting for FCU connection...")
            rospy.sleep(1)
        
        # Publish initial setpoints
        for _ in range(100):
            self.target_pub.publish(self.pose)
            rate.sleep()

        # Arm and set to offboard mode
        self.set_offboard_mode()
        self.arm_drone()

        # Begin sending position commands
        self.control_loop(rate)

    def state_callback(self, msg):
        self.current_state = msg

    def arm_drone(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            response = self.arming_client(value=True)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def set_offboard_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            response = self.set_mode_client(custom_mode="OFFBOARD")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def control_loop(self, rate):
        last_request = rospy.Time.now()
        while not rospy.is_shutdown():
            # Re-check and re-set offboard mode every 5 seconds if not set
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                self.set_offboard_mode()
                last_request = rospy.Time.now()
            
            # Re-arm if needed every 5 seconds
            if not self.current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                self.arm_drone()
                last_request = rospy.Time.now()
            
            # Publish position continuously
            self.target_pub.publish(self.pose)
            rate.sleep()

if __name__ == '__main__':
    OffboardController()