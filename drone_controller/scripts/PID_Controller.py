#!/usr/bin/python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import time

class OffboardController:
    def __init__(self):
        rospy.init_node('offboard_controller')
        
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.target_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.error_sub = rospy.Subscriber('/yolo/center_error', Float32MultiArray, self.error_callback)
        
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        # Initialize state and target position
        self.current_state = State()
        self.pose = PoseStamped()
        self.pose.pose.position.x = 1.8
        self.pose.pose.position.y = 2.0
        self.pose.pose.position.z = 3.5
        self.error_x = 0
        self.error_y = 0
        self.threshold = 20 
        
        # Initialize error variables and PID gains
        self.Kp = -0.000001
        self.Ki = -0.0000001
        self.Kd = -0.0005
        self.integral_x = 0
        self.integral_y = 0
        self.prev_error_x = 0
        self.prev_error_y = 0
        
        
        rate = rospy.Rate(20)  
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("Waiting for FCU connection...")
            rospy.sleep(1)
        
        # Publish initial setpoints to establish Offboard mode
        for _ in range(100):
            self.target_pub.publish(self.pose)
            rate.sleep()

        self.set_offboard_mode()
        self.arm_drone()


        self.control_loop(rate)

    def state_callback(self, msg):
        self.current_state = msg    # Update drone state

    def error_callback(self, msg):
        self.error_x = msg.data[0]   # Update x-axis error
        self.error_y = msg.data[1]   # Update y-axis error

    def arm_drone(self):
        rospy.wait_for_service('/mavros/cmd/arming')   
        try:
            self.arming_client(value=True)    # Arm the drone
            rospy.loginfo("Drone armed")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def set_offboard_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            self.set_mode_client(custom_mode="OFFBOARD")   # Set Offboard mode
            rospy.loginfo("Offboard mode set")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def control_loop(self, rate):
        pid_hold_count = 0                  # Wait count before PID adjustments
        last_request = rospy.Time.now()
        while not rospy.is_shutdown():
            print(pid_hold_count)
            # Ensure Offboard mode and armed state
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                self.set_offboard_mode()
                last_request = rospy.Time.now()

            if not self.current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                self.arm_drone()
                last_request = rospy.Time.now()
            
            if pid_hold_count > 300:
                # Adjust position using PID
                adjustment_x, adjustment_y = self.pid_controller(self.error_x, self.error_y)

                self.pose.pose.position.x += adjustment_x
                self.pose.pose.position.y += adjustment_y

                self.target_pub.publish(self.pose)

                # Land if errors are within threshold
                if abs(self.error_x) < self.threshold and abs(self.error_y) < self.threshold:
                    self.land_drone()
                    break

                rate.sleep()
            else:
                # Publish initial setpoints during hold
                self.target_pub.publish(self.pose)
                pid_hold_count += 1
                rate.sleep()

    def pid_controller(self, error_x, error_y):
        p_x = self.Kp * error_x
        p_y = self.Kp * error_y

        self.integral_x += error_x
        self.integral_y += error_y
        i_x = self.Ki * self.integral_x
        i_y = self.Ki * self.integral_y

        d_x = self.Kd * (error_x - self.prev_error_x)
        d_y = self.Kd * (error_y - self.prev_error_y)

        self.prev_error_x = error_x
        self.prev_error_y = error_y

        adjustment_x = p_x + i_x + d_x
        adjustment_y = p_y + i_y + d_y

        return adjustment_x, adjustment_y

    def land_drone(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            self.land_client(altitude=0, latitude=0, longitude=0) 
            rospy.loginfo("Landing initiated")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        OffboardController()
    except rospy.ROSInterruptException:
        pass
