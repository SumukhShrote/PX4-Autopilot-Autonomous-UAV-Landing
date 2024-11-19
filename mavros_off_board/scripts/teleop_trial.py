#!/usr/bin/python3

from __future__ import print_function
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped
import sys, select, termios, tty, time, math

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
       w                    i    
   a   s    d           j   k   l
       x                    ,    

   Translation         Rotation (Up-dowm)

r/v : increase/decrease max speeds by 10%
t/b : increase/decrease only linear speed by 10%
y/n : increase/decrease only angular speed by 10%
u/m : increase/decrease only altitude  by 10%

1 Change mode
2 Arm
3 Takeoff
4 Disarm
5 Land

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 1, 0),
    'a': (0, -1, 1, 0),
    'd': (0, 1, 1, 0),
    'x': (-1, 0, 1, 0),
    'k': (0, 0, 1, 0),
    'j': (0, 0, 1, 1),
    'l': (0, 0, 1, -1),
}

speedBindings = {
    'r': (1.1, 1.1, 1),
    'v': (.9, .9, 1),
    't': (1.1, 1, 1),
    'b': (.9, 1, 1),
    'y': (1, 1.1, 1),
    'n': (1, .9, 1),
    'i': (1, 1, 1.1),
    ',': (1, 1, .9),
}

x, y = 0.0, 0.0
last_time = time.time()

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.35)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return f"currently:\tspeed {speed}\tturn {turn}"

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub1 = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    pub2 = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.init_node('teleop_node_local')

    arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    landing_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    speed = rospy.get_param("~speed", 1.5)
    turn = rospy.get_param("~turn", 1.0)
    alt = 2
    vx = 0
    vy = 0
    z = 0
    yaw_rate = 0
    status = 0
    mask = 1987
    last_z = 0
    rate = rospy.Rate(10)

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey()
            print("using getKey function input:", key)
            if key in moveBindings.keys():
                vx, vy, z, yaw_rate = moveBindings[key]
                last_z = z
            elif key in speedBindings.keys():
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                alt *= speedBindings[key][2]
                print(vels(speed, turn))
                status = (status + 1) % 15

            elif key == '2':
                rospy.wait_for_service('/mavros/cmd/arming')
                response = arming_cl(value=True)
                rospy.loginfo(response)

            elif key == '4':
                rospy.wait_for_service('/mavros/cmd/arming')
                response = arming_cl(value=False)
                rospy.loginfo(response)

            elif key == '3':
                rospy.wait_for_service('/mavros/cmd/takeoff')
                response = takeoff_cl(altitude=3, latitude=0, longitude=0, min_pitch=0, yaw=0)
                rospy.loginfo(response)

            elif key == '5':
                rospy.wait_for_service('/mavros/cmd/land')
                response = landing_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
                rospy.loginfo(response)

            elif key == '1':
                rospy.wait_for_service('/mavros/set_mode')
                response = change_mode(custom_mode="OFFBOARD")
                rospy.loginfo(response)

            else:
                vx = vy = yaw_rate = 0
                z = last_z
                if key == '\x03':
                    break

            post = PositionTarget()
            post.header.frame_id = "home"
            post.header.stamp = rospy.Time.now()
            post.coordinate_frame = 8
            post.type_mask = mask
            post.position.z = z * alt
            post.velocity.x = vy * speed
            post.velocity.y = vx * speed
            post.yaw_rate = yaw_rate * turn
            pub1.publish(post)

            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time

            x += (vx * speed * math.cos(yaw_rate) - vy * speed * math.sin(yaw_rate)) * delta_time
            y += (vx * speed * math.sin(yaw_rate) + vy * speed * math.cos(yaw_rate)) * delta_time
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z * alt
            pub2.publish(pose)

            print("Published position and velocity.\n")
            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        post = PositionTarget()
        post.header.frame_id = "home"
        post.header.stamp = rospy.Time.now()
        post.coordinate_frame = 8
        post.type_mask = mask
        post.position.z = 1
        post.velocity.x = 0
        post.velocity.y = 0
        post.yaw_rate = 0
        pub1.publish(post)
        rate.sleep()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
