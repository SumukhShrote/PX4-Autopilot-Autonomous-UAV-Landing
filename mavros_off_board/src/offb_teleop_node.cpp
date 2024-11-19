#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/PoseStamped.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <map>
#include <string>

std::map<char, std::tuple<int, int, int, int>> moveBindings{
    {'w', {1, 0, 1, 0}},
    {'a', {0, -1, 1, 0}},
    {'d', {0, 1, 1, 0}},
    {'x', {-1, 0, 1, 0}},
    {'k', {0, 0, 1, 0}},
    {'j', {0, 0, 1, 1}},
    {'l', {0, 0, 1, -1}},
};

std::map<char, std::tuple<float, float, float>> speedBindings{
    {'r', {1.1, 1.1, 1}},
    {'v', {0.9, 0.9, 1}},
    {'t', {1.1, 1, 1}},
    {'b', {0.9, 1, 1}},
    {'y', {1, 1.1, 1}},
    {'n', {1, 0.9, 1}},
    {'i', {1, 1, 1.1}},
    {',', {1, 1, 0.9}},
};

float x = 0.0, y = 0.0;
double last_time = ros::Time::now().toSec();
float speed = 1.5;
float turn = 1.0;
float alt = 2.0;

struct termios oldt;

void setRawMode() {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

void resetMode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

char getKey() {
    char buf = 0;
    struct timeval timeout;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 350000;

    int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
    if (rv == -1) {
        perror("select");
        return 0;
    } else if (rv == 0) {
        return 0;
    } else {
        read(STDIN_FILENO, &buf, 1);
        return buf;
    }
}

void quitHandler(int sig) {
    resetMode();
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "teleop_node_local");
    ros::NodeHandle nh;

    ros::Publisher pub1 = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_cl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    ros::ServiceClient landing_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    ros::ServiceClient change_mode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    signal(SIGINT, quitHandler);
    setRawMode();

    float vx = 0, vy = 0, z = 0, yaw_rate = 0;
    int mask = 1987;
    double last_z = 0;
    ros::Rate rate(10);

    std::cout << "Press CTRL-C to quit." << std::endl;

    while (ros::ok()) {
        char key = getKey();
        if (key && moveBindings.count(key)) {
            auto &[vx_tmp, vy_tmp, z_tmp, yaw_tmp] = moveBindings[key];
            vx = vx_tmp;
            vy = vy_tmp;
            z = z_tmp;
            yaw_rate = yaw_tmp;
            last_z = z;
        } else if (key && speedBindings.count(key)) {
            auto &[s, t, a] = speedBindings[key];
            speed *= s;
            turn *= t;
            alt *= a;
            std::cout << "Speed: " << speed << " Turn: " << turn << std::endl;
        } else if (key == '2') {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if (arming_cl.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Armed successfully.");
            }
        } else if (key == '4') {
            mavros_msgs::CommandBool disarm_cmd;
            disarm_cmd.request.value = false;
            if (arming_cl.call(disarm_cmd) && disarm_cmd.response.success) {
                ROS_INFO("Disarmed successfully.");
            }
        } else if (key == '3') {
            mavros_msgs::CommandTOL takeoff_cmd;
            takeoff_cmd.request.altitude = 3;
            if (takeoff_cl.call(takeoff_cmd) && takeoff_cmd.response.success) {
                ROS_INFO("Takeoff initiated.");
            }
        } else if (key == '5') {
            mavros_msgs::CommandTOL land_cmd;
            if (landing_cl.call(land_cmd) && land_cmd.response.success) {
                ROS_INFO("Landing initiated.");
            }
        } else if (key == '1') {
            mavros_msgs::SetMode mode_cmd;
            mode_cmd.request.custom_mode = "OFFBOARD";
            if (change_mode.call(mode_cmd) && mode_cmd.response.mode_sent) {
                ROS_INFO("Mode set to OFFBOARD.");
            }
        } else if (key == 3) { // CTRL-C
            break;
        }

        mavros_msgs::PositionTarget post;
        post.header.frame_id = "home";
        post.header.stamp = ros::Time::now();
        post.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        post.type_mask = mask;
        post.position.z = z * alt;
        post.velocity.x = vy * speed;
        post.velocity.y = vx * speed;
        post.yaw_rate = yaw_rate * turn;
        pub1.publish(post);

        double current_time = ros::Time::now().toSec();
        double delta_time = current_time - last_time;
        last_time = current_time;

        x += (vx * speed * cos(yaw_rate) - vy * speed * sin(yaw_rate)) * delta_time;
        y += (vx * speed * sin(yaw_rate) + vy * speed * cos(yaw_rate)) * delta_time;

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z * alt;
        pub2.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    resetMode();
    return 0;
}
