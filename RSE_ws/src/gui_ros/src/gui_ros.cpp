#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h" // Add this header
#include "actionlib_msgs/GoalID.h"
#include "TCPServer.h"
#include "json.hpp"
#include <signal.h>

using json = nlohmann::json;

TCPServer server;
double linear_vel = 0.0;
double angular_vel = 0.0;
double goal_x = 0.0;
double goal_y = 0.0;
bool navigation_enabled = false;

void sigintHandler(int sig) {
    ROS_INFO("Shutting down...");
    server.closeConnection();
    ros::shutdown();
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "tcp_server_node");
    ros::NodeHandle nh;

    // Create publisher for cmd_vel topic
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("joy_vel", 1);
    ros::Publisher move_base_goal_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    ros::Publisher move_base_cancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

    // Register SIGINT handler
    signal(SIGINT, sigintHandler);

    // Connect to the server
    server.connect(12222345);

    // Accept connection
    server.acceptConnection();

    while (ros::ok()) {
        // Receive JSON data
        json received_json = server.receiveJSON();
        // Check if the received JSON is not empty
        if (!received_json.empty()) {
            // Extract linear and angular velocities if JSON is not empty
            if (received_json["action"] == "Velocity") {
                linear_vel = received_json["linear"];
                angular_vel = received_json["angular"];
                geometry_msgs::Twist cmd_vel_msg;
                cmd_vel_msg.linear.x = linear_vel;
                cmd_vel_msg.angular.z = angular_vel;

                // Publish Twist message to cmd_vel topic
                cmd_vel_pub.publish(cmd_vel_msg);
            } else if (received_json["action"] == "Navigation") {
                goal_x = received_json["x"];
                goal_y = received_json["y"];
                navigation_enabled = received_json["nav_state"];
                if (navigation_enabled) {
                // Create and publish the MoveBaseActionGoal message
                move_base_msgs::MoveBaseActionGoal goal_msg;
                goal_msg.goal.target_pose.header.frame_id = "map"; // Assuming the goal is in the map frame
                goal_msg.goal.target_pose.pose.position.x = goal_x;
                goal_msg.goal.target_pose.pose.position.y = goal_y;
                goal_msg.goal.target_pose.pose.orientation.w = 1.0;
                move_base_goal_pub.publish(goal_msg);
        } else {
            // Cancel the goal
            actionlib_msgs::GoalID cancel_msg;
            move_base_cancel_pub.publish(cancel_msg);
        }
            }
        }



        // Create Twist message

        ros::spinOnce();
    }

    return 0;
}

