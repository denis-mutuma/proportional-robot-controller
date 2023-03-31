#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"
#include <cstdlib>
#include <iostream>

// declaring robot position variables
geometry_msgs::Twist cmd_vel;
geometry_msgs::Pose2D current;
geometry_msgs::Pose2D desired;

// the goal is a value between 0 and 11 in x and y directions
double goal_x;
double goal_y;

const double PI = 3.14159265;

// the coefficient(gain) for linear velocity
const double kLinear = 0.75;

// the coefficient(gain) for angular velocity
const double kAngular = 0.75;

// distance we are willing to accept as "close enough"
const double kLinearTolerance = 0.1;

// angle we are willing to accept as "close enough"
const double kAngularTolerance = 0.1;

// initialize desired pose params
// goal_x and goal_y will be set using the std input
void Setup()
{
    desired.x = goal_x;
    desired.y = goal_y;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
}

// callback function to update the current location
void UpdatePose(const turtlesim::PoseConstPtr &kCurrentPose)
{
    current.x = kCurrentPose->x;
    current.y = kCurrentPose->y;
    current.theta = kCurrentPose->theta;
}

// calculate the Euclidean distance between current position and desired location
double GetDistanceError()
{
    return sqrt(pow(desired.x - current.x, 2) + pow(desired.y - current.y, 2));
}

// calculates error in approach angle
double GetAngularError()
{
    double delta_x = desired.x - current.x;
    double delta_y = desired.y - current.y;
    double theta_bearing = atan2(delta_y, delta_x);

    double angular_error = theta_bearing - current.theta;
    angular_error = angular_error > PI ? angular_error - (2 * PI) : angular_error;
    angular_error = angular_error < -PI ? angular_error + (2 * PI) : angular_error;

    return angular_error;
}

// set the robot velocity if robot not at desired position but first turn the robot
// to point in the direction of the desired position before heading there
void SetVelocity()
{
    double angular_error = GetAngularError();
    double distance_error = GetDistanceError();

    if (distance_error > kLinearTolerance)
    {
        if (abs(angular_error) < kAngularTolerance)
        {
            cmd_vel.linear.x = kLinear * distance_error;
            cmd_vel.angular.z = 0;
        }
        else
        {
            cmd_vel.angular.z = kAngular * angular_error;
            cmd_vel.linear.x = 0;
        }
    }
    else
    {
        std::cout << "Arrived at destination" << std::endl;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
    }
}

int main(int argc, char **argv)
{
    // read the goal_x and goal_y from std input
    std::cout << "Please enter two space separated desired positions x and y: " << std::endl;
    std::cin >> goal_x >> goal_y;

    Setup();

    // register node "goto_x" with roscore and get a nodehandle
    ros::init(argc, argv, "goto_xy");
    ros::NodeHandle node;

    // subscribe to topic and set callback to update robot pose
    ros::Subscriber subscribe_current_pose = node.subscribe("turtle1/pose", 0, UpdatePose);

    // register node as publisher
    ros::Publisher publish_velocity = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);

    // set publish frequency at 10 Hz
    ros::Rate loop_rate(10);

    // execute loop until connection to master is lost
    while (ros::ok())
    {
        // call the callbacks waiting to be called
        ros::spinOnce();

        // call controller after callbacks are done
        SetVelocity();

        // publish messages
        publish_velocity.publish(cmd_vel);

        // some output to display robot variables at runtime
        std::cout << "desired x = " << desired.x << std::endl
                  << "desired y = " << desired.y << std::endl
                  << "current x = " << current.x << std::endl
                  << "current y = " << current.y << std::endl
                  << "euclidean distance error x = " << GetDistanceError() << std::endl
                  << "cmd_vel x = " << cmd_vel.linear.x << std::endl
                  << "cmd_vel y = " << cmd_vel.linear.y << std::endl;

        // this sleeps while maintaining a frequency of 10 Hz
        loop_rate.sleep();
    }

    return 0;
}