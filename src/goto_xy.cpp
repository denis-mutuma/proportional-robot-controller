#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"
#include <cstdlib>
#include <iostream>

// declaring robot position variables
geometry_msgs::Twist cmdVel;
geometry_msgs::Pose2D current;
geometry_msgs::Pose2D desired;

// the GOAL is a value between 0 and 11 in x and y directions
double GOAL_X;
double GOAL_Y;

const double PI = 3.14159265;

// the coefficient(gain) for linear velocity
const double K_linear = 0.75;

// the coefficient(gain) for angular velocity
const double K_angular = 0.75;

// distance we are willing to accept as "close enough"
const double linearTolerance = 0.1;

// angle we are willing to accept as "close enough"
const double angularTolerance = 0.1;

// initialize desired pose params
// GOAL_X and GOAL_Y will be set using the std input
void setup()
{
    desired.x = GOAL_X;
    desired.y = GOAL_Y;
    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVel.linear.z = 0;
    cmdVel.angular.x = 0;
    cmdVel.angular.y = 0;
    cmdVel.angular.z = 0;
}

// callback function to update the current location
void update_pose(const turtlesim::PoseConstPtr &currentPose)
{
    current.x = currentPose->x;
    current.y = currentPose->y;
    current.theta = currentPose->theta;
}

// calculate the Euclidean distance between current position and desired location
double getDistanceError()
{
    return sqrt(pow(desired.x - current.x, 2) + pow(desired.y - current.y,2));
}

// calculates error in approach angle
double getAngularError()
{
    double delta_x = desired.x - current.x;
    double delta_y = desired.y - current.y;
    double thetaBearing = atan2(delta_y, delta_x);

    double angularError = thetaBearing - current.theta;
    angularError = angularError > PI ? angularError - (2*PI) : angularError;
    angularError = angularError < -PI ? angularError + (2*PI) : angularError;

    return angularError;
}

// set the robot velocity if robot not at desired position but first turn the robot 
// to point in the direction of the desired position before heading there
void set_velocity()
{
    double angularError = getAngularError();
    double distanceError = getDistanceError();

    if(distanceError > linearTolerance)
    {
        if(abs(angularError) < angularTolerance)
        {
            cmdVel.linear.x = K_linear*distanceError;
            cmdVel.angular.z = 0;
        } 
        else 
        {
            cmdVel.angular.z = K_angular * angularError;
            cmdVel.linear.x = 0;
        }
    } 
    else {
        std::cout << "Arrived at destination" << std::endl;
        cmdVel.linear.x = 0;
        cmdVel.linear.y = 0;
        cmdVel.angular.z = 0;
    }
}

int main(int argc, char** argv)
{
    // read the GOAL_X and GOAL_Y from std input
    std::cout << "Please enter two space separated desired positions x and y: " << std::endl;
    std::cin >> GOAL_X >> GOAL_Y;

    setup(); 

    // register node "goto_x" with roscore and get a nodehandle
    ros::init(argc, argv, "goto_xy");
    ros::NodeHandle node;

    // subscribe to topic and set callback to update robot pose
    ros::Subscriber subCurrentPose = node.subscribe("turtle1/pose", 0, update_pose);

    // register node as publisher
    ros::Publisher pubVelocity = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",0);

    // set publish frequency at 10 Hz
    ros::Rate loop_rate(10);

    // execute loop until connection to master is lost
    while(ros::ok())
    {
    // call the callbacks waiting to be called
    ros::spinOnce();

    // call controller after callbacks are done
    set_velocity();

    // publish messages
    pubVelocity.publish(cmdVel);

    // some output to display robot variables at runtime
    std::cout << "desired x = " << desired.x << std::endl
        << "desired y = " << desired.y << std::endl
        << "current x = " << current.x << std::endl
        << "current y = " << current.y << std::endl
        << "euclidean distance error x = " << getDistanceError() << std::endl
        << "cmd_vel x = " << cmdVel.linear.x << std::endl
        << "cmd_vel y = " << cmdVel.linear.y << std::endl;

    // this sleeps while maintaining a frequency of 10 Hz
    loop_rate.sleep();
    }

    return 0;
}