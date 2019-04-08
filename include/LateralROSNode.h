#ifndef LATERALROSNODE_H
#define LATERALROSNODE_H

#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "bolt_msgs/SteeringCmd.h"
#include "bolt_msgs/SteeringReport.h"
#include "std_msgs/Float32MultiArray.h"

#include "Point.h"
#include "Gains.h"
#include "LateralNodeException.h"

enum Status {
    Running, Stopped
};

class LateralROSNode {
public:
    LateralROSNode();
    void Initialize(int argc, char **argv);
    void UpdateLoop();
    void Cleanup();

    // Signal Handers
    static Status status;
    static void sigint_handler(int s);

    // Helpers
    void wait_for_waypoints();

    // Callbacks
    void speedCallback(const std_msgs::Float32::ConstPtr& msg);
    void headingCallback(const std_msgs::Float32::ConstPtr& msg);
    void angularVelocityCallback(const std_msgs::Float32::ConstPtr& msg);
    void positionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void waypointsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void simulationStatusCallback(const std_msgs::String::ConstPtr& msg);

    std::map<float, Gains> gains_map;
    float highest_gains_speed;

private:

    ros::NodeHandle node;
    ros::Rate loop_rate;

    float speed;
    float heading;
    float angular_velocity;
    Point position;
    std::vector<Point> waypoints;

    // Publishers
    ros::Publisher pub_steering_angle;
    ros::Publisher pub_simulation_status;

    // Subscribers
    ros::Subscriber sub_speed;
    ros::Subscriber sub_heading;
    ros::Subscriber sub_angular_velocity;
    ros::Subscriber sub_waypoints;
    ros::Subscriber sub_position;
    ros::Subscriber sub_simulation_status;
};

#endif
