#include <iostream>
#include <string>
#include <math.h>

#include "LateralNodeException.h"
#include "CarConstants.h"
#include "FeedForward.h"
#include "Feedback.h"
#include "Controller.h"
#include "LateralROSNode.h"
#include "signal.h"

using namespace std;
using namespace Eigen;
using namespace std_msgs;

Status LateralROSNode::status = Running;

LateralROSNode::LateralROSNode() : loop_rate(30), speed(0), heading(0), angular_velocity(0), highest_gains_speed(0) {}

void LateralROSNode::sigint_handler(int s) {
    status = Stopped;
    throw LateralNodeException("Captured control-c signal");
}

void LateralROSNode::simulationStatusCallback(const std_msgs::String::ConstPtr& msg){
	string message = msg->data;
    if (message == "stop") {
        status = Stopped;
        cout << "Received stop status message from simulation" << endl;
    }
}

void LateralROSNode::speedCallback(const std_msgs::Float32::ConstPtr& msg){
	speed = msg->data;
}

void LateralROSNode::headingCallback(const std_msgs::Float32::ConstPtr& msg){
	heading = msg->data;
}

void LateralROSNode::angularVelocityCallback(const std_msgs::Float32::ConstPtr& msg){
    angular_velocity = msg->data;
}

void LateralROSNode::positionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	for(std::vector<float>::const_iterator iterator = msg->data.begin(); iterator != msg->data.end(); iterator++){
        float x = *iterator;
        iterator++;
        float y = *iterator;
        position = Point(x, y);
	}
}

void LateralROSNode::waypointsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    waypoints.clear();
	for(std::vector<float>::const_iterator iterator = msg->data.begin(); iterator != msg->data.end(); iterator++){
        float x = *iterator;
        iterator++;
        float y = *iterator;
        waypoints.push_back(Point(x, y));
	}
}

// Waits until waypoint data has been sent
void LateralROSNode::wait_for_waypoints() {
    cout << "Ready and waiting on waypoints to be sent..." << endl;
    while (waypoints.size() < 3) {
        ros::spinOnce();
        loop_rate.sleep();

        // Send ready message to simulation
        std_msgs::String status_msg;
        status_msg.data = "lateral_controller: ready";
        pub_simulation_status.publish(status_msg);
        continue;
    }
}

void LateralROSNode::Initialize(int argc, char **argv)
{  
    // publishers
    pub_steering_angle = node.advertise<bolt_msgs::SteeringCmd>("autodrive_sim/input/steering_angle", 1);
    pub_simulation_status = node.advertise<std_msgs::String>("autodrive_sim/input/status", 1);

    // subscriptions
    sub_speed = node.subscribe("autodrive_sim/output/speed", 1, &LateralROSNode::speedCallback, this);
    sub_heading = node.subscribe("autodrive_sim/output/heading", 1, &LateralROSNode::headingCallback, this);
    sub_angular_velocity = node.subscribe("autodrive_sim/output/angular_velocity", 1, &LateralROSNode::angularVelocityCallback, this);
    sub_waypoints = node.subscribe("autodrive_sim/output/waypoints", 1,&LateralROSNode::waypointsCallback, this);
    sub_position = node.subscribe("autodrive_sim/output/position", 1, &LateralROSNode::positionCallback, this);
    sub_simulation_status = node.subscribe("autodrive_sim/output/status", 1, &LateralROSNode::simulationStatusCallback, this);

    // Setup Control-c handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = LateralROSNode::sigint_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
}

void LateralROSNode::UpdateLoop()
{
    wait_for_waypoints();
    // wait for other inputs maybe

    // Begin the main loop
    cout << "Starting Controller" << endl;
    while(ros::ok() && status == Running) {

        FeedForward feedforward(position, speed, waypoints);

        float rounded_speed = round(speed);
        if (rounded_speed <= 0)
            rounded_speed = 1;

        if (rounded_speed > highest_gains_speed)
            rounded_speed = highest_gains_speed;

        cout << "speed: " << speed << endl;
        cout << "rounded_speed: " << rounded_speed << endl;
        Gains g = gains_map[rounded_speed];
        cout << g << endl;
        Feedback feedback(&feedforward, position, heading, angular_velocity, g.k_lat, g.k_head, g.k_o);

        float steering_cmd = -57.2958 * (feedforward.get_feedforward_angle() + feedback.get_feedback_angle());

        // normalize angle
        if (steering_cmd > 34){
            steering_cmd = 34;
        }
        else if (steering_cmd < -34){
            steering_cmd = -34;
        }

        // Construct new steering msg and publish to the simulation
        bolt_msgs::SteeringCmd msg;
        msg.steering_wheel_angle_cmd = steering_cmd;
        pub_steering_angle.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void LateralROSNode::Cleanup()
{
    //std_msgs::String msg;
    //msg.data = "stop";
    //pub_simulation_status.publish(msg);
    //cout << "Lateral Controller has stopped and cleaned up successfully." << endl;
}
