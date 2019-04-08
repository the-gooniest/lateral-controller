#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES

#include <eigen3/Eigen/Dense>
#include "Feedback.h"
#include "FeedForward.h"

using namespace Eigen;
using namespace std;

Feedback::Feedback(FeedForward *ff, Point &car_position, float car_heading, float angular_velocity, float k_lat, float k_head, float k_o) {
    feedback_angle = 
        calculate_lateral_error(ff, car_position, k_lat) + 
        calculate_heading_error(ff, car_position, car_heading, k_head) +
        calculate_omega_error(ff, angular_velocity, k_o);
}

float Feedback::calculate_lateral_error(FeedForward *ff, Point &car_position, float k_lat) {
    if (ff->straight_line)
        return 0;

    // Calculate distance from the vehicle's position to the center of the feedforward circle
    Vector2f distance_vector(car_position.x - ff->center.x, car_position.y - ff->center.y);
    float distance = distance_vector.norm();

    // positive error = left turn
    // negative error = right turn
    float error = distance - abs(ff->radius);
    float correction = k_lat * ff->circle_direction * error;
    return correction;
}

float Feedback::calculate_heading_error(FeedForward *ff, Point &car_position, float car_heading, float k_head) {
    // Calculate vector from circle center to car position
    Point circle_vector(car_position.x - ff->center.x, car_position.y - ff->center.y);

    // get tangent vector to the road vector (depends on the circle's direction)
    Point road_vector;
    if (ff->circle_direction == 1)
        road_vector = Point(-circle_vector.y, circle_vector.x);
    else
        road_vector = Point(circle_vector.y, -circle_vector.x);

    // get the road's heading in world space
    float road_heading = atan2(road_vector.y, road_vector.x);
    if (road_heading < 0)
        road_heading += M_PI * 2;
    car_heading = fmod(car_heading, M_PI * 2);

    //cout << "road heading: " << road_heading << endl;
    //cout << "car heading: " << car_heading << endl;

    // Calculate the angle difference between the car's and the road's headings
    float error = road_heading - car_heading;
    if (error >= M_PI)
        error -= M_PI * 2;
    else if (error <= -M_PI)
        error += M_PI * 2;
    cout << "heading error: " << error << endl;

    // apply gain to the heading error
    float correction = k_head * error;
    cout << "heading correction = " << correction << endl;
    return correction;
}

float Feedback::calculate_omega_error(FeedForward *ff, float angular_velocity, float k_o) {
    float correction = -k_o * angular_velocity; // speed / radius
    return correction;
}