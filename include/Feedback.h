#ifndef FEEDBACK_H
#define FEEDBACK_H

#include "Point.h"

// Forward declare FeedForward class
class FeedForward;

class Feedback {

    float feedback_angle;

public:

    Feedback(FeedForward *ff, Point &car_position, float car_heading, float angular_velocity, float k_lat, float k_head, float k_o);

    float calculate_lateral_error(FeedForward *ff, Point &car_position, float k_lat);
    float calculate_heading_error(FeedForward *ff, Point &car_position, float car_heading, float k_head);
    float calculate_omega_error(FeedForward *ff, float angular_velocity, float k_o);

    float get_feedback_angle() { return feedback_angle; }
};

#endif