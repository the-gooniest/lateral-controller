#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

#include <vector>
#include "Point.h"

class Point;

class FeedForward {

    // Helper functions
    void find_circle_points(const Point& vehicle_position, std::vector<Point> &waypoints);
    void circle_fit();
    void calculate_feedforward_angle(const float &speed);

public:

    // The circle's center
    Point center;

    // Radius of curvature
    float radius;

    // Check for circle points that make a straight line
    bool straight_line;

    // curving left = 1, curving right = 0
    int circle_direction;

    float feedforward_angle;

    // Current 3 points
    Point circle_points[3];

    FeedForward(const Point& vehicle_position, const float &speed, std::vector<Point> &waypoints);
    const float get_feedforward_angle() { return feedforward_angle; }
};

#endif
