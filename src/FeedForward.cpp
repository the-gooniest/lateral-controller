#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

#include "LateralNodeException.h"
#include "CarConstants.h"
#include "FeedForward.h"

using namespace Eigen;
using namespace std;

FeedForward::FeedForward(const Point& vehicle_position, const float &speed, vector<Point> &waypoints) :
        radius(0), straight_line(false), circle_direction(1) {
    if (waypoints.size() < 3)
        throw LateralNodeException("Less than 3 waypoints given");

    find_circle_points(vehicle_position, waypoints);
    circle_fit();
    calculate_feedforward_angle(speed);
}

// Set circle_points array to the three waypoints
// the car is currently driving through.
void FeedForward::find_circle_points(const Point& vehicle_position, vector<Point> &waypoints) {

    int index = 0;
    while (index < waypoints.size()) {
        if (index > waypoints.size() - 3)
            throw LateralNodeException("Car moved beyond available waypoints");

        Point point1 = waypoints[index];
        Point point2 = waypoints[index + 1];
        Point point3 = waypoints[index + 2];
    
        // Get the perpendicular dividing line between points 1 and 3
        Vector2f line_vector(-(point3.y - point1.y), point3.x - point1.x);

        // Get the direction from point2 to the car
        Vector2f car_vector(vehicle_position.x - point2.x, vehicle_position.y - point2.y);

        Matrix2f A;
        A << line_vector, car_vector;
        float determinant = A.determinant();

        if (determinant <= 0) {
            // Ahead of line
            index++;
        }
        else {
            // Behind line
            break;
        }
    }
    circle_points[0] = waypoints[index];
    circle_points[1] = waypoints[index + 1];
    circle_points[2] = waypoints[index + 2];
}

void FeedForward::circle_fit() {
    Point point1 = circle_points[0];
    Point point2 = circle_points[1];
    Point point3 = circle_points[2];

    // check if points need to be reordered to not get a vertical slope
    // (which would cause us to divide by 0)
    // http://paulbourke.net/geometry/circlesphere/
    if (point2.x - point1.x == 0 || point3.x - point2.x == 0) {
        point1 = circle_points[2];
        point2 = circle_points[0];
        point3 = circle_points[1];
        cout << "reordered points" << endl;
    }

    // Get slopes of lines between points 1 and 2 and points 2 and 3
    float m1 = (point2.y - point1.y) / (point2.x - point1.x);
    float m2 = (point3.y - point2.y) / (point3.x - point2.x);

    if (m1 == m2) {
        //cout << "Parallel lines\n";
        straight_line = true;
        return;
    }

    // Calculate the center of the circle made by points 1, 2, and 3
    float x = ((m1 * m2 * (point1.y - point3.y)) + (m2 * (point1.x + point2.x)) - (m1 * (point2.x + point3.x))) / (2 * (m2 - m1));
    float y = ((-1 / m1) * (x - ((point2.x + point1.x) / 2))) + ((point2.y + point1.y) / 2);
    center = Point(x, y);
    cout << "center: " << center << "\n";

    // Calculate the radius of the circle
    Vector2f radius_vector(point1.x - center.x, point1.y - center.y);
    radius = radius_vector.norm();
    cout << "radius: " << radius << endl;

    Vector2f v1(circle_points[1].x -  circle_points[0].x,  circle_points[1].y -  circle_points[0].y);
    Vector2f v2(circle_points[2].x -  circle_points[1].x,  circle_points[2].y -  circle_points[1].y);

    // Use the determinant of the vectors v1 and v2
    // to calculate the direction of circle
    // (left turn = positive determinant)
    // (right turn = negative determinant)
    Matrix2f A;
    A << v1, v2;
    float determinant = A.determinant();
    if (determinant < 0)
        circle_direction = -1;

    if (circle_direction == 1)
        cout << "road direction = left\n";
    else if (circle_direction == -1)
        cout << "road direction = right\n";
    else
        throw LateralNodeException("Bad circle direction");
}

void FeedForward::calculate_feedforward_angle(const float &speed) {
    if (straight_line) {
        feedforward_angle = 0;
        return;
    }

    float length_between_axles = CarConstants::front_length + CarConstants::rear_length;
    float angle = atan(length_between_axles / radius);
    feedforward_angle =
        circle_direction * angle; /*
        ((CarConstants::steering_gradient * speed * speed) / 
        (CarConstants::gravity * radius)));*/
}