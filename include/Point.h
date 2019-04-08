#ifndef POINT_H
#define POINT_H

#include <iostream>

class Point {
public:
    float x;
    float y;

    Point() : x(0), y(0) {}
    Point(float x1, float y1) : x(x1) , y(y1) {}
    
    friend std::ostream& operator<<(std::ostream &out, const Point &point);
};

inline std::ostream& operator<<(std::ostream &out, const Point &point) {
    out << "(" << point.x << ", " << point.y << ")";
    return out;
}

#endif