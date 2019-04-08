/*
#include <iostream>
#include <cmath> //This is to get infinity and trig functions

#include <eigen3/Eigen/Dense>

#include "Controller.h"
#include "FeedForward.h"


using namespace std;
using namespace Eigen;

#define pi 3.141592

Controller::Controller()
{

}

float Controller::feedforward(CarConstants car, WayPoints wps, float vx)
{
    // These are the inputs in order: front length, rear length, steering gradient,
    // radius of curvature, longitudinal velocity, the lookahead points, and a boolean describing if the current lookahead points are in a straight line. The method
    // returns 0 if the path is straight, and takes into account the direction of the turn in the calculation of the steering angle.
    
    float lf, lr, Ksg;
    lf = car.front_length;
    lr = car.rear_length;
    Ksg = car.steering_gradient;

    Matrix<float, 3,2> lookahead_points = wps.get_lookahead_points();
    float r = wps.get_circle_r();
    bool is_straight = wps.get_is_straight();

    float dff;

    //PRINT CURRENT LOOKAHEAD DATA
    // std::cout << lookahead_points << endl << endl;

    if (is_straight) {
        dff = 0;
    }
    else {
        if (r == 0) 
            {
                std::cout << "Error inside feedforward: radius is 0\n";
                return 0;
            }
            else
            {
                // Compute the direction of the turn based on the determinant of a matrix containing the distances between the third and second, and first and second points.
                // direction is -1 if the turn is left, and positive 1 if it is right. 
                Matrix<float, 1,2> q1, q2;
                q1 = lookahead_points.row(1) - lookahead_points.row(0);
                q2 = lookahead_points.row(2) - lookahead_points.row(1);
                Matrix<float, 2,2> m;
                m << q1,q2;

                int direction;
                if (m.determinant()<0) {
                    direction = -1;// right hand turn
                }
                else {
                    direction = 1;// left hand turn
                }

                // Evaluate the steering angle required to turn at radius r and apply the direction.
                dff = (lf+lr)/r + Ksg*vx*vx/(9.81*r);
                dff = direction*dff;
            }
    }  
    return dff;
}

float Controller::feedback(WayPoints wps, float x_car, float y_car, float vx, float theta_car, float omega_car)
{

    // The inputs go in order: x position of the vehicle, y position of the vehicle, 
    // the x position of the circle's center, the y position of the circle's center, 
    // the radius of the circle, the longitudinal velocity, the vehicle's heading,
    // the vehicle's angular velocity, and the points from our current segment.


    // Declare variables
    float dfb;
    float klat;
    float elat;
    float kyawrate;
    float eyawrate;
    float ktheta;
    float etheta;

    Matrix<float, 3,2> lookahead_points = wps.get_lookahead_points();
    float x_circle = wps.get_circle_x();
    float y_circle = wps.get_circle_y();
    float r = wps.get_circle_r();
    bool is_straight = wps.get_is_straight();

    //establish change of x and change of y between points
    float x1 = lookahead_points(0,0);
    float x2 = lookahead_points(1,0);
    float y1 = lookahead_points(0,0);
    float y2 = lookahead_points(1,0);

    float dx = x2 - x1; //change of x
    float dy = y2 - y1; //change of y

    //elat calculations
    if (is_straight){
        if (dy == 0 && dx == 0){
            cout << "Point 1 and Point 2 are the same. Error in feedback\n";
            //Need to update this later
        }
        else if (dx == 0)   //going north or south
        {
            elat = x_car - x1;
            Matrix2f LoRvectors; 
            LoRvectors << x_car - x1,0,cos(theta_car), sin(theta_car);
            int LoR = copysignf (1, LoRvectors.determinant());
            klat = -.03*LoR;
        }
        else {
            float m = dy/dx;
            Matrix <float, 1, 2 > a;
            a << 1,m;
            Matrix <float, 1, 2 > b;
            b << x_car,y_car-(y1-m*x1);
            Matrix <float, 2, 2 > ab;
            ab << a,b;
            klat = -.03;
            if (dx > 0) // going east, northeastm southeast
            {
                elat = ab.determinant() / a.norm();
            }
            else    // going west, northwest or southwest 
            {
                elat = -ab.determinant() / a.norm();
            }
        }   
    }
    else //A best-fit circle is defined between the points, so it's easier
    {
        elat = sqrtf(powf(x_car-x_circle,2)+powf(y_car-y_circle,2)) - r;
        //sqrt((xcar-xcir)^2+(ycar-ycir)^2) - radius in float
        Matrix2f LoRvectors;
        LoRvectors << x_car-x_circle,y_car-y_circle,cos(theta_car),sin(theta_car);
        int LoR = copysignf(1, LoRvectors.determinant());
        klat = 0.03*LoR;
    }
 
    // car's yawrate - v/r error
    Matrix<float, 1,2> q1, q2;
    q1 = lookahead_points.row(1) - lookahead_points.row(0);//vector from point 0 to point 1
    q2 = lookahead_points.row(2) - lookahead_points.row(1);//vector from point 1 to point 2
    Matrix<float, 2,2> m;
    m << q1,q2;
    int direc; 
    direc = copysignf(1,m.determinant());

    // -1 is a right hand turn
    // +1 is a left hand turn
    if (is_straight){
        eyawrate = omega_car;
    } else {
        eyawrate = omega_car - direc*vx/r;
    }
    kyawrate = .03;

    // heading error
    float theta_wrap = fmod(theta_car+pi,2*pi);
    if (theta_wrap<0){
        theta_wrap += 2*pi;
    }
    theta_wrap -= pi;
    float eyaw =  theta_wrap - atan2(q1(1),q1(0));
    ktheta = .26;
    etheta = fmod(eyaw+pi,2*pi);
    if (etheta<0){
        etheta += 2*pi;
    }
    etheta -= pi;

    dfb = klat*elat + kyawrate*eyawrate + ktheta*etheta;
    return dfb;
}

float Controller::total_steerangle(CarConstants car, WayPoints wps, float vx, float x_car, float y_car, float theta_car, float omega_car)
{
    float dff = feedforward(car, wps, vx);
    float dfb = feedback(wps, x_car, y_car, vx, theta_car, omega_car);

    // cout << wps.lookahead_points <<endl;
    cout << 57.2958*(dff) <<endl<<endl;
    // cout << 57.2958*(dfb) <<endl<<endl;

    return dff;
}
*/
