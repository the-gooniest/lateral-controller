#include <CarConstants.h>

// Total mass of the the 2017 Chevy Bolt in kg.
// TODO: change mass with added weight from sensors, people, etc
const float CarConstants::mass = 1605.0f;

// Yaw inertia of the 2017 Chevy Bolt in kg*m^2.
// This value is not used for now.
const float CarConstants::yaw_inertia = 2045.0f;

// Length from CG (Center of Gravity) to the front wheels of the 2017 Chevy Bolt in m. 
// TODO: check if this is correct.
const float CarConstants::front_length = 1.135f;

// Length from CG to the rear wheels of the 2017 Chevy Bolt in m.
// TODO: check if this is correct.
const float CarConstants::rear_length = 1.465f;

// Cornering stiffness of the front wheels of a 1949 Buick in N/rad.
// TODO: Testing is required to find actual value
const float CarConstants::front_stiffness = 77850.0f;

// Cornering stiffness of the rear wheels of a 1949 Buick in N/rad.
// TODO: Testing is required to find actual value
const float CarConstants::rear_stiffness = 76510.0f;

// Gravity in m/s
const float CarConstants::gravity = 9.81f;

// Front weight of the vehicle in N.
const float CarConstants::front_weight = (mass*rear_length*gravity)/(front_length+rear_length);

// Rear weight of the vehicle in N.
const float CarConstants::rear_weight = (mass*front_length*gravity)/(front_length+rear_length);

// Steering Gradient of the vehicle in radians.
const float CarConstants::steering_gradient = (front_weight/front_stiffness) - (rear_weight/rear_stiffness);
