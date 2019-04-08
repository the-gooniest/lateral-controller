#include "LateralNodeException.h"
#include "LateralROSNode.h"
#include "CarConstants.h"
#include "FeedForward.h"
#include "Gains.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <map>

using namespace std;

int main(int argc, char **argv) 
{   
    ros::init(argc, argv, "lateral_controller");
    LateralROSNode LateralNode;

    float speed, k_lat, k_head, k_o;
    string input;
    ifstream file_input;
    bool error;

    file_input.open("/home/steven/catkin_ws/src/lateral_controller/src/gains.txt");
    error=false;

    while (true)
    {
        getline(file_input, input);
        if (!file_input) break; //check for eof

        istringstream buffer(input);
        buffer >> speed >> k_lat >> k_head >> k_o;

        //check for non numerical input
        //and less/more input than needed
        if (!buffer || !buffer.eof())
        {
            error=true;
            break;
        }
        Gains g(k_lat, k_head, k_o);
        LateralNode.gains_map.insert(pair<float, Gains>(speed, g));
        if (LateralNode.highest_gains_speed < speed)
            LateralNode.highest_gains_speed = speed;
    }

    if (error)
        cout << "file is corrupted..." << endl;

    for (map<float, Gains>::iterator it = LateralNode.gains_map.begin(); it != LateralNode.gains_map.end(); ++it) {
        cout << it->first << '\t' << it->second.k_lat << '\t' << it->second.k_head << '\t' << it->second.k_o << endl;
    }
    cout << "Highest speed: " << LateralNode.highest_gains_speed << endl;

    try {
        LateralNode.Initialize(argc, argv);
        LateralNode.UpdateLoop();
    }
    catch (const LateralNodeException e) {
        cout << "\n" << e.what() << endl;
        cout << "Stopping lateral_controller node..." << endl;
    }

    LateralNode.Cleanup();
    return 0;
}
