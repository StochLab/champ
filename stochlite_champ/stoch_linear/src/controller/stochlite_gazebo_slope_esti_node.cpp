// written by github@aditya-shirwatkar

#include <iostream>
#include "stoch_linear/gazebo_slope_estimator.h"

using namespace stochlite;

int main(int argc, char **argv)
{   
    // node for slope estimator
    ros::init(argc, argv, "stochlite_gazebo_slope_esti_node");

    GazeboSlopeEstimator slope_est;
}
