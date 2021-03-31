#include <iostream>
#include <map>
#include <string>
#include <cstdio>
#include <gazebo_slope_estimator.h>

using namespace stochlite;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stochlite_gazebo_slope_esti_node");

    GazeboSlopeEstimator slope_est;
    // //create a map that stores strings indexed by strings
    // std::map<std::string, std::string> m;

    // //add some items to the map
    // m["cat"] = "mieow";
    // m["dog"] = "woof";
    // m["horse"] = "neigh";
    // m["fish"] = "bubble";
    // //now loop through all of the key-value pairs
    // //in the map and print them out
    // for ( auto item : m )
    // {
    //     //item.first is the key
    //     std::cout << item.first << " goes ";

    //     //item.second is the value
    //     std::cout << item.second << std::endl;
    // }

    // char buffer[100];
    // int retVal;
    // char name[] = "Max";
    // int age = 23;

    // for (int i = 0; i < 3; i++)
    // {
    //     /* code */
    //     retVal = sprintf(buffer, "Hi, I am %s and I am %d years old", name, age);
    //     std::cout << buffer << std::endl;
    //     std::cout << "Number of characters written = " << retVal << std::endl;
    
    // }
    

    // //finally, look up the sound of a cat
    // std::cout << "What is the sound of a cat? " << m["cat"] 
    //           << std::endl;

    // return 0;
}
// void tofCallback(const sensor_msgs::Range& tof_data)