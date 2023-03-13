#include "ClutteredEnvGenerator.hpp"
// #include "timing.h"
using namespace graceful_mpc;
int main(int argc, char** argv) {
    ros::init(argc, argv, "clutter_env_generator");
    ros::NodeHandle nh("~");
    // timing::Timer map("map_generate");
    auto envGen = ClutteredEnvGenerator(nh);
    envGen.generateStaticMap();
    auto rate = ros::Rate(10);
    // 70ms for 2cm resolution; 2ms for 5cm resolution; 12ms for 3cm;
    while(ros::ok()){
        // map.Start();
        envGen.updateDynamicMap();
        envGen.publishMap();
        // map.Stop();
        rate.sleep();
        // timing::Timing::Print(std::cout);
    }
    ros::spin();
}
