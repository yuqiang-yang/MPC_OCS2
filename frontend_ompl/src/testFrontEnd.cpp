#include <FrontEndOMPLRRTStar.hpp>
using namespace perceptive_mpc;
int main(int argc, char** argv) {
    FrontEndOMPLRRTStarConfig config;
    config.planning_time = 0.05;
    config.margin_x = 0.2;
    config.margin_y = 0.2;
    config.margin_z = 0.2;

    FrontEndOMPLRRTStar frontEndOMPLRRTStar(config);

    Eigen::Matrix<double,7,1> start;
    start << 0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.4;
    Eigen::Matrix<double,7,1> end;
    end << 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.3;
    Eigen::Matrix<double,Eigen::Dynamic,7> desired_trajectory; 
    frontEndOMPLRRTStar.Plan(start,end,desired_trajectory);
    std::cerr << desired_trajectory << std::endl;

    return 0;

}

