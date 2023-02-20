#include <FrontEndOMPLRRTStar.hpp>
using namespace graceful_mpc;
int main(int argc, char** argv) {
    FrontEndOMPLRRTStarConfig config;
    config.planning_time = 2;
    config.margin_x = 0.2;
    config.margin_y = 0.2;
    config.margin_z = 0.2;
    config.collisionCheckerResolution = 0.02;
    config.obstacle_margin = 0.05;
    FrontEndOMPLRRTStar frontEndOMPLRRTStar(config);

    Eigen::Matrix<double,7,1> start;
    start << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double,7,1> end;
    end << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0;
    Eigen::Matrix<double,Eigen::Dynamic,7> desired_trajectory; 
    frontEndOMPLRRTStar.Plan(start,end,desired_trajectory);
    // std::cerr << desired_trajectory << std::endl;

    return 0;

}

