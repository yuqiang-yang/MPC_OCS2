#pragma once
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <fstream>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>


#include <ESDFMap.h>
#include <graceful_mpc/costs/PointsOnRobot.h>



using namespace ompl;
namespace graceful_mpc{
// Define the compound state space
class MobileManipulatorStateSpace : public base::CompoundStateSpace
{
public:
    class StateType : public CompoundStateSpace::StateType
    {
    public:   
        StateType() = default;
    };
    MobileManipulatorStateSpace() : base::CompoundStateSpace()
    {
        addSubspace(std::make_shared<base::DubinsStateSpace>(), 1.0);
        addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 1.0);
    }

    // Set the bounds for the state space
    void setBounds(double x_min, double x_max, double y_min, double y_max,
                   double yaw_min, double yaw_max, double j1_min, double j1_max,
                   double j2_min, double j2_max, double j3_min, double j3_max,
                   double j4_min, double j4_max, double j5_min, double j5_max,
                   double j6_min, double j6_max)
    {
        auto se2_space = std::static_pointer_cast<base::DubinsStateSpace>(getSubspace(0));
        base::RealVectorBounds base_bound(2);
        base::RealVectorBounds arm_bound(6);
        base_bound.setLow(-10);
        base_bound.setHigh(10);
        arm_bound.setLow(-6.28);
        arm_bound.setHigh(6.28);
        se2_space->setBounds(base_bound);
        auto real_vector_space = std::static_pointer_cast<base::RealVectorStateSpace>(getSubspace(1));
        real_vector_space->setBounds(arm_bound);
    }
};
struct stateValidateCheckerConfig
{
    double max_distance;
    double obstacle_margin;
    std::shared_ptr<const PointsOnRobot> pointsOnRobot;
    std::shared_ptr<fiesta::ESDFMap> esdfMap;
};

// Define a custom state validity checker
class MobileManipulatorStateValidityChecker : public base::StateValidityChecker
{
public:
    MobileManipulatorStateValidityChecker(const base::SpaceInformationPtr& si,const stateValidateCheckerConfig config) : base::StateValidityChecker(si),pointsOnRobot_(config.pointsOnRobot),esdfMap_(config.esdfMap)
    {
        max_distance_ = config.max_distance;
        obstacle_margin_ = config.obstacle_margin;
    }

    // Check if a state is valid
    bool isValid(const base::State* state) const override
    {
        // Extract the SE2 component of the state
        auto se2_state = state->as<base::CompoundState>()->as<base::DubinsStateSpace::StateType>(0);
        double x = se2_state->getX();
        double y = se2_state->getY();
        double yaw = se2_state->getYaw();

        // Extract the 6-DoF manipulator component of the state
        auto real_vector_state = state->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
        double j1 = real_vector_state->values[0];
        double j2 = real_vector_state->values[1];
        double j3 = real_vector_state->values[2];
        double j4 = real_vector_state->values[3];
        double j5 = real_vector_state->values[4];
        double j6 = real_vector_state->values[5];
        

#define MY_2_PI 6.28
        // Check if the 6-DoF manipulator component is within bounds
 if (j1 < -MY_2_PI || j1 > MY_2_PI || j2 < -MY_2_PI || j2 > MY_2_PI || j3 < -MY_2_PI || j3 > MY_2_PI ||
        j4 < -MY_2_PI || j4 > MY_2_PI || j5 < -MY_2_PI || j5 > MY_2_PI || j6 < -MY_2_PI || j6 > MY_2_PI)
        return false;

    // TODO: Perform collision checking between the mobile manipulator and obstacles
    Eigen::Matrix<double,17,1> xx;
    xx << x,y,yaw,j1,j2,j3,j4,j5,j6,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    Eigen::VectorXd positionsPointsOnRobot = pointsOnRobot_->getPoints(xx);
    assert(positionsPointsOnRobot.size() % 3 == 0);
    Eigen::VectorXd radii = pointsOnRobot_->getRadii();
    int numPoints = pointsOnRobot_->numOfPoints();
    for (int i = 0; i < numPoints; i++) {
      Eigen::Vector3d gradientFiesta ;
      gradientFiesta << 0.0, 0.0, 0.0;
      Eigen::Matrix<double, 3, 1> position = positionsPointsOnRobot.segment<3>(i * 3);
      double distance = esdfMap_->GetDistWithGradTrilinear(position,gradientFiesta);
      if(distance != -1 && distance - radii(i) - obstacle_margin_< 0){
          return false;
      }
    }

    // If the state is valid, return true
    return true;
}
    double clearance(const base::State* state)
    {
        // Extract the SE2 component of the state
        auto se2_state = state->as<base::CompoundState>()->as<base::DubinsStateSpace::StateType>(0);
        double x = se2_state->getX();
        double y = se2_state->getY();
        double yaw = se2_state->getYaw();

        // Extract the 6-DoF manipulator component of the state
        auto real_vector_state = state->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
        double j1 = real_vector_state->values[0];
        double j2 = real_vector_state->values[1];
        double j3 = real_vector_state->values[2];
        double j4 = real_vector_state->values[3];
        double j5 = real_vector_state->values[4];
        double j6 = real_vector_state->values[5];    
        double res_clearance = 0.;
        // TODO: Perform collision checking between the mobile manipulator and obstacles
        Eigen::Matrix<double,17,1> xx;
        xx << x,y,yaw,j1,j2,j3,j4,j5,j6,Eigen::Matrix<double,9,1>::Zero();
        Eigen::VectorXd positionsPointsOnRobot = pointsOnRobot_->getPoints(xx);
        assert(positionsPointsOnRobot.size() % 3 == 0);
        Eigen::VectorXd radii = pointsOnRobot_->getRadii();
        int numPoints = pointsOnRobot_->numOfPoints();
        for (int i = 0; i < numPoints; i++) {
        Eigen::Vector3d gradientFiesta ;
        gradientFiesta << 0.0, 0.0, 0.0;
        Eigen::Matrix<double, 3, 1> position = positionsPointsOnRobot.segment<3>(i * 3);
        double distance = esdfMap_->GetDistWithGradTrilinear(position,gradientFiesta);
        if(distance != -1){
            res_clearance += distance;
        }
        else{
            res_clearance += max_distance_;

        }
        }    
    }
private:
    std::shared_ptr<const PointsOnRobot> pointsOnRobot_;
    std::shared_ptr<fiesta::ESDFMap> esdfMap_;
    double max_distance_;
    double obstacle_margin_;

};
} //graceful_mpc