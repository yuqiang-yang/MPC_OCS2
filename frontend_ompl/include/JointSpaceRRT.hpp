#pragma once

#include <ros/ros.h>
#include <Eigen/Core>

#include "DubinsWholebodyFrontEnd.hpp"
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include "Fiesta.h"
#include "timing.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace graceful_mpc{
    struct JointSpaceRRTConfig{
    public:
    double planning_time;
    double collisionCheckerResolution;
    double max_distance;
    double obstacle_margin;
    double distance_gain;
    std::shared_ptr<const PointsOnRobot> pointsOnRobot;
    std::shared_ptr<fiesta::ESDFMap> esdf_map;
};

 class ClearanceObjective : public ob::StateCostIntegralObjective
 {
 public:
     ClearanceObjective(const ob::SpaceInformationPtr& si) :
         ob::StateCostIntegralObjective(si, true)
     {
     }
     ob::Cost stateCost(const ob::State* s) const override
     {
         return ob::Cost(1 / (si_->getStateValidityChecker()->clearance(s) +
             std::numeric_limits<double>::min()));
     }

 };
class JointSpaceRRT{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        JointSpaceRRT(JointSpaceRRTConfig& config);
        bool Plan(const Eigen::Matrix<double,9,1>& start,const Eigen::Matrix<double,9,1>& goal,Eigen::Matrix<double,Eigen::Dynamic,9>& desired_trajectory);
    protected:
        ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);
        ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
        {
            auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
            auto clearObj(std::make_shared<ClearanceObjective>(si));
        
            return lengthObj + distance_gain_*clearObj;
        }
        std::shared_ptr<fiesta::ESDFMap> esdf_map_;
        std::shared_ptr<MobileManipulatorStateSpace> space_;
        ob::SpaceInformationPtr si_;
        ob::ProblemDefinitionPtr pdef_;
        using RRTXStaticPtr = std::shared_ptr<ompl::geometric::RRTXstatic>;
        using RRTConnectPtr = std::shared_ptr<ompl::geometric::RRTConnect>;
        using RRTPtr = std::shared_ptr<ompl::geometric::RRT>;

        // ob::PlannerPtr optimizingPlanner_;
        // RRTstarPtr optimizingPlanner_;
        // RRTXStaticPtr optimizingPlanner_;
        // RRTConnectPtr optimizingPlanner_;
        RRTPtr optimizingPlanner_;
        
        double max_distance_;
        double distance_gain_;
        double planning_time_;
        double obstacle_margin_;
        struct JointSpaceRRTConfig config_;
};


  

}// namespace graceful mpc