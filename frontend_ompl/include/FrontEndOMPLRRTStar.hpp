#pragma once

#include <ros/ros.h>
#include <Eigen/Core>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/PlannerTerminationCondition.h>


// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/rrt/RRTstar.h>


// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include "Fiesta.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace perceptive_mpc{
    struct FrontEndOMPLRRTStarConfig{
    public:
    double planning_time;
    double margin_x;
    double margin_y;
    double margin_z;
    double edgeLength;
    double obstacle_margin;
    double distance_gain;
    std::shared_ptr<fiesta::ESDFMap> esdf_map;
};

 class ClearanceObjective : public ob::StateCostIntegralObjective
 {
 public:
     ClearanceObjective(const ob::SpaceInformationPtr& si) :
         ob::StateCostIntegralObjective(si, true)
     {
     }
  
     // Our requirement is to maximize path clearance from obstacles,
     // but we want to represent the objective as a path cost
     // minimization. Therefore, we set each state's cost to be the
     // reciprocal of its clearance, so that as state clearance
     // increases, the state cost decreases.
     ob::Cost stateCost(const ob::State* s) const override
     {
         return ob::Cost(1 / (si_->getStateValidityChecker()->clearance(s) +
             std::numeric_limits<double>::min()));
     }

 };
    class FrontEndOMPLRRTStar{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            FrontEndOMPLRRTStar(FrontEndOMPLRRTStarConfig& config);
            bool Plan(const Eigen::Matrix<double,7,1>& start,const Eigen::Matrix<double,7,1>& goal,Eigen::Matrix<double,Eigen::Dynamic,7>& desired_trajectory);
        protected:
            ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);
            ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
            {
                auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
                auto clearObj(std::make_shared<ClearanceObjective>(si));
            
                return distance_gain_*lengthObj + clearObj;
            }
            std::shared_ptr<fiesta::ESDFMap> esdf_map_;
            ob::StateSpacePtr space_;
            ob::SpaceInformationPtr si_;
            ob::ProblemDefinitionPtr pdef_;
            using RRTstarPtr = std::shared_ptr<ompl::geometric::RRTstar>;
            // ob::PlannerPtr optimizingPlanner_;
            RRTstarPtr optimizingPlanner_;
            double margin_x_;
            double margin_y_;
            double margin_z_;
            double distance_gain_;
            double planning_time_;
            double obstacle_margin_;
            struct FrontEndOMPLRRTStarConfig config_;
    };

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si, std::shared_ptr<fiesta::ESDFMap> esdf_map,double obstacle_margin) :
        ob::StateValidityChecker(si),obstacle_margin_(obstacle_margin) {esdf_map_ = esdf_map;}
 
    bool isValid(const ob::State* state) const;
    
    double clearance(const ob::State* state) const;
protected:
    std::shared_ptr<fiesta::ESDFMap> esdf_map_;
    double obstacle_margin_;

};




  

}// namespace perceptive mpc