#pragma once
// #include <perceptive_mpc/EsdfCachingServer.hpp>
#include <voxblox_ros/esdf_server.h>

#include <ros/ros.h>
#include <voxblox/interpolator/interpolator.h>
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
namespace voxblox {
class EsdfCachingVoxel;

template <typename VoxelType>
class Interpolator;
}  // namespace voxblox



namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace perceptive_mpc{
    using typename voxblox::EsdfCachingVoxel;
    using voxblox::Interpolator;
    struct FrontEndOMPLRRTStarConfig{
    public:
    double planning_time;
    double margin_x;
    double margin_y;
    double margin_z;
    double edgeLength;
    double obstacle_margin;
    std::shared_ptr<Interpolator<EsdfCachingVoxel>> interpolator_;
};

    class FrontEndOMPLRRTStar{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            FrontEndOMPLRRTStar(FrontEndOMPLRRTStarConfig& config);
            bool Plan(const Eigen::Matrix<double,7,1>& start,const Eigen::Matrix<double,7,1>& goal,Eigen::Matrix<double,Eigen::Dynamic,7>& desired_trajectory);
        protected:
            ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

            std::shared_ptr<Interpolator<EsdfCachingVoxel>> interpolator_;
            ob::StateSpacePtr space_;
            ob::SpaceInformationPtr si_;
            ob::ProblemDefinitionPtr pdef_;
            using RRTstarPtr = std::shared_ptr<ompl::geometric::RRTstar>;
            // ob::PlannerPtr optimizingPlanner_;
            RRTstarPtr optimizingPlanner_;
            double margin_x_;
            double margin_y_;
            double margin_z_;
            double planning_time_;
            double obstacle_margin_;
            struct FrontEndOMPLRRTStarConfig config_;
    };


class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si, std::shared_ptr<Interpolator<EsdfCachingVoxel>> interpolator,double obstacle_margin) :
        ob::StateValidityChecker(si),obstacle_margin_(obstacle_margin) {interpolator_ = interpolator;}
 
    bool isValid(const ob::State* state) const;
    
    double clearance(const ob::State* state) const;
protected:
    std::shared_ptr<Interpolator<EsdfCachingVoxel>> interpolator_;
    double obstacle_margin_;

};



}// namespace perceptive mpc