#include <FrontEndOMPLRRTStar.hpp>
using namespace graceful_mpc;


FrontEndOMPLRRTStar::FrontEndOMPLRRTStar(FrontEndOMPLRRTStarConfig& config)
:config_(config),esdf_map_(config.esdf_map),margin_x_(config.margin_x),margin_y_(config.margin_y),margin_z_(config.margin_z),planning_time_(config.planning_time),obstacle_margin_(config.obstacle_margin),distance_gain_(config.distance_gain),maxVel_(config.maxVel)
{

}
//the head 4 value is quaternion. the tail 3 value is position
bool FrontEndOMPLRRTStar::Plan(const Eigen::Matrix<double,7,1>& start,const Eigen::Matrix<double,7,1>& goal,Eigen::Matrix<double,Eigen::Dynamic,7>& position_trajectory,Eigen::VectorXd& time_trajectory,Eigen::Matrix<double,Eigen::Dynamic,3>& velocity_trajectory)
{
    const int VISUALIZE_CNT = 100;
    ob::RealVectorBounds bound(3);

    double min_x = start.coeff(4) < goal.coeff(4)?start.coeff(4):goal.coeff(4);
    double max_x = start.coeff(4) > goal.coeff(4)?start.coeff(4):goal.coeff(4);
    bound.setLow(0,min_x-config_.margin_x);
    bound.setHigh(0,max_x+config_.margin_x);

    double min_y = start.coeff(5) < goal.coeff(5)?start.coeff(5):goal.coeff(5);
    double max_y = start.coeff(5) > goal.coeff(5)?start.coeff(5):goal.coeff(5);
    bound.setLow(1,min_y-config_.margin_y);
    bound.setHigh(1,max_y+config_.margin_y);

    double min_z = start.coeff(6) < goal.coeff(6)?start.coeff(6):goal.coeff(6);
    double max_z = start.coeff(6) > goal.coeff(6)?start.coeff(6):goal.coeff(6);
    bound.setLow(2,min_z-config_.margin_z);
    bound.setHigh(2,max_z+config_.margin_z);

    space_.reset(new ob::RealVectorStateSpace(3));
    space_->as<ob::RealVectorStateSpace>()->setBounds(bound); 

    si_.reset(new ob::SpaceInformation(space_));
    si_->setStateValidityCheckingResolution(config_.collisionCheckerResolution);
    si_->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si_,esdf_map_,obstacle_margin_)));
    si_->setup();
    std::cout << "resolution"<<si_->getStateValidityCheckingResolution() << std::endl;
    pdef_.reset(new ob::ProblemDefinition(si_));
    pdef_->setOptimizationObjective(getBalancedObjective(si_));
    
    auto optimizingPlanner_ = std::make_shared<og::RRTXstatic>(si_);

    
    // optimizingPlanner_->setRange(config_.edgeLength);   fuck!! make me stuck in the bug for 2 days.


    ob::ScopedState<> plan_start(space_);
    
    plan_start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start.coeff(4);
    plan_start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start.coeff(5);
    plan_start->as<ob::RealVectorStateSpace::StateType>()->values[2] = start.coeff(6);
    ob::ScopedState<> plan_end(space_);
    plan_end->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal.coeff(4);
    plan_end->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal.coeff(5);
    plan_end->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal.coeff(6);

    pdef_->setStartAndGoalStates(plan_start, plan_end);


    ob::PlannerData pData(si_);
    if(!si_->getStateValidityChecker()->isValid(plan_end->as<ob::RealVectorStateSpace::StateType>()))
    {
        std::cerr << "invalid goal state!!!!!!!"  << si_->getStateValidityChecker()->clearance(plan_end->as<ob::RealVectorStateSpace::StateType>())<< std::endl;
        return false;
    }
    else
    {
        std::cerr << "valid goal with clearance" << si_->getStateValidityChecker()->clearance(plan_end->as<ob::RealVectorStateSpace::StateType>()) << std::endl;
    }

    optimizingPlanner_->setProblemDefinition(pdef_);
    optimizingPlanner_->setup();
    optimizingPlanner_->printSettings(std::cout);

    ob::PlannerStatus solved = optimizingPlanner_->ob::Planner::solve(planning_time_);
        std::cout
            << optimizingPlanner_->getName()
            << " found a solution of length "
            << pdef_->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef_->getSolutionPath()->cost(pdef_->getOptimizationObjective()) << std::endl;



    int cnt = 0;
    while(solved != ob::PlannerStatus::EXACT_SOLUTION) //solved means the OMPL find an exact result
    {   
        if(cnt == 3) return false;
        cnt++;
        optimizingPlanner_->getPlannerData(pData);
        // std::cerr << "Cannot find solution: " << "with vertices " << pData.numVertices() << "   iterations:" << optimizingPlanner_->numIterations() << std::endl;
        solved = optimizingPlanner_->ob::Planner::solve(planning_time_);
    }
        // std::cerr << "find solution successfully : " << "with vertices " << pData.numVertices() << "   iterations:" << optimizingPlanner_->numIterations() << std::endl;

    //position path is solve by RRT*
    std::vector<ob::State *> states = std::static_pointer_cast<og::PathGeometric>(pdef_->getSolutionPath())->getStates();
    size_t rows = std::static_pointer_cast<og::PathGeometric>(pdef_->getSolutionPath())->getStateCount();

    // pdef_->getSolutionPath()->print(std::cout);
    std::size_t count = 0;
    Eigen::Matrix<double,Eigen::Dynamic,7> desired_trajectory;
    position_trajectory.resize(VISUALIZE_CNT,7);
    velocity_trajectory.resize(VISUALIZE_CNT,3);
    desired_trajectory.resize(rows,7);
    time_trajectory.resize(VISUALIZE_CNT);

    for(ob::State* state: states){
    const ob::RealVectorStateSpace::StateType* state3D =
        state->as<ob::RealVectorStateSpace::StateType>();
        desired_trajectory(count,4) = state3D->values[0]; 
        desired_trajectory(count,5) = state3D->values[1]; 
        desired_trajectory(count,6) = state3D->values[2]; 
        count++;
    }

    // minco minco minco minco to make the path smooth
    Eigen::Matrix3d initState = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d finalState = Eigen::Matrix3d::Zero();  
    minco::MINCO_S3NU minco;
    for(int i = 0;i < rows;i++){
        std::cerr << desired_trajectory.row(i) << std::endl;
    }
    initState.col(0) = desired_trajectory.row(0).tail<3>();
    finalState.col(0) = desired_trajectory.row(rows-1).tail<3>();


    minco.setConditions(initState,finalState,count-1);  //piece num = pointNum -1
    Eigen::Matrix3Xd points;
    points.resize(3,count-2);
    Eigen::VectorXd times;
    times.resize(count-1);
    for(int i = 0; i < count-2;i++){
        points.col(i) = desired_trajectory.row(i+1).tail<3>();        
        times[i] = (desired_trajectory.row(i+1).tail<3>() - desired_trajectory.row(i).tail<3>()).norm()/maxVel_;
    }
    times[count-2] = (desired_trajectory.row(count-1).tail<3>() - desired_trajectory.row(count-2).tail<3>()).norm()/maxVel_;

    // std::cerr << "rows" << std::endl << rows<<std::endl;
    // std::cerr << "desired_trajectory" << std::endl << desired_trajectory<<std::endl;

    minco.setParameters(points,times);
    Trajectory<5> traj;
    traj.clear();
    minco.getTrajectory(traj);
    // std::cerr << traj.getTotalDuration() << " total duration" << std::endl;

    double resolution = traj.getTotalDuration()/(double)VISUALIZE_CNT;
    int tmp = 0; 

    for(double t = 0; t <traj.getTotalDuration() && tmp < VISUALIZE_CNT;t+=resolution){
        position_trajectory.row(tmp).tail<3>()=traj.getPos(t);
        velocity_trajectory.row(tmp).tail<3>()=traj.getVel(t);
       
        time_trajectory(tmp) = t;
        tmp++;
    }
    std::cerr << std::endl;

    // //rotation path is solved by interpolation

    // //the rotation state in this project is (x y z w)
    Eigen::Quaterniond start_quat(start.coeff(3),start.coeff(0),start.coeff(1),start.coeff(2));  //w x y z
    Eigen::Quaterniond goal_quat(goal.coeff(3),goal.coeff(0),goal.coeff(1),goal.coeff(2));  //w x y z
    std::vector<double> interpolate_t(VISUALIZE_CNT);
    for(int i = 0;i < VISUALIZE_CNT;i++){
        interpolate_t[i] = 1.0/(VISUALIZE_CNT-1) * i;
        Eigen::Quaterniond tmp = start_quat.slerp(interpolate_t[i],goal_quat);
        position_trajectory(i,0) =  tmp.coeffs()(0);
        position_trajectory(i,1) =  tmp.coeffs()(1);
        position_trajectory(i,2) =  tmp.coeffs()(2);
        position_trajectory(i,3) =  tmp.coeffs()(3);
    }

    return solved;

}
ob::OptimizationObjectivePtr FrontEndOMPLRRTStar::getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}


bool ValidityChecker::isValid(const ob::State* state) const
{
    // return true;
    return clearance(state) > obstacle_margin_;
}

double ValidityChecker::clearance(const ob::State* state) const{
    // return 9.0;
    const ob::RealVectorStateSpace::StateType* state3D =
    state->as<ob::RealVectorStateSpace::StateType>();
  
    Eigen::Vector3d position;
    position[0] = state3D->values[0];
    position[1] = state3D->values[1];
    position[2] = state3D->values[2];

    Eigen::Vector3d gradientFiesta ;
    gradientFiesta << 0.0, 0.0, 0.0;
    if(!esdf_map_) return 3.0;
    double distance = esdf_map_->GetDistWithGradTrilinear(position,gradientFiesta);
    if (distance != -1) {
        // std::cerr << "the query point distance is " << distance << std::endl;
        // std::cerr << "the query point gradient is " << gradientVoxblox.transpose() << std::endl;
        return distance;
    }
    else
    {
        std::cerr << "esdf_map misses the point"  <<position.transpose() << std::endl;
        return 3.0;
    }

}




