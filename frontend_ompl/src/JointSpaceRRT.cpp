#include <JointSpaceRRT.hpp>
using namespace graceful_mpc;


JointSpaceRRT::JointSpaceRRT(JointSpaceRRTConfig& config)
:config_(config),esdf_map_(config.esdf_map),planning_time_(config.planning_time),obstacle_margin_(config.obstacle_margin),distance_gain_(config.distance_gain)
{
    max_distance_ = config.max_distance;   
}

bool JointSpaceRRT::Plan(const Eigen::Matrix<double,9,1>& startVector,const Eigen::Matrix<double,9,1>& goalVector,Eigen::Matrix<double,Eigen::Dynamic,9>& desired_trajectory)
{
    // Define the state space
    space_ = std::make_shared<MobileManipulatorStateSpace>();
    space_->setBounds(-999.0, 999.0, -999.0, 999.0, -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI,
    -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI);
    // Define the space information
    si_ = std::make_shared<base::SpaceInformation>(space_);
    stateValidateCheckerConfig config;
    config.esdfMap = config_.esdf_map;
    config.pointsOnRobot = config_.pointsOnRobot;
    config.max_distance = config_.max_distance;
    config.obstacle_margin = config_.obstacle_margin;
    auto validCheckcer = std::make_shared<MobileManipulatorStateValidityChecker>(si_,config);
    si_->setStateValidityChecker(validCheckcer);
    si_->setup();

    // Define the problem definition
    auto pdef = std::make_shared<geometric::SimpleSetup>(si_);

    // Define the start state
    ompl::base::ScopedState<MobileManipulatorStateSpace> start = base::ScopedState<MobileManipulatorStateSpace>(si_);
    start->as<base::ReedsSheppStateSpace::StateType>(0)->setXY(startVector[0], startVector[1]);
    start->as<base::ReedsSheppStateSpace::StateType>(0)->setYaw(startVector[2]);
    start->as<base::RealVectorStateSpace::StateType>(1)->values[0] = startVector[3];
    start->as<base::RealVectorStateSpace::StateType>(1)->values[1] = startVector[4];
    start->as<base::RealVectorStateSpace::StateType>(1)->values[2] = startVector[5];
    start->as<base::RealVectorStateSpace::StateType>(1)->values[3] = startVector[6];
    start->as<base::RealVectorStateSpace::StateType>(1)->values[4] = startVector[7];
    start->as<base::RealVectorStateSpace::StateType>(1)->values[5] = startVector[8];
    pdef->setStartState(start);
    // Define the goal state
    ompl::base::ScopedState<MobileManipulatorStateSpace> goal = base::ScopedState<MobileManipulatorStateSpace>(si_);
    goal->as<base::ReedsSheppStateSpace::StateType>(0)->setXY(goalVector[0], goalVector[1]);
    goal->as<base::ReedsSheppStateSpace::StateType>(0)->setYaw(goalVector[2]);
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[0] = goalVector[3];
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[1] = goalVector[4];
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[2] = goalVector[5];
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[3] = goalVector[6];
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[4] = goalVector[7];
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[5] = goalVector[8];
    pdef->setGoalState(goal);
    if(!validCheckcer->isValid(start.get())){
        std::cerr << "frontEnd invalid start" <<  std::endl;
        return false;
    }
    if(!validCheckcer->isValid(goal.get())){
        std::cerr << "frontEnd invalid end" <<  std::endl;
        return false;

    }
    // Define the planner
    auto planner = std::make_shared<geometric::RRT>(si_);
    planner->setIntermediateStates(true);
    planner->printSettings(std::cerr);
    pdef->setPlanner(planner);
    
    // Solve the problem
    pdef->solve(planning_time_);

    // Get the solution path
    auto path = pdef->getSolutionPath();
    // Print the solution path
    // std::cout << "Solution path: ";
    desired_trajectory.resize(path.getStateCount(),9);
    path.print(std::cout);
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        const MobileManipulatorStateSpace::StateType* state = static_cast<const MobileManipulatorStateSpace::StateType *>(path.getState(i));
        auto car_state = state->as<base::ReedsSheppStateSpace::StateType>(0);
        auto arm_state = state->as<base::RealVectorStateSpace::StateType>(1);


        desired_trajectory.row(i) << car_state->getX(),car_state->getY(), car_state->getYaw(), arm_state->values[0], arm_state->values[1],arm_state->values[2], arm_state->values[3], arm_state->values[4],arm_state->values[5];
    }
    return true;
    
}
ob::OptimizationObjectivePtr JointSpaceRRT::getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}
