#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <fstream>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>



using namespace ompl;

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
        addSubspace(std::make_shared<base::ReedsSheppStateSpace>(), 1.0);
        addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 1.0);
    }

    // Set the bounds for the state space
    void setBounds(double x_min, double x_max, double y_min, double y_max,
                   double yaw_min, double yaw_max, double j1_min, double j1_max,
                   double j2_min, double j2_max, double j3_min, double j3_max,
                   double j4_min, double j4_max, double j5_min, double j5_max,
                   double j6_min, double j6_max)
    {
        auto se2_space = std::static_pointer_cast<base::ReedsSheppStateSpace>(getSubspace(0));
        base::RealVectorBounds base_bound(2);
        base::RealVectorBounds arm_bound(6);
        base_bound.setLow(-100);
        base_bound.setHigh(100);
        arm_bound.setLow(-6.28);
        arm_bound.setHigh(6.28);
        se2_space->setBounds(base_bound);
        auto real_vector_space = std::static_pointer_cast<base::RealVectorStateSpace>(getSubspace(1));
        real_vector_space->setBounds(arm_bound);
    }
};

// Define a custom state validity checker
class MobileManipulatorStateValidityChecker : public base::StateValidityChecker
{
public:
    MobileManipulatorStateValidityChecker(const base::SpaceInformationPtr& si) : base::StateValidityChecker(si)
    {
    }

    // Check if a state is valid
    bool isValid(const base::State* state) const override
    {
        // Extract the SE2 component of the state
        auto se2_state = state->as<base::CompoundState>()->as<base::ReedsSheppStateSpace::StateType>(0);
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

    // If the state is valid, return true
    return true;
}};

void writePathToFile(const std::string& filename, const ompl::geometric::PathGeometric& path)
{
    std::ofstream outFile(filename);
    if (outFile.is_open())
    {
        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            const MobileManipulatorStateSpace::StateType* state = static_cast<const MobileManipulatorStateSpace::StateType *>(path.getState(i));
            auto car_state = state->as<base::ReedsSheppStateSpace::StateType>(0);
            auto arm_state = state->as<base::RealVectorStateSpace::StateType>(1);


            outFile << car_state->getX() << " " << car_state->getY() << " " << car_state->getYaw() << " "
                    << arm_state->values[0] << " " << arm_state->values[1] << " " << arm_state->values[2] << " " << arm_state->values[3] << " " << arm_state->values[4] << " " << arm_state->values[5]<< std::endl;
        }
        outFile.close();
        std::cout << "Successfully wrote the path to file: " << filename << std::endl;
    }
    else
    {
        std::cout << "Unable to open file: " << filename << std::endl;
    }
}
int main()
{
    // Define the state space
    auto space = std::make_shared<MobileManipulatorStateSpace>();
    space->setBounds(-9999.0, 9999.0, -9999.0, 9999.0, -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI,
    -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI);
    // Define the space information
    auto si = std::make_shared<base::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<MobileManipulatorStateValidityChecker>(si));
    si->setup();

    // Define the problem definition
    auto pdef = std::make_shared<geometric::SimpleSetup>(si);

    // Define the start state
    ompl::base::ScopedState<MobileManipulatorStateSpace> start = base::ScopedState<MobileManipulatorStateSpace>(si);
    start->as<base::ReedsSheppStateSpace::StateType>(0)->setXY(0.0, 0.0);
    start->as<base::ReedsSheppStateSpace::StateType>(0)->setYaw(0.0);
    start->as<base::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>(1)->values[1] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>(1)->values[2] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>(1)->values[3] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>(1)->values[4] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>(1)->values[5] = 0.0;
    pdef->setStartState(start);
    // Define the goal state
    ompl::base::ScopedState<MobileManipulatorStateSpace> goal = base::ScopedState<MobileManipulatorStateSpace>(si);
    goal->as<base::ReedsSheppStateSpace::StateType>(0)->setXY(1.0, 1.0);
    goal->as<base::ReedsSheppStateSpace::StateType>(0)->setYaw(M_PI / 2.0);
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[0] = 1.0;
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[1] = 1.0;
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[2] = 1.0;
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[3] = 1.0;
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[4] = 1.0;
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[5] = 1.0;
    pdef->setGoalState(goal);

    // Define the planner
    auto planner = std::make_shared<geometric::RRT>(si);
    planner->setIntermediateStates(true);
    pdef->setPlanner(planner);

    // Solve the problem
    pdef->solve();

    // Get the solution path
    auto path = pdef->getSolutionPath();
    // Print the solution path
    std::cout << "Solution path: ";
    path.print(std::cout);
    writePathToFile("path.txt",path);
    return 0;
}