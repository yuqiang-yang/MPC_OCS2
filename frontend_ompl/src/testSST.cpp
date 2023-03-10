#include <iostream>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/ControlSpace.h>
#include <fstream>

namespace ob = ompl::base;
namespace oc = ompl::control;
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
        addSubspace(std::make_shared<base::SE2StateSpace>(), 1.0);
        addSubspace(std::make_shared<base::RealVectorStateSpace>(6), 1.0);
    }

    // Set the bounds for the state space
    void setBounds(double x_min, double x_max, double y_min, double y_max,
                   double yaw_min, double yaw_max, double j1_min, double j1_max,
                   double j2_min, double j2_max, double j3_min, double j3_max,
                   double j4_min, double j4_max, double j5_min, double j5_max,
                   double j6_min, double j6_max)
    {
        auto se2_space = std::static_pointer_cast<base::SE2StateSpace>(getSubspace(0));
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

// Define the control space
class MobileManipulatorControlSpace : public control::CompoundControlSpace
{
public:
    class ControlType : public control::CompoundControlSpace::ControlType
    {
    public:   
        ControlType() = default;
    };
    MobileManipulatorControlSpace(const base::StateSpacePtr &stateSpace) : control::CompoundControlSpace(stateSpace)
    {
        addSubspace(std::make_shared<control::RealVectorControlSpace>(stateSpace,2));
        addSubspace(std::make_shared<control::RealVectorControlSpace>(stateSpace,6));
    }

    // Set the bounds for the state space
    void setBounds()
    {
        auto se2_space = std::static_pointer_cast<control::RealVectorControlSpace>(getSubspace(0));
        base::RealVectorBounds base_bound(2);
        base::RealVectorBounds arm_bound(6);
        base_bound.setLow(-2);
        base_bound.setHigh(2);
        arm_bound.setLow(-1);
        arm_bound.setHigh(1);
        se2_space->setBounds(base_bound);
        auto real_vector_space = std::static_pointer_cast<control::RealVectorControlSpace>(getSubspace(1));
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
        auto se2_state = state->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
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



        // Check if the 6-DoF manipulator component is within bounds
 if (j1 < -M_PI_2 || j1 > M_PI_2 || j2 < -M_PI_2 || j2 > M_PI_2 || j3 < -M_PI_2 || j3 > M_PI_2 ||
        j4 < -M_PI_2 || j4 > M_PI_2 || j5 < -M_PI_2 || j5 > M_PI_2 || j6 < -M_PI_2 || j6 > M_PI_2)
        return false;

    // TODO: Perform collision checking between the mobile manipulator and obstacles

    // If the state is valid, return true
    return true;
}};
class MobileManipulatorPropagator : public oc::StatePropagator
{
public:
    MobileManipulatorPropagator(const oc::SpaceInformationPtr& si) : oc::StatePropagator(si)
    {
        si_ = si;
    }

    void propagate(const ob::State* state, const oc::Control* control, const double duration, ob::State* result) const override
    {
        auto carState = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
        auto manipulatorState = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    
        const auto carControl = control->as<oc::CompoundControl>()->as<oc::RealVectorControlSpace::ControlType>(0)->values;
        const auto manipulatorControl = control->as<oc::CompoundControl>()->as<oc::RealVectorControlSpace::ControlType>(1)->values;

        auto carResult = result->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
        auto manipulatorResult = result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

        // Propagate the car state
        carResult->setX(carState->getX() + duration * carControl[0] * cos(carState->getYaw()));
        carResult->setY(carState->getY() + duration * carControl[0] * sin(carState->getYaw()));

        carResult->setYaw(carState->getYaw() + duration * carControl[1]);
        // Propagate the remaining DOFs using Euler integration
        for (unsigned int i = 1; i < 6; ++i)
        {
            manipulatorResult->values[i] = manipulatorState->values[i] + duration * manipulatorControl[i-1];
        }
        si_->getStateSpace()->enforceBounds(result);

}
private:
    oc::SpaceInformationPtr si_;
};

void writePathToFile(const std::string& filename, const ompl::control::PathControl& path)
{
    std::ofstream outFile(filename);
    if (outFile.is_open())
    {
        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            const MobileManipulatorStateSpace::StateType* state = static_cast<const MobileManipulatorStateSpace::StateType *>(path.getState(i));
            auto car_state = state->as<base::SE2StateSpace::StateType>(0);
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
    auto spaceC = std::make_shared<MobileManipulatorControlSpace>(space);
    space->setBounds(-9999.0, 9999.0, -9999.0, 9999.0, -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI,
    -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI);
    spaceC->setBounds();
    // Define the space information
    auto si = std::make_shared<control::SpaceInformation>(space,spaceC);

    si->setStateValidityChecker(std::make_shared<MobileManipulatorStateValidityChecker>(si));
    si->setStatePropagator(std::make_shared<MobileManipulatorPropagator>(si));
    
    si->setup();

    // Define the problem definition
    auto pdef = std::make_shared<control::SimpleSetup>(si);

    // Define the start state
    ompl::base::ScopedState<MobileManipulatorStateSpace> start = base::ScopedState<MobileManipulatorStateSpace>(si);
    start->as<base::SE2StateSpace::StateType>(0)->setXY(0.0, 0.0);
    start->as<base::SE2StateSpace::StateType>(0)->setYaw(0.0);
    start->as<base::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>(1)->values[1] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>(1)->values[2] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>(1)->values[3] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>(1)->values[4] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>(1)->values[5] = 0.0;
    pdef->setStartState(start);
    // Define the goal state
    ompl::base::ScopedState<MobileManipulatorStateSpace> goal = base::ScopedState<MobileManipulatorStateSpace>(si);
    goal->as<base::SE2StateSpace::StateType>(0)->setXY(1.0, 1.0);
    goal->as<base::SE2StateSpace::StateType>(0)->setYaw(M_PI / 2.0);
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[0] = 1.0;
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[1] = 1.0;
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[2] = 1.0;
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[3] = 1.0;
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[4] = 1.0;
    goal->as<base::RealVectorStateSpace::StateType>(1)->values[5] = 1.0;
    pdef->setGoalState(goal);

    // Define the planner
    auto planner = std::make_shared<control::SST>(si);
    pdef->setPlanner(planner);

    // Solve the problem
    auto result = pdef->solve(60);
    if(result)
    {
        // Get the solution path
        auto path = pdef->getSolutionPath();
        
        // Print the solution path
        std::cout << "Solution path: ";
        path.print(std::cout);
        writePathToFile("path.txt",path);
    }
    else
    {
        std::cout << "no solution found" << std::endl;
    }
    return 0;
}