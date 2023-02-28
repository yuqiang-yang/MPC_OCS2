#include <graceful_mpc/costs/StateCost.h>
#include <graceful_mpc/costs/InterpolatePoseTrajectory.h>

using namespace ocs2;
using namespace graceful_mpc;


void StateCost::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
  x_des_ = state_vector_t::Zero();
  const auto& desiredTimeTrajectory = costDesiredTrajectoriesPtr_->desiredTimeTrajectory();
  const auto& desiredStateTrajectory = costDesiredTrajectoriesPtr_->desiredStateTrajectory();
  double desire_theta = 0;

  auto it = std::lower_bound(desiredTimeTrajectory.begin(), desiredTimeTrajectory.end(), t);
  int timeAIdx = it - desiredTimeTrajectory.begin() - 1;
  if (timeAIdx == -1) {
    desire_theta = desiredStateTrajectory[0].tail<Definitions::VELOCITY_DIM_>()[0];
  } else if (timeAIdx == desiredTimeTrajectory.size() - 1) {
    desire_theta = desiredStateTrajectory[timeAIdx].tail<Definitions::VELOCITY_DIM_>()[0];
  } else {
    double tau = (t - desiredTimeTrajectory[timeAIdx]) / (desiredTimeTrajectory[timeAIdx + 1] - desiredTimeTrajectory[timeAIdx]);
    desire_theta = (1-tau)*desiredStateTrajectory[timeAIdx].tail<Definitions::VELOCITY_DIM_>()[0] +  tau*desiredStateTrajectory[timeAIdx+1].tail<Definitions::VELOCITY_DIM_>()[0];
  }

  x_des_[2] = desire_theta; //set the theta to a user-specific value to guide the mobile manipulator

  BASE::setCurrentStateAndControl(t,x,u);
}

void StateCost::getIntermediateCost(StateCost::scalar_t& L) {

  L = (x_-x_des_).transpose() * Q_ * (x_-x_des_);
}
void StateCost::getIntermediateCostDerivativeState(StateCost::state_vector_t& dLdx) {
  
  dLdx = Q_ * (x_ - x_des_) * 2.0;
}

void StateCost::getIntermediateCostSecondDerivativeState(StateCost::state_matrix_t& dLdxx) {
  dLdxx = Q_ * 2.0;
}
void StateCost::getIntermediateCostDerivativeInput(StateCost::input_vector_t& dLdu) {
  dLdu = input_vector_t::Zero();
}
void StateCost::getIntermediateCostSecondDerivativeInput(StateCost::input_matrix_t& dLduu) {
  dLduu = input_matrix_t::Zero();
}
void StateCost::getIntermediateCostDerivativeInputState(StateCost::input_state_matrix_t& dLdux) {
  dLdux = input_state_matrix_t::Zero();
}
void StateCost::getTerminalCost(StateCost::scalar_t& Phi) {
  Phi = (x_ - x_des_).transpose() * QFinal_ * (x_ - x_des_);
}
void StateCost::getTerminalCostDerivativeState(StateCost::state_vector_t& dPhidx) {
  dPhidx = QFinal_ * (x_ - x_des_) * 2.0;
}
void StateCost::getTerminalCostSecondDerivativeState(StateCost::state_matrix_t& dPhidxx) {
  dPhidxx = QFinal_ * 2.0;
}
void StateCost::getIntermediateCostDerivativeTime(CostFunctionBase::scalar_t& dLdt) {
  dLdt = 0;
}
void StateCost::getTerminalCostDerivativeTime(CostFunctionBase::scalar_t& dPhidt) {
  dPhidt = 0;
}
