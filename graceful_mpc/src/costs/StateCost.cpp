#include <graceful_mpc/costs/StateCost.h>
using namespace ocs2;
using namespace graceful_mpc;



void StateCost::getIntermediateCost(StateCost::scalar_t& L) {

  L = x_.transpose() * Q_ * x_;
}
void StateCost::getIntermediateCostDerivativeState(StateCost::state_vector_t& dLdx) {
  
  dLdx = Q_ * x_ * 2.0;
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
  Phi = x_.transpose() * QFinal_ * x_;
}
void StateCost::getTerminalCostDerivativeState(StateCost::state_vector_t& dPhidx) {
  dPhidx = QFinal_ * x_ * 2.0;
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
