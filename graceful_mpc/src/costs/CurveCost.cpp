#include <graceful_mpc/costs/CurveCost.h>
#include <graceful_mpc/costs/InterpolatePoseTrajectory.h>

using namespace ocs2;
using namespace graceful_mpc;
using std::abs;
#define myeps(x) (x >= 0? (x+eps_):(x-eps_))
#define mybound(x) (abs(x) > 2? (x/abs(x)*1.0):(x))
void CurveCost::getIntermediateCost(CurveCost::scalar_t& L) {
  auto w = x_.tail<Definitions::VELOCITY_STATE_DIM_>()[1];
  auto v = x_.tail<Definitions::VELOCITY_STATE_DIM_>()[0];
  L = (weight_ * w*w/myeps(v*v));

}
void CurveCost::getIntermediateCostDerivativeState(CurveCost::state_vector_t& dLdx) {
  auto w = x_.tail<Definitions::VELOCITY_STATE_DIM_>()[1];
  auto v = x_.tail<Definitions::VELOCITY_STATE_DIM_>()[0]; 
  dLdx.tail<Definitions::VELOCITY_STATE_DIM_>()[0] = -weight_*2.0*w*w/myeps(v*v*v);
  dLdx.tail<Definitions::VELOCITY_STATE_DIM_>()[1] = mybound(weight_*2.0*w/myeps(v*v));

  //  std::cerr << "v:"<< v << " w:" << w << " d1[0]:" << dLdx.tail<Definitions::VELOCITY_STATE_DIM_>()[0] <<  " d1[1]:" << dLdx.tail<Definitions::VELOCITY_STATE_DIM_>()[1] << std::endl;
  // std::cerr << "dLd1" << std::endl << dLdx.transpose() <<std::endl;

}

void CurveCost::getIntermediateCostSecondDerivativeState(CurveCost::state_matrix_t& dLdxx) {
  auto w = x_.tail<Definitions::VELOCITY_STATE_DIM_>()[1];
  auto v = x_.tail<Definitions::VELOCITY_STATE_DIM_>()[0]; 
  // dLdxx(Definitions::POSITION_STATE_DIM_,Definitions::POSITION_STATE_DIM_) = weight_*6.0*w*w/myeps(v*v*v*v);
  // dLdxx(Definitions::POSITION_STATE_DIM_+1,Definitions::POSITION_STATE_DIM_) = -weight_*4.0*w/myeps(v*v*v);
  // dLdxx(Definitions::POSITION_STATE_DIM_,Definitions::POSITION_STATE_DIM_+1) = -weight_*4.0*w/myeps(v*v*v);
  // dLdxx(Definitions::POSITION_STATE_DIM_+1,Definitions::POSITION_STATE_DIM_+1) = weight_*2.0/myeps(v*v);
  // std::cerr << "dLdxx" << std::endl << dLdxx <<std::endl;
  //  std::cerr << "v:"<< v << " w:" << w << " dxx[0,0]:" << weight_*6.0*w*w/myeps(v*v*v*v) <<  " d1[1,1]:" << weight_*2.0/myeps(v*v) << std::endl;

}
void CurveCost::getIntermediateCostDerivativeInput(CurveCost::input_vector_t& dLdu) {
  dLdu = input_vector_t::Zero();
}
void CurveCost::getIntermediateCostSecondDerivativeInput(CurveCost::input_matrix_t& dLduu) {
  dLduu = input_matrix_t::Zero();
}
void CurveCost::getIntermediateCostDerivativeInputState(CurveCost::input_state_matrix_t& dLdux) {
  dLdux = input_state_matrix_t::Zero();
}
void CurveCost::getTerminalCost(CurveCost::scalar_t& Phi) {
  auto w = x_.tail<Definitions::VELOCITY_STATE_DIM_>()[1];
  auto v = x_.tail<Definitions::VELOCITY_STATE_DIM_>()[0];
  Phi = scalar_t(0);
  // Phi = weight_ * w/(v+1e-3);
}
void CurveCost::getTerminalCostDerivativeState(CurveCost::state_vector_t& dPhidx) {
  auto w = x_.tail<Definitions::VELOCITY_STATE_DIM_>()[1];
  auto v = x_.tail<Definitions::VELOCITY_STATE_DIM_>()[0]; 
  dPhidx = state_vector_t::Zero();
  // dPhidx.tail<Definitions::VELOCITY_STATE_DIM_>()[0] = -weight_*w/v/v;
  // dPhidx.tail<Definitions::VELOCITY_STATE_DIM_>()[1] = weight_*1/v;
}

void CurveCost::getTerminalCostSecondDerivativeState(CurveCost::state_matrix_t& dPhidxx) {
  dPhidxx = state_matrix_t::Zero();
}
void CurveCost::getIntermediateCostDerivativeTime(CostFunctionBase::scalar_t& dLdt) {
  dLdt = 0;
}
void CurveCost::getTerminalCostDerivativeTime(CostFunctionBase::scalar_t& dPhidt) {
  dPhidt = 0;
}
