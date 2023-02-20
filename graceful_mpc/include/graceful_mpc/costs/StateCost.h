
#pragma once

#include <ocs2_core/cost/QuadraticGaussNewtonCostBaseAD.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <algorithm>
#include <cmath>
#include <utility>

#include <graceful_mpc/kinematics/KinematicsInterface.hpp>
#include "graceful_mpc/Definitions.h"

// CPPAD stuff
#include "cppad/cg/math.hpp"

namespace graceful_mpc {

struct StateCostConfig {
  Eigen::Matrix<double, Definitions::STATE_DIM_, Definitions::STATE_DIM_> Q = Eigen::Matrix<double, Definitions::STATE_DIM_, Definitions::STATE_DIM_>::Identity();
  Eigen::Matrix<double, Definitions::STATE_DIM_, Definitions::STATE_DIM_> QFinal = Eigen::Matrix<double, Definitions::STATE_DIM_, Definitions::STATE_DIM_>::Zero();
};

class StateCost : public ocs2::CostFunctionBase<Definitions::STATE_DIM_, Definitions::INPUT_DIM_> {
 
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = CostFunctionBase<Definitions::STATE_DIM_, Definitions::INPUT_DIM_>;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  /**
   * Constructor
   */
  StateCost(const StateCostConfig& config)
      : Q_(config.Q), QFinal_(config.QFinal) {}
  ~StateCost() override = default;

  StateCost* clone() const override { return new StateCost(*this); }
  void getIntermediateCost(scalar_t& L) override;
  void getIntermediateCostDerivativeState(state_vector_t& dLdx) override;
  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override;
  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override;
  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override;
  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) override;
  void getTerminalCost(scalar_t& Phi) override;
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) override;
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override;
  void getIntermediateCostDerivativeTime(scalar_t& dLdt) override;
  void getTerminalCostDerivativeTime(scalar_t& dPhidt) override;

  void getIntermediateCostDerivativeStateVerbose(state_vector_t& dLdx);

 private:
  const state_matrix_t Q_;
  const state_matrix_t QFinal_;
};

}  // namespace graceful_mpc