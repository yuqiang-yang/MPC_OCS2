
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

struct CurveCostConfig {
  double weight;
};

class CurveCost : public ocs2::CostFunctionBase<Definitions::STATE_DIM_, Definitions::INPUT_DIM_> {
 
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
  CurveCost(const CurveCostConfig& config)
      : weight_(config.weight),eps_(1e-5) {}
  ~CurveCost() override = default;

  CurveCost* clone() const override { return new CurveCost(*this); }
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
  

 private:
 double weight_;
 const double eps_;
};

}  // namespace graceful_mpc