/******************************************************************************
Copyright (c) 2022, Yuqiang Yang . All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include "ocs2_core/cost/CostFunctionBaseAD.h"
#include <graceful_mpc/kinematics/KinematicsInterface.hpp>
#include "graceful_mpc/Definitions.h"
namespace graceful_mpc {

struct FrontOrientationCostConfig{
    double weight;
    std::shared_ptr<const KinematicsInterface<CppAD::AD<CppAD::cg::CG<double>>>> kinematics;
   
};

class FrontOrientationCost : public ocs2::CostFunctionBaseAD<Definitions::STATE_DIM_, Definitions::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::CostFunctionBaseAD<Definitions::STATE_DIM_, Definitions::INPUT_DIM_>;
  using typename BASE::ad_dynamic_vector_t;
  using typename BASE::ad_scalar_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  FrontOrientationCost(const FrontOrientationCostConfig& config)
      : BASE(), weight_(config.weight),kinematics_(config.kinematics)  {}

  FrontOrientationCost(const FrontOrientationCost& rhs) = default;

  ~FrontOrientationCost() override = default;

  FrontOrientationCost* clone() const override { return new FrontOrientationCost(*this); }

 protected:
  /**
   * Interface method to the intermediate cost function.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector.
   * @param [in] stateDesired: desired state vector.
   * @param [in] inputDesired: desired input vector.
   * @param [in] logicVariable: logic variable vector.
   * @param [out] costValue: cost value.
   */
  virtual void intermediateCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                const ad_dynamic_vector_t& parameters, ad_scalar_t& costValue) const override;

  void terminalCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& parameters,
                            ad_scalar_t& costValue) const override;

 private:
    int weight_;
    const std::shared_ptr<const KinematicsInterface<CppAD::AD<CppAD::cg::CG<double>>>> kinematics_;
    
}; // class

}  // namespace graceful_mpc
