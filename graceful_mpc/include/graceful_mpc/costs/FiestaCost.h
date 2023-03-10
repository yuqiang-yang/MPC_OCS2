/*
 * Copyright (c) 2023 Yang yuqiang <yuqiang6149@163.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of this work nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include <graceful_mpc/costs/PointsOnRobot.h>

#include "ocs2_core/cost/CostFunctionBase.h"
#include "graceful_mpc/Definitions.h"
#include <ESDFMap.h>

namespace graceful_mpc {

struct FiestaCostConfig {
  double mu = 1;
  double delta = 1e-3;
  double maxDistance = 2.0;
  double d_threshold; 
  double weight;
  std::shared_ptr<const PointsOnRobot> pointsOnRobot;
  std::shared_ptr<fiesta::ESDFMap> esdfMap;
};

class FiestaCost : public ocs2::CostFunctionBase<Definitions::STATE_DIM_, Definitions::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = CostFunctionBase<Definitions::STATE_DIM_, Definitions::INPUT_DIM_>;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  FiestaCost(const FiestaCostConfig& config)
      : CostFunctionBase(),
        esdfMap_(config.esdfMap),
        maxDistance_(config.maxDistance),
        mu_(config.mu),
        weight_(config.weight),
        d_threshold_(config.d_threshold),
        delta_(config.delta),
        pointsOnRobot_(config.pointsOnRobot),
        distances_(pointsOnRobot_->numOfPoints()),
        gradientsFiesta_(pointsOnRobot_->numOfPoints(), pointsOnRobot_->numOfPoints() * 3),
        gradients_(pointsOnRobot_->numOfPoints(), int(Definitions::STATE_DIM_)) 
        { if(!config.esdfMap) std::cerr << "the config esdf is null ptr" << std::endl;}

  FiestaCost(const FiestaCost& rhs)
      : CostFunctionBase(),
        esdfMap_(rhs.esdfMap_),
        maxDistance_(rhs.maxDistance_),
        mu_(rhs.mu_),
        delta_(rhs.delta_),
        weight_(rhs.weight_),
        d_threshold_(rhs.d_threshold_),        
        pointsOnRobot_(new PointsOnRobot(*rhs.pointsOnRobot_)),
        distances_(rhs.distances_),
        gradientsFiesta_(rhs.gradientsFiesta_),
        gradients_(rhs.gradients_) {}

  /**
   * Destructor
   */
  ~FiestaCost() override = default;

  FiestaCost* clone() const override { return new FiestaCost(*this); }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u);

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
  scalar_t mu_;
  scalar_t delta_;
  scalar_t maxDistance_;
  scalar_t d_threshold_;
  scalar_t weight_;

  std::shared_ptr<const PointsOnRobot> pointsOnRobot_;
  std::shared_ptr<fiesta::ESDFMap> esdfMap_;

  Eigen::Matrix<scalar_t, -1, 1> distances_;
  Eigen::MatrixXd gradientsFiesta_;
  Eigen::Matrix<scalar_t, -1, Definitions::STATE_DIM_> gradients_;
  
  // scalar_t getPenaltyFunctionValue(scalar_t h) const {
  //   if (h > delta_) {
  //     return -mu_ * log(h);
  //   } else {
  //     return mu_ * (-log(delta_) + scalar_t(0.5) * pow((h - 2.0 * delta_) / delta_, 2.0) - scalar_t(0.5));
  //   };
  // };

  // scalar_t getPenaltyFunctionDerivative(scalar_t h) const {
  //   if (h > delta_) {
  //     return -mu_ / h;
  //   } else {
  //     return mu_ * ((h - 2.0 * delta_) / (delta_ * delta_));
  //   };
  // };

  // scalar_t getPenaltyFunctionSecondDerivative(scalar_t h) const {
  //   if (h > delta_) {
  //     return mu_ / (h * h);
  //   } else {
  //     return mu_ / (delta_ * delta_) ;
  //   };
  // };
  scalar_t getPenaltyFunctionValue(scalar_t h) const {
    return h > d_threshold_ ? 0: weight_*(h-d_threshold_)*(h-d_threshold_);
  };

  scalar_t getPenaltyFunctionDerivative(scalar_t h) const {
    return h > d_threshold_ ? 0: weight_*2*(h-d_threshold_);

  };

  scalar_t getPenaltyFunctionSecondDerivative(scalar_t h) const {
    return h > d_threshold_ ? 0: weight_*2;
  };
};
}  // namespace graceful_mpc