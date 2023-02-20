/*
 * Copyright (c) 2020 Johannes Pankert <pankertj@ethz.ch>
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
#include <graceful_mpc/costs/FiestaCost.h>
#include "graceful_mpc/StateConversion.h"
using namespace ocs2;
using namespace graceful_mpc;

void FiestaCost::setCurrentStateAndControl(const FiestaCost::scalar_t& t, const FiestaCost::state_vector_t& x,
                                            const FiestaCost::input_vector_t& u) {
    if (pointsOnRobot_) {
    gradientsFiesta_.setZero();
    Eigen::VectorXd positionsPointsOnRobot = pointsOnRobot_->getPoints(x);
    Eigen::MatrixXd jacobianPointsOnRobot = pointsOnRobot_->getJacobian(x);
    Eigen::VectorXd radii = pointsOnRobot_->getRadii();
    assert(positionsPointsOnRobot.size() % 3 == 0);
    int numPoints = pointsOnRobot_->numOfPoints();
    // std::cerr << "numPoints" << numPoints <<std::endl;
    for (int i = 0; i < numPoints; i++) {
      Eigen::Vector3d gradientFiesta ;
      gradientFiesta << 0.0, 0.0, 0.0;
      Eigen::Matrix<scalar_t, 3, 1> position = positionsPointsOnRobot.segment<3>(i * 3);
      double distance = esdfMap_->GetDistWithGradTrilinear(position,gradientFiesta);
      if(distance != -1){
        distances_[i] = distance - radii(i); 
        gradientsFiesta_.block<1, 3>(i, 3 * i) = gradientFiesta.transpose().cast<double>();
      }else {
        // std::cerr << "the query point is out of the map range! Please change the size."  << position.transpose() << std::endl;
        distances_[i] = maxDistance_ - radii(i);
        // gradientsFiesta_.block<1, 3>(i, 3 * i) = gradientFiesta.transpose().cast<double>();
      };
    }
    assert(gradients_.rows() == gradientsFiesta_.rows());
    assert(gradients_.cols() == jacobianPointsOnRobot.cols());
    gradients_ = gradientsFiesta_ * jacobianPointsOnRobot;
  }

  BASE::setCurrentStateAndControl(t, x, u);
}

void FiestaCost::getIntermediateCost(FiestaCost::scalar_t& L) {
  L = distances_.unaryExpr([this](const auto& x) { return getPenaltyFunctionValue(x); }).sum();
  // L = FiestaCost::scalar_t(0);
}
void FiestaCost::getIntermediateCostDerivativeState(FiestaCost::state_vector_t& dLdx) {
  dLdx = gradients_.transpose() * distances_.unaryExpr([this](const auto& x) { return getPenaltyFunctionDerivative(x); });
}

void FiestaCost::getIntermediateCostDerivativeStateVerbose(FiestaCost::state_vector_t& dLdx){
  getIntermediateCostDerivativeState(dLdx);
  std::cout << "distances_: " << std::endl << distances_ << std::endl;
  std::cout << "distances penalty_derivative_: " << std::endl << distances_.unaryExpr([this](const auto& x) { return getPenaltyFunctionDerivative(x); }) << std::endl;
  std::cout << "gradients_: " << std::endl << gradients_ << std::endl;
  std::cout << "gradientsFiesta_: " << std::endl << gradientsFiesta_ << std::endl;
  Eigen::MatrixXd jacobianPointsOnRobot = pointsOnRobot_->getJacobian(x_);
  std::cout << "jacobianPointsOnRobot: " << std::endl << jacobianPointsOnRobot << std::endl;

}

void FiestaCost::getIntermediateCostSecondDerivativeState(FiestaCost::state_matrix_t& dLdxx) {
  dLdxx = gradients_.transpose() *
          distances_.unaryExpr([this](const auto& x) { return getPenaltyFunctionSecondDerivative(x); }).asDiagonal() * gradients_;
  // dLdxx = FiestaCost::state_matrix_t::Zero();
}
void FiestaCost::getIntermediateCostDerivativeInput(FiestaCost::input_vector_t& dLdu) {
  dLdu = input_vector_t::Zero();
}
void FiestaCost::getIntermediateCostSecondDerivativeInput(FiestaCost::input_matrix_t& dLduu) {
  dLduu = input_matrix_t::Zero();
}
void FiestaCost::getIntermediateCostDerivativeInputState(FiestaCost::input_state_matrix_t& dLdux) {
  dLdux = input_state_matrix_t::Zero();
}
void FiestaCost::getTerminalCost(FiestaCost::scalar_t& Phi) {
  Phi = 0;
}
void FiestaCost::getTerminalCostDerivativeState(FiestaCost::state_vector_t& dPhidx) {
  dPhidx = state_vector_t ::Zero();
}
void FiestaCost::getTerminalCostSecondDerivativeState(FiestaCost::state_matrix_t& dPhidxx) {
  dPhidxx = state_matrix_t ::Zero();
}
void FiestaCost::getIntermediateCostDerivativeTime(CostFunctionBase::scalar_t& dLdt) {
  dLdt = 0;
}
void FiestaCost::getTerminalCostDerivativeTime(CostFunctionBase::scalar_t& dPhidt) {
  dPhidt = 0;
}
