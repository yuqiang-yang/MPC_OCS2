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

#pragma once

#include <graceful_mpc/ExplicitTemplateInstantiations.h>

// C++
#include <stdlib.h>

#include <iostream>
#include <string>

// OCS2
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ddp/SLQ_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/Rollout_Settings.h>

#include "ocs2_core/cost/CostFunctionLinearCombination.h"

// MM
#include <graceful_mpc/OperatingPoint.h>

#include "graceful_mpc/Definitions.h"
#include "graceful_mpc/ModelSettings.h"
#include "graceful_mpc/SystemDynamics.h"
#include "graceful_mpc/kinematics/KinematicsInterface.hpp"
#include "graceful_mpc/costs/FiestaCost.h"
// ros
#include <ros/package.h>
#include <ros/ros.h>

namespace graceful_mpc {

// class FiestaCostConfig;

struct GracefulMpcInterfaceConfig {
  std::string taskFileName;
  std::shared_ptr<FiestaCostConfig> fiestaConfig = nullptr;
  std::shared_ptr<KinematicsInterfaceAD> kinematicsInterface;
};

class GracefulMpcInterface : public ocs2::RobotInterface<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using dim_t = ocs2::Dimensions<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
  using base_cost_t = ocs2::CostFunctionBase<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
  using constraint_t = ocs2::ConstraintBase<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
  using linear_constraint_t = ocs2::LinearConstraint<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
  using time_triggered_rollout_t = ocs2::TimeTriggeredRollout<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
  using cost_linear_combination_t = ocs2::CostFunctionLinearCombination<Definitions::STATE_DIM_, Definitions::INPUT_DIM_>;
  using mpc_t = ocs2::MPC_SLQ<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
  using Base = ocs2::RobotInterface<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
  /**
   * Constructor
   */
  GracefulMpcInterface(const GracefulMpcInterfaceConfig& config);

  /**
   * Destructor
   */
  ~GracefulMpcInterface() = default;

  /**
   * setup all optimizes.
   *
   */
  void setupOptimizer(const std::string& taskFile) { resetMpc(); };

  /**
   * reset the mpc module
   */
  void resetMpc();

  /**
   * Get the model settings.
   *
   * @return Model settings.
   */

  void setInitialState(dim_t::state_vector_t initialState){ initialState_ = initialState;};

  ModelSettings& modelSettings() { return modelSettings_; };

  const dim_t::state_vector_t& getInitialState() { return initialState_; }
  ocs2::SLQ_Settings& slqSettings() { return slqSettings_; };
  ocs2::MPC_Settings& mpcSettings() { return mpcSettings_; };

  mpc_t& getMpc();

  const dynamics_t& getDynamics() const override { return *systemDynamicsPtr_; };

  const dynamics_derivatives_t& getDynamicsDerivatives() const override { return *systemDynamicsPtr_; };

  const cost_t& getCost() const override { return *costPtr_; };

  const constraint_t* getConstraintPtr() const override { return constraintPtr_.get(); };

  const OperatingPoint& getOperatingPoints() const override { return *operatingPointPtr_; };

  const KinematicsInterfaceAD& getKinematics() const{ return *kinematicsInterface_;}; // added by yq

 protected:
  /**
   * Loads the settings from the path file.
   *
   * @param [in] taskFile: Task's file full path.
   */
  void loadSettings(const std::string& taskFile);

  void setupConstraints(const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits, const Eigen::VectorXd& velocityLimits,
                        double positionMpcMarginRad);

 protected:
  std::string taskFile_;
  std::string libraryFolder_;

  std::unique_ptr<mpc_t> mpcPtr_;

  ModelSettings modelSettings_;
  ocs2::SLQ_Settings slqSettings_;
  ocs2::MPC_Settings mpcSettings_;
  dim_t::state_vector_t initialState_;

  std::unique_ptr<SystemDynamics> systemDynamicsPtr_;
  std::unique_ptr<cost_linear_combination_t> costPtr_;
  OperatingPoint::Ptr operatingPointPtr_;
  constraint_t::Ptr constraintPtr_;
  time_triggered_rollout_t::Ptr timeTriggeredRolloutPtr_;

  size_t numPartitions_ = 0;
  dim_t::scalar_array_t partitioningTimes_;

  std::shared_ptr<FiestaCostConfig> fiestaConfig_;
  std::shared_ptr<KinematicsInterfaceAD> kinematicsInterface_;


};

}  // namespace graceful_mpc