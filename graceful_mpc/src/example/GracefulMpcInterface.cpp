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

#include "example/GracefulMpcInterface.h"
#include <graceful_mpc/costs/QuadraticEndeffectorTrackingCost.h>
#include <graceful_mpc/costs/FiestaCost.h>
#include <graceful_mpc/costs/FrontOrientationCost.h>
#include <graceful_mpc/costs/ManipulabilityCost.h>
#include <graceful_mpc/costs/StateCost.h>
#include <graceful_mpc/costs/CurveCost.h>

#include <graceful_mpc/kinematics/ur5/UR5Kinematics.hpp>

#include <cmath>

namespace graceful_mpc {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GracefulMpcInterface::GracefulMpcInterface(const GracefulMpcInterfaceConfig& config)
    : fiestaConfig_(config.fiestaConfig), kinematicsInterface_(config.kinematicsInterface) {
  std::string packagePath = ros::package::getPath("graceful_mpc");

  std::string taskFileName = config.taskFileName;
  if (config.taskFileName == "") {
    taskFileName = "task.info";
  }
  taskFile_ = packagePath + "/config/" + taskFileName;
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = packagePath + "/auto_generated/" + taskFileName;
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  // load setting from loading file
  loadSettings(taskFile_);

  // MPC
  resetMpc();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GracefulMpcInterface::loadSettings(const std::string& taskFile) {
  /*
   * Default initial condition
   */
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  /*
   * SLQ-MPC settings
   */
  slqSettings_.loadSettings(taskFile);
  mpcSettings_.loadSettings(taskFile);

  /*
   * Model settings
   */
  modelSettings_.loadSettings(taskFile);

  /*
   * Dynamics
   */
  systemDynamicsPtr_.reset(new SystemDynamics());

  systemDynamicsPtr_->initialize("base_arm_kinematics", libraryFolder_, modelSettings_.recompileLibraries_);

  /*
   * Cost functions
   */
  using WeightedCost = cost_linear_combination_t::WeightedCost;
  std::vector<WeightedCost> weightedCostFunctions;

  {
    // task space tracking cost
    QuadraticEndeffectorTrackingCostConfig config;
    ocs2::loadData::loadEigenMatrix(taskFile, "ee_tracking_task.Q", config.eePoseQ);
    ocs2::loadData::loadEigenMatrix(taskFile, "ee_tracking_task.R", config.R);
    ocs2::loadData::loadEigenMatrix(taskFile, "ee_tracking_task.Q_final", config.eePoseQFinal);
    config.kinematics = kinematicsInterface_;

    // std::cerr << "Q:       \n" << config.eePoseQ << std::endl;
    // std::cerr << "R:       \n" << config.R << std::endl;
    // std::cerr << "Q_final: \n" << config.eePoseQFinal << std::endl;

    std::shared_ptr<QuadraticEndeffectorTrackingCost> eeCost(new QuadraticEndeffectorTrackingCost(config));
    eeCost->initialize("ee_quadratic_cost", libraryFolder_, modelSettings_.recompileLibraries_);

    weightedCostFunctions.push_back(std::make_pair(1, eeCost));
  }

  bool useFiestaCost = false;
  ocs2::loadData::loadCppDataType(taskFile, "fiesta_cost.activate", useFiestaCost);
  std::cerr << "fiesta_cost.activate:       \n" << useFiestaCost << std::endl;

  if (fiestaConfig_ && useFiestaCost) { //add by yq
    ocs2::loadData::loadCppDataType(taskFile, "fiesta_cost.mu", fiestaConfig_->mu);
    // std::cerr << "fiesta_cost.mu:       \n" << fiestaConfig_->mu << std::endl;
    ocs2::loadData::loadCppDataType(taskFile, "fiesta_cost.delta", fiestaConfig_->delta);
    // std::cerr << "fiesta_cost.delta:       \n" << fiestaConfig_->delta << std::endl;
    ocs2::loadData::loadCppDataType(taskFile, "fiesta_cost.max_distance", fiestaConfig_->maxDistance);
    // std::cerr << "fiesta_cost.max_distance:       \n" << fiestaConfig_->maxDistance << std::endl;
    ocs2::loadData::loadCppDataType(taskFile, "fiesta_cost.weight", fiestaConfig_->weight);
    std::cerr << "fiesta_cost.weight:       \n" << fiestaConfig_->weight << std::endl;

    ocs2::loadData::loadCppDataType(taskFile, "fiesta_cost.d_threshold", fiestaConfig_->d_threshold);
    std::cerr << "fiesta_cost.d_threshold:       \n" << fiestaConfig_->d_threshold << std::endl;

    std::shared_ptr<FiestaCost> fiestaCost(new FiestaCost(*fiestaConfig_));
    weightedCostFunctions.push_back(std::make_pair(1, fiestaCost));
  }

  bool useFrontOrientationCost = false;
  ocs2::loadData::loadCppDataType(taskFile, "orientationCost.activate", useFrontOrientationCost);
  // std::cerr << "orientationCost.activate:   " << useFrontOrientationCost << std::endl;
  if(useFrontOrientationCost){
    FrontOrientationCostConfig config;
    config.kinematics = kinematicsInterface_;
    ocs2::loadData::loadCppDataType(taskFile, "orientationCost.weight", config.weight);
    // std::cerr << "orientationCost.weight:   "<< config.weight<< std::endl;
    std::shared_ptr<FrontOrientationCost> frontOrientationCost(new FrontOrientationCost(config));
    frontOrientationCost->initialize("front_orientation_cost",libraryFolder_, modelSettings_.recompileLibraries_);
    weightedCostFunctions.push_back(std::make_pair(1,frontOrientationCost));
  }

  bool useManipulailityCost = false;
  ocs2::loadData::loadCppDataType(taskFile, "manipulabilityCost.activate", useManipulailityCost);
  // std::cerr << "ManipulailityCost.activate:   " << useManipulailityCost << std::endl;
  if(useManipulailityCost){
    ManipulabilityCostConfig config;
    config.kinematics = kinematicsInterface_;
    ocs2::loadData::loadCppDataType(taskFile, "manipulabilityCost.weight", config.weight);
    // std::cerr << "manipulability.weight:   "<< config.weight << std::endl;
    std::shared_ptr<ManipulabilityCost> manipulabilityCost(new ManipulabilityCost(config));
    manipulabilityCost->initialize("manipulability_cost",libraryFolder_, false);  //disable recompile this library
    weightedCostFunctions.push_back(std::make_pair(1,manipulabilityCost));
  } 

  bool useStateCost = false;
  ocs2::loadData::loadCppDataType(taskFile, "stateCost.activate", useStateCost);
  if(useStateCost){
    StateCostConfig config;
    // std::cerr << "manipulability.weight:   "<< config.weight << std::endl;
    ocs2::loadData::loadEigenMatrix(taskFile, "stateCost.Q", config.Q);
    ocs2::loadData::loadEigenMatrix(taskFile, "stateCost.QFinal", config.QFinal);
    // std::cerr << config.Q << std::endl;
    // std::cerr << config.QFinal << std::endl;

    std::shared_ptr<StateCost> stateCost(new StateCost(config));
    weightedCostFunctions.push_back(std::make_pair(1,stateCost));
  } 

  bool useCurveCost = false;
  ocs2::loadData::loadCppDataType(taskFile, "CurveCost.activate", useCurveCost);
  if(useCurveCost){
    CurveCostConfig config;
    ocs2::loadData::loadCppDataType(taskFile, "CurveCost.weight", config.weight);

    std::shared_ptr<CurveCost> curveCost(new CurveCost(config));
    weightedCostFunctions.push_back(std::make_pair(1,curveCost));
  } 
  costPtr_.reset(new cost_linear_combination_t(weightedCostFunctions));

  Eigen::VectorXd lowerLimits((int)Definitions::STATE_DIM_);
  ocs2::loadData::loadEigenMatrix(taskFile, "limits.lower", lowerLimits);
  Eigen::VectorXd upperLimits((int)Definitions::STATE_DIM_);
  ocs2::loadData::loadEigenMatrix(taskFile, "limits.upper", upperLimits);
  Eigen::VectorXd accLimits((int)Definitions::INPUT_DIM_);
  ocs2::loadData::loadEigenMatrix(taskFile, "limits.acc", accLimits);
  std::cerr << "lowerLimits:       \n" << lowerLimits.transpose() << std::endl;
  std::cerr << "upperLimits:       \n" << upperLimits.transpose() << std::endl;
  std::cerr << "accLimits:    \n" << accLimits.transpose() << std::endl;

  bool useJointSpaceConstraints = false;
  ocs2::loadData::loadCppDataType(taskFile, "limits.enforce_limits", useJointSpaceConstraints);
  std::cerr << "useJointSpaceConstraints:       \n" << useJointSpaceConstraints << std::endl;
  if (useJointSpaceConstraints) {

    setupConstraints(lowerLimits, upperLimits, accLimits);
  } else {
    constraintPtr_.reset(new constraint_t());
  }

  operatingPointPtr_.reset(new OperatingPoint());

  /*
   * Rollout
   */
  ocs2::Rollout_Settings rolloutSettings;
  rolloutSettings.loadSettings(taskFile, "slq.rollout");
  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*systemDynamicsPtr_, rolloutSettings));

  /*
   * Time partitioning which defines the time horizon and the number of data partitioning
   */
  dim_t::scalar_t timeHorizon;
  ocs2::loadData::loadPartitioningTimes(taskFile, timeHorizon, numPartitions_, partitioningTimes_, true);
}
void GracefulMpcInterface::setupConstraints(const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits,
                                              const Eigen::VectorXd& accLimits) {
  std::unique_ptr<linear_constraint_t> linearConstraintPtr(new linear_constraint_t());

  linearConstraintPtr->numInequalityConstraint_ =  Definitions::STATE_DIM_*2 +  Definitions::INPUT_DIM_*2 ; 
  Eigen::VectorXd h0Eigen(linearConstraintPtr->numInequalityConstraint_);
  h0Eigen << -1 * (lowerLimits),
      1 * (upperLimits), accLimits, accLimits;
  linearConstraintPtr->h0_.resize(linearConstraintPtr->numInequalityConstraint_);
  for (int i = 0; i < linearConstraintPtr->numInequalityConstraint_; i++) {
    linearConstraintPtr->h0_[i] = h0Eigen[i];
  }

  linearConstraintPtr->dhdx_.resize(linearConstraintPtr->numInequalityConstraint_, linear_constraint_t::state_vector_t::Zero());
  const int armStateOffset = STATE_DIM_ - ARM_STATE_DIM_;

  for (int i = 0; i < STATE_DIM_; i++) {
    linearConstraintPtr->dhdx_[i][i] = 1;
    linearConstraintPtr->dhdx_[i + STATE_DIM_][i] = -1;
  }

  linearConstraintPtr->dhdu_.resize(linearConstraintPtr->numInequalityConstraint_, linear_constraint_t::input_vector_t::Zero());
  const int accConstraintOffset = 2 * STATE_DIM_;
  for (int i = 0; i < INPUT_DIM_; i++) {
    linearConstraintPtr->dhdu_[accConstraintOffset + i][i] = 1;
    linearConstraintPtr->dhdu_[accConstraintOffset + INPUT_DIM_ + i][i] = -1;
  }

  linearConstraintPtr->ddhdxdx_.resize(linearConstraintPtr->numInequalityConstraint_, linear_constraint_t::state_matrix_t::Zero());
  linearConstraintPtr->ddhdudu_.resize(linearConstraintPtr->numInequalityConstraint_, linear_constraint_t::input_matrix_t::Zero());
  linearConstraintPtr->ddhdudx_.resize(linearConstraintPtr->numInequalityConstraint_, linear_constraint_t::input_state_matrix_t::Zero());

  for (int i = 0; i < linearConstraintPtr->numInequalityConstraint_; i++) {
    std::cerr << "Constraint " << i << ": " << linearConstraintPtr->h0_[i] << " + [" << linearConstraintPtr->dhdx_[i].transpose()
              << "] * x + [" << linearConstraintPtr->dhdu_[i].transpose() << "] * u >= 0" << std::endl;
  }

  constraintPtr_ = move(linearConstraintPtr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GracefulMpcInterface::resetMpc() {
  mpcPtr_.reset(new mpc_t(timeTriggeredRolloutPtr_.get(), systemDynamicsPtr_.get(), constraintPtr_.get(), costPtr_.get(),
                          operatingPointPtr_.get(), partitioningTimes_, slqSettings_, mpcSettings_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GracefulMpcInterface::mpc_t& GracefulMpcInterface::getMpc() {
  return *mpcPtr_;
}

}  // namespace graceful_mpc