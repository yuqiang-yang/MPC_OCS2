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

#include <ocs2_comm_interfaces/SystemObservation.h>
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_MRT_Interface.h>
#include <ocs2_core/Dimensions.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/LinearConstraint.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_SLQ.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include "graceful_mpc/Definitions.h"
#include <ocs2_ddp/SLQ.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

extern template class ocs2::SLQ<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
extern template class ocs2::DDP_BASE<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
extern template class ocs2::Solver_BASE<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;

extern template class ocs2::MPC_SLQ<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
extern template class ocs2::MPC_BASE<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;

extern template class ocs2::RobotInterface<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
extern template class ocs2::Dimensions<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
extern template class ocs2::SystemOperatingPoint<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;

extern template class ocs2::MPC_MRT_Interface<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
extern template class ocs2::MRT_BASE<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;

extern template class ocs2::SystemObservation<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;

extern template class ocs2::LinearConstraint<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
extern template class ocs2::ConstraintBase<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;

extern template class ocs2::RolloutBase<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
extern template class ocs2::TimeTriggeredRollout<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_>;
