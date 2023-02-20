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
#include <graceful_mpc/SystemDynamics.h>

using namespace graceful_mpc;
// state X_b Y_b Theta_b Theta_arm ... velocity 
void SystemDynamics::systemFlowMap(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                   ad_dynamic_vector_t& stateDerivative) const {
  ad_scalar_t linearAcc = input.head<Definitions::BASE_INPUT_DIM_>()[0];
  ad_scalar_t angularAcc = input.head<Definitions::BASE_INPUT_DIM_>()[1];
  ad_scalar_t theta = state.head<Definitions::BASE_STATE_DIM_>()[2];

  
  stateDerivative.head<Definitions ::BASE_STATE_DIM_>() = state.tail<Definitions::VELOCITY_STATE_DIM_>().head<Definitions ::BASE_STATE_DIM_>();
  stateDerivative.head<Definitions ::POSITION_STATE_DIM_>().tail<Definitions::ARM_STATE_DIM_>() = state.tail<Definitions::VELOCITY_STATE_DIM_>().tail<Definitions ::ARM_STATE_DIM_>();
  stateDerivative.tail<Definitions ::VELOCITY_STATE_DIM_>()[0] = linearAcc*CppAD::cos(theta);
  stateDerivative.tail<Definitions ::VELOCITY_STATE_DIM_>()[1] = linearAcc*CppAD::sin(theta);
  stateDerivative.tail<Definitions ::VELOCITY_STATE_DIM_>()[2] = angularAcc;
  stateDerivative.tail<Definitions ::VELOCITY_STATE_DIM_>().tail<Definitions::ARM_STATE_DIM_>() = input.tail<Definitions::ARM_INPUT_DIM_>();

}

SystemDynamics* SystemDynamics::clone() const { return new SystemDynamics(*this); }
