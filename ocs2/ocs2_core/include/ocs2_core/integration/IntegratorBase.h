/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <limits>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/Observer.h>
#include <ocs2_core/integration/OdeBase.h>
#include <ocs2_core/integration/SystemEventHandler.h>
#include <chrono>

namespace ocs2 {

/**
 * The interface class for integration of autonomous systems.
 *
 * @tparam STATE_DIM: Dimension of the state space, can be Eigen::Dynamic.
 */
template <int STATE_DIM>
class IntegratorBase {
 public:
  using DIMENSIONS = Dimensions<STATE_DIM, 0>;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;

  using system_t = OdeBase<STATE_DIM>;
  using observer_t = Observer<STATE_DIM>;
  using system_func_t = std::function<void(const state_vector_t& x, state_vector_t& dxdt, scalar_t t)>;
  using observer_func_t = std::function<void(const state_vector_t& x, scalar_t t)>;

  /**
   * Default constructor
   * @param [in] eventHandler
   */
  explicit IntegratorBase(std::shared_ptr<SystemEventHandler<STATE_DIM>> eventHandlerPtr = nullptr)
      : eventHandlerPtr_(std::move(eventHandlerPtr)) {
    if (eventHandlerPtr_ == nullptr) {
      eventHandlerPtr_ = std::make_shared<SystemEventHandler<STATE_DIM>>();
    }
  }

  /**
   * Default destructor
   */
  virtual ~IntegratorBase() = default;

  /**
   * Equidistant integration based on initial and final time as well as step length.
   *
   * @param [in] system: System dynamics
   * @param [in] observer: Observer
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dt: Time step.
   */
  void integrate_const(system_t& system, observer_t& observer, const state_vector_t& initialState, scalar_t startTime, scalar_t finalTime,
                       scalar_t dt, int maxNumSteps = std::numeric_limits<int>::max()) {
    eventHandlerPtr_->setMaxNumSteps(maxNumSteps);
    observer_func_t callback = [&](const state_vector_t& x, scalar_t t) {
      observer.observe(system, x, t);
      eventHandlerPtr_->handleEvent(system, t, x);
    };
    run_integrate_const(system.systemFunction(), callback, initialState, startTime, finalTime, dt);
  }

  /**
   * Adaptive time integration based on start time and final time.
   *
   * @param [in] system: System dynamics
   * @param [in] observer: Observer
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   */
  void integrate_adaptive(system_t& system, observer_t& observer, const state_vector_t& initialState, scalar_t startTime,
                          scalar_t finalTime, scalar_t dtInitial = 0.01, scalar_t AbsTol = 1e-6, scalar_t RelTol = 1e-3,
                          int maxNumSteps = std::numeric_limits<int>::max()) {
    auto start = std::chrono::high_resolution_clock::now();
    eventHandlerPtr_->setMaxNumSteps(maxNumSteps);
    std::cerr.setf(std::ios::fixed);
    std::cerr.precision(5);
    state_vector_t x = state_vector_t::Zero();
    state_vector_t dxdt = state_vector_t::Zero();
    scalar_t t(startTime);
    system.systemFunction()(x,dxdt,t);
    // std::cout << "before integrade:" << dxdt.transpose() << std::endl;

    state_vector_t initalStateBackup = initialState;
    scalar_t startTimeBackup = startTime;
    scalar_t finalTimeBackup = finalTime;
    scalar_t dtInitialBackup = dtInitial;
    observer_func_t callback = [&](const state_vector_t& x, scalar_t t) {
      observer.observe(system, x, t);
      eventHandlerPtr_->handleEvent(system, t, x);
    };
    run_integrate_adaptive(system.systemFunction(), callback, initialState, startTime, finalTime, dtInitial, AbsTol, RelTol);
    auto stop = std::chrono::high_resolution_clock::now();
    using us = std::chrono::microseconds;
    us elapsedUs = std::chrono::duration_cast<us>(stop - start);
    // std::cout << "integrate_adaptive: " << elapsedUs.count() << "us" << std::endl;
    state_vector_t x1 = state_vector_t::Zero();
    state_vector_t dxdt1 = state_vector_t::Zero();
    scalar_t t1(startTime);
    system.systemFunction()(x1,dxdt1,t1);

    if (dxdt1.isApprox(dxdt) && initalStateBackup == initialState && startTimeBackup==startTime && finalTimeBackup == finalTime && dtInitialBackup == dtInitial)
    {
      // std::cerr << "remain the same:"<< system.getNumFunctionCalls()<<std::endl;
    }
    else{
      // std::cerr << "change!!!!!!!!!!!!!!!!!!!!!"<< std::endl;
    }

  }

  /**
   * Output integration based on a given time trajectory.
   *
   * @param [in] system: System dynamics
   * @param [in] observer: Observer
   * @param [in] initialState: Initial state.
   * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
   * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   */
  void integrate_times(system_t& system, observer_t& observer, const state_vector_t& initialState,
                       typename scalar_array_t::const_iterator beginTimeItr, typename scalar_array_t::const_iterator endTimeItr,
                       scalar_t dtInitial = 0.01, scalar_t AbsTol = 1e-6, scalar_t RelTol = 1e-3,
                       int maxNumSteps = std::numeric_limits<int>::max()) {
    eventHandlerPtr_->setMaxNumSteps(maxNumSteps);
    // std::cerr << "debugging initial time" << startTime << " initial state " << initialState.transpose() << std::endl;

    observer_func_t callback = [&](const state_vector_t& x, scalar_t t) {
      observer.observe(system, x, t);
      eventHandlerPtr_->handleEvent(system, t, x);
    };
    run_integrate_times(system.systemFunction(), callback, initialState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol);
  }

 protected:
  virtual void run_integrate_const(system_func_t system, observer_func_t observer, const state_vector_t& initialState, scalar_t startTime,
                                   scalar_t finalTime, scalar_t dt) = 0;

  virtual void run_integrate_adaptive(system_func_t system, observer_func_t observer, const state_vector_t& initialState,
                                      scalar_t startTime, scalar_t finalTime, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) = 0;

  virtual void run_integrate_times(system_func_t system, observer_func_t observer, const state_vector_t& initialState,
                                   typename scalar_array_t::const_iterator beginTimeItr, typename scalar_array_t::const_iterator endTimeItr,
                                   scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) = 0;

 private:
  std::shared_ptr<SystemEventHandler<STATE_DIM>> eventHandlerPtr_;
};

}  // namespace ocs2
