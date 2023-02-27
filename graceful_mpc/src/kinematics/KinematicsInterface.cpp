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
#include <Eigen/Dense>
#include <graceful_mpc/kinematics/KinematicsInterface.hpp>
#include "graceful_mpc/Definitions.h"
using namespace graceful_mpc;

template <typename SCALAR_T>
KinematicsInterface<SCALAR_T>::KinematicsInterface(const KinematicInterfaceConfig& config) : config_(config) {}

template <typename SCALAR_T>
void KinematicsInterface<SCALAR_T>::computeState2EndeffectorTransform(Eigen::Matrix<SCALAR_T, 4, 4>& transform,
                                                                      const Eigen::Matrix<SCALAR_T, -1, 1>& state) const {
  if (state.size() != 17) {
    std::stringstream ss;
    ss << "Error: state.size()=" << state.size() << "!=17";
    std::string errorMessage = ss.str();
    std::cerr << std::endl << errorMessage << std::endl << std::endl;
    throw std::runtime_error(ss.str());
  }
  
  Eigen::Matrix<SCALAR_T,13,1> transformState;
  Eigen::AngleAxis<SCALAR_T> ax(state.template head<Definitions::BASE_STATE_DIM_>()[2],Eigen::Matrix<SCALAR_T,3,1>::UnitZ());
  Eigen::Quaternion<SCALAR_T> quat(ax); 
  transformState[0] = quat.x();
  transformState[1] = quat.y();
  transformState[2] = quat.z();
  transformState[3] = quat.w();
  transformState.template head<6>().template tail<2>() = state.template head<2>();
  transformState[6] = (SCALAR_T)0;
  transformState.template tail<Definitions::ARM_STATE_DIM_>() = state.template head<Definitions::POSITION_STATE_DIM_>().template tail<Definitions::ARM_STATE_DIM_>();

  Eigen::Quaternion<SCALAR_T> baseOrientation;
  baseOrientation.coeffs() = transformState.template head<7>().template head<4>();
  const Eigen::Matrix<SCALAR_T, 3, 1>& basePosition = transformState.template head<7>().template tail<3>();
  const Eigen::Matrix<SCALAR_T, 6, 1>& armState = transformState.template tail<6>();

  // homogeneous transform world -> base
  Eigen::Matrix<SCALAR_T, 4, 4> homWorld2Base = Eigen::Matrix<SCALAR_T, 4, 4>::Identity();
  homWorld2Base.template topLeftCorner<3, 3>() = baseOrientation.toRotationMatrix();
  homWorld2Base.template topRightCorner<3, 1>() = basePosition;
  // world -> endeffector
  transform = homWorld2Base * computeBase2EndeffectorTransform(armState);
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 4, 4> KinematicsInterface<SCALAR_T>::computeBase2EndeffectorTransform(
    const Eigen::Matrix<SCALAR_T, 6, 1>& armState) const {
  const Eigen::Matrix<SCALAR_T, 4, 4> armBaseToWrist2Transform = computeArmMountToToolMountTransform(armState);
  return config_.transformBase_X_ArmMount.template cast<SCALAR_T>() * armBaseToWrist2Transform *
         config_.transformToolMount_X_Endeffector.template cast<SCALAR_T>();
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, -1> KinematicsInterface<SCALAR_T>::computeState2MultiplePointsOnRobot(
    const Eigen::Matrix<SCALAR_T, -1, 1>& state, const std::vector<std::vector<double>>& points) const {
  if (state.size() != 17) {
    std::stringstream ss;
    ss << "Error: state.size()=" << state.size() << "!=17";
    std::string errorMessage = ss.str();
    std::cerr << std::endl << errorMessage << std::endl << std::endl;
    throw std::runtime_error(ss.str());
  }

  int dim = 0;
  for (int i = 0; i < points.size(); i++) {
    for (int j = 0; j < points[i].size(); j++) {
      dim++;
    }
  }
  if (dim == 0) {
    return Eigen::Matrix<SCALAR_T, 3, -1>(3, 0);
  }

  Eigen::Matrix<SCALAR_T,13,1> transformState;
  Eigen::AngleAxis<SCALAR_T> ax(state.template head<Definitions::BASE_STATE_DIM_>()[2],Eigen::Matrix<SCALAR_T,3,1>::UnitZ());
  Eigen::Quaternion<SCALAR_T> quat(ax); 
  transformState[0] = quat.x();
  transformState[1] = quat.y();
  transformState[2] = quat.z();
  transformState[3] = quat.w();
  transformState.template head<6>().template tail<2>() = state.template head<2>();
  transformState[6] = (SCALAR_T)0;
  transformState.template tail<Definitions::ARM_STATE_DIM_>() = state.template head<Definitions::POSITION_STATE_DIM_>().template tail<Definitions::ARM_STATE_DIM_>();

  Eigen::Quaternion<SCALAR_T> baseOrientation;
  baseOrientation.coeffs() = transformState.template head<7>().template head<4>();
  const Eigen::Matrix<SCALAR_T, 3, 1>& basePosition = transformState.template head<7>().template tail<3>();
  const Eigen::Matrix<SCALAR_T, 6, 1>& armState = transformState.template tail<6>();

  Eigen::Matrix<SCALAR_T, 4, 4> worldXFrBase = Eigen::Matrix<SCALAR_T, 4, 4>::Identity();
  worldXFrBase.template topLeftCorner<3, 3>() = baseOrientation.toRotationMatrix();
  worldXFrBase.template topRightCorner<3, 1>() = basePosition;

  return computeArmState2MultiplePointsOnRobot(armState, points, config_.transformBase_X_ArmMount, config_.transformToolMount_X_Endeffector,
                                               worldXFrBase);
}

template <typename SCALAR_T>
SCALAR_T KinematicsInterface<SCALAR_T>::getEEOrientationAtan(const Eigen::Matrix<SCALAR_T, -1, 1>& state)const{
  
  Eigen::Matrix<SCALAR_T,13,1> transformState;
  Eigen::AngleAxis<SCALAR_T> ax(state.template head<Definitions::BASE_STATE_DIM_>()[2],Eigen::Matrix<SCALAR_T,3,1>::UnitZ());
  Eigen::Quaternion<SCALAR_T> quat(ax); 
  transformState[0] = quat.x();
  transformState[1] = quat.y();
  transformState[2] = quat.z();
  transformState[3] = quat.w();
  transformState.template head<6>().template tail<2>() = state.template head<2>();
  transformState[6] = (SCALAR_T)0;
  transformState.template tail<Definitions::ARM_STATE_DIM_>() = state.template head<Definitions::POSITION_STATE_DIM_>().template tail<Definitions::ARM_STATE_DIM_>();

  Eigen::Quaternion<SCALAR_T> baseOrientation;
  baseOrientation.coeffs() = transformState.template head<7>().template head<4>();
  const Eigen::Matrix<SCALAR_T, 3, 1>& basePosition = transformState.template head<7>().template tail<3>();
  const Eigen::Matrix<SCALAR_T, 6, 1>& armState = transformState.template tail<6>();

  // homogeneous transform world -> base
  Eigen::Matrix<SCALAR_T, 4, 4> homWorld2Base = Eigen::Matrix<SCALAR_T, 4, 4>::Identity();
  homWorld2Base.template topLeftCorner<3, 3>() = baseOrientation.toRotationMatrix();
  homWorld2Base.template topRightCorner<3, 1>() = basePosition;

  auto armBaseTransform = homWorld2Base * config_.transformBase_X_ArmMount.template cast<SCALAR_T>();
  Eigen::Matrix<SCALAR_T, 4, 4> homWorld2End;
  computeState2EndeffectorTransform(homWorld2End,state);


  return CppAD::atan((homWorld2End(1,3) - armBaseTransform(1,3))/(homWorld2End(0,3) - armBaseTransform(0,3)));
}
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 6> KinematicsInterface<SCALAR_T>::getArmJacobian(const Eigen::Matrix<SCALAR_T, 6, 1>& armState)const{

  const Eigen::Matrix<SCALAR_T, 6, 6> baseToWrist3Jacobian = computeArmJacobian(armState);
  return baseToWrist3Jacobian;

}

template <typename SCALAR_T>
SCALAR_T KinematicsInterface<SCALAR_T>::getManipulability(const Eigen::Matrix<SCALAR_T, 6, 1>& armState)const{
  Eigen::Matrix<SCALAR_T, 6, 6> jacobian = getArmJacobian(armState);
  jacobian = jacobian.transpose()*jacobian;

  return jacobian.determinant();
}
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 4, 4> KinematicsInterface<SCALAR_T>::getArmTransform(const Eigen::Matrix<SCALAR_T, 6, 1>& armState)
{
  return computeArmMountToToolMountTransform(armState) * config_.transformToolMount_X_Endeffector.template cast<SCALAR_T>();

  // return computeArmMountToToolMountTransform(armState);
}

template class graceful_mpc::KinematicsInterface<double>;
template class graceful_mpc::KinematicsInterface<CppAD::AD<CppAD::cg::CG<double>>>;