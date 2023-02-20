#pragma once

#include "Definitions.h"
#include <Eigen/Core>
#include <Eigen/Geometry> 

namespace graceful_mpc {

class StateConversion {
 public:
  StateConversion(){}

   ~StateConversion() = default;

  static Eigen::Matrix<double,13,1>  converteTo13d(const Eigen::Matrix<double,18,1> x){
    Eigen::Matrix<double,13,1> transformState;
    Eigen::AngleAxisd ax(x.head<Definitions::BASE_STATE_DIM_>()[2],Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quat(ax); 
    transformState[0] = quat.x();
    transformState[1] = quat.y();
    transformState[2] = quat.z();
    transformState[3] = quat.w();
    transformState.head<6>().tail<2>() = x.head<2>();
    transformState[6] = 0;
    transformState.tail<Definitions::ARM_STATE_DIM_>() = x.head<Definitions::POSITION_STATE_DIM_>().tail<Definitions::ARM_STATE_DIM_>();
    return transformState;
  }
};

} //namespace graceful_mpc