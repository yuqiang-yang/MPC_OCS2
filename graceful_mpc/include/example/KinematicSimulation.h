/*
 * Copyright (c) 2023 Yuqiang Yang <auyuqiangyang@gmail.com>
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

// graceful_mpc
#include <example/GracefulMpcInterface.h>
#include <graceful_mpc/ExplicitTemplateInstantiations.h>
#include <graceful_mpc/costs/PointsOnRobot.h>

#include <graceful_mpc/Definitions.h>

// ocs2
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

// ros
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <atomic>
#include <boost/thread.hpp>

#include <kindr/Core>

#include "ocs2_core/Dimensions.h"
// #include "FrontEndOMPLRRTStar.hpp"
#include "graceful_mpc/PoseVelocityTrajectory.h"
#include "timing.h"
#include "JointSpaceRRT.hpp"


namespace graceful_mpc {

class KinematicSimulation {
 public:
  typedef ocs2::SystemObservation<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_> Observation;
  typedef ocs2::MPC_MRT_Interface<graceful_mpc::STATE_DIM_, graceful_mpc::INPUT_DIM_> MpcInterface;
  typedef MpcInterface::input_vector_t InputVector;
  using reference_vector_t = Eigen::Matrix<double, Definitions::REFERENCE_DIM, 1>;
  using dynamic_vector_t = ocs2::dynamic_vector_t;
  using dynamic_vector_array_t = ocs2::dynamic_vector_array_t;
  explicit KinematicSimulation(const ros::NodeHandle& nh = ros::NodeHandle());

  bool run();

 protected:
  std::string mpcTaskFile_;
  std::unique_ptr<graceful_mpc::GracefulMpcInterface> ocs2Interface_;
  std::shared_ptr<MpcInterface> mpcInterface_;
  std::shared_ptr<PointsOnRobot> pointsOnRobot_;
  std::shared_ptr<graceful_mpc::JointSpaceRRT> jointSpaceRRT_;
  // TODO: uncomment for admittance control on hardware:
  // AdmittanceReferenceModule admittanceReferenceModule;

  // params

  double mpcUpdateFrequency_;
  double tfUpdateFrequency_;
  double controlLoopFrequency_;
  double maxLinearVelocity_;
  double maxAngularVelocity_;
  Eigen::Vector3d defaultLinear_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d defaultAngular_ = Eigen::Vector3d::Zero();


  // flags
  std::atomic_bool planAvailable_;
  std::atomic_bool mpcUpdateFailed_;

  double initialTime_;
  MpcInterface::state_vector_t optimalState_;

  boost::shared_mutex observationMutex_;
  Observation observation_;
  MpcInterface::state_vector_t MidObservation_;
  MpcInterface::state_vector_t FinalObservation_;
  MpcInterface::input_vector_t input_;

  Observation observationBuffer_;
  KinematicInterfaceConfig kinematicInterfaceConfig_;

  boost::shared_mutex costDesiredTrajectoryMutex_;
  ocs2::CostDesiredTrajectories costDesiredTrajectories_;
  JointSpaceRRTConfig jointSpaceRRTConfig_;

  ros::Time latestObservationTime_;

  // ros
  ros::NodeHandle nh_;
  ros::Publisher armStatePublisher_;
  ros::Publisher midStatePublisher_;
  ros::Publisher finalStatePublisher_;

  ros::Publisher endEffectorPosePublisher_;
  ros::Publisher markerPosePublisher_;
  ros::Publisher pointsOnRobotPublisher_;
  ros::Publisher cameraTransformPublisher_;
  ros::Publisher frontEndVisualizePublisher_;
  ros::Publisher wholebodyControlPublisher_;
  ros::Publisher odomPublisher_;

  ros::Subscriber desiredEndEffectorPoseSubscriber_;
  ros::Subscriber wholebodyStateSubscriber_;
  ros::Subscriber desiredEndEffectorWrenchPoseTrajectorySubscriber_;
  tf::TransformBroadcaster tfBroadcaster_;

  int lastFrontEndWayPointNum_;
 protected:
  // thread 1 simulates the control loop: query new mpc plan, writes observation
  bool trackerLoop(ros::Rate rate);

  // thread 2 is the mpc solver
  bool mpcUpdate(ros::Rate rate);

  // thread 3 updates the tf for visualization
  bool tfUpdate(ros::Rate rate);

  bool waitForESDFReady_;
  int ESDFUpdateCnt_ ;

  bool urControlActivate_;
  bool realsenseActivate_;
  
  // compute the current end effector Pose on the base of the latest observation
  kindr::HomTransformQuatD getEndEffectorPose();

  // publish the transform from odom to the robot base
  void publishBaseTransform(const Observation& observation,std::string tf_prefix);

  // publish the joint state message of the arm state
  void publishArmState(const Observation& observation,std::string tf_prefix);

  void publishCameraTransform(const Observation& observation);
  // publish the current end effector pose to ros
  void publishEndEffectorPose();

  void publishMarkerPose(const Observation& observation);
    // parse all ros parameters
  void parseParameters();

  std::shared_ptr<FiestaCostConfig> configureCollisionAvoidance(std::shared_ptr<KinematicsInterfaceAD> kinematicInterface);

  // update the desired end effector pose on ros msg
  void desiredEndEffectorPoseCb(const geometry_msgs::PoseStampedConstPtr& msgPtr);

  void desiredPoseVelocityTrajectoryCb(const graceful_mpc::PoseVelocityTrajectory& poseVelocityTrajectory);


  void wholebodyStateCb(const std_msgs::Float64MultiArray& msg);
  // make sure the forwarded integrated state is normalized to unit quaternion observation for the base rotation
  void setCurrentObservation(const Observation& observation);

  void loadTransforms();
  void initializeCostDesiredTrajectory();
  void initRosTopic();
  kindr::HomTransformQuatD getEndEffectorPoseInArmFr();
  double getEEAtan();
  double getManipulability();

  private:
  double horizon_;
  bool verbose_;
  double infoRate_;
  bool isFirstObservationReceived_;
  bool isPureSimulation_;
  double filter_coeff_;
  MpcInterface::state_vector_t initialState_;
};
}  // namespace graceful_mpc