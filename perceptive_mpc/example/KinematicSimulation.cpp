/*
 * Copyright (c) 2020 Johannes Pankert <pankertj@ethz.ch>, Giuseppe Rizzi
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
#include <geometry_msgs/TransformStamped.h>

#include <example/KinematicSimulation.h>

#include <perceptive_mpc/kinematics/ur5/UR5Kinematics.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <perceptive_mpc/costs/VoxbloxCost.h>

#include <perceptive_mpc/costs/InterpolatePoseTrajectory.h>

#include <kindr_ros/kindr_ros.hpp>


using namespace perceptive_mpc;

KinematicSimulation::KinematicSimulation(const ros::NodeHandle& nh)
    : nh_(nh), mpcUpdateFailed_(false), planAvailable_(false), kinematicInterfaceConfig_(),isFirstObservationReceived_(false) {
      
      
    }

bool KinematicSimulation::run() {


  parseParameters();
  loadTransforms();

  kinematicInterfaceConfig_.baseMass = 70;
  kinematicInterfaceConfig_.baseCOM = Eigen::Vector3d::Zero();
  

  PerceptiveMpcInterfaceConfig config;
  config.taskFileName = mpcTaskFile_;
  config.kinematicsInterface = std::make_shared<UR5Kinematics<ad_scalar_t>>(kinematicInterfaceConfig_);
  config.voxbloxConfig = configureCollisionAvoidance(config.kinematicsInterface);
  ocs2Interface_.reset(new PerceptiveMpcInterface(config));
  mpcInterface_ = std::make_shared<MpcInterface>(ocs2Interface_->getMpc());
  mpcInterface_->reset();

  // TODO: uncomment for admittance control on hardware:
  // admittanceReferenceModule.initialize();
   
  frontEndOMPLRRTStarConfig_.planning_time = 0.05;
  frontEndOMPLRRTStarConfig_.margin_x = 0.2;
  frontEndOMPLRRTStarConfig_.margin_y = 0.2;
  frontEndOMPLRRTStarConfig_.margin_z = 0.2;
  frontEndOMPLRRTStarConfig_.obstacle_margin = 0;
  frontEndOMPLRRTStarConfig_.interpolator_ = esdfCachingServer_->getInterpolator();
  frontEndOMPLRRTStar_.reset(new FrontEndOMPLRRTStar(frontEndOMPLRRTStarConfig_));
  // Init ros stuff
  armStatePublisher_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  ROS_INFO("Waiting for joint states subscriber ...");
  while (ros::ok() && armStatePublisher_.getNumSubscribers() == 0) {
    ros::Rate(100).sleep();
  }
  ROS_INFO("Joint state subscriber is connected.");
  wholebodyStateSubscriber_ = nh_.subscribe("/wholebodystate", 1, &KinematicSimulation::wholebodyStateCb, this);
  while(ros::ok() && wholebodyStateSubscriber_.getNumPublishers() == 0 && !isPureSimulation_){
     ros::Rate(100).sleep();
  }
  while(ros::ok() && !isPureSimulation_){
    ros::spinOnce();
    ros::Rate(10).sleep();
    if(isFirstObservationReceived_) break;
  }
  if(!isPureSimulation_)
  {
    ocs2Interface_->setInitialState(observation_.state());  
    observation_.state() = observationBuffer_.state();
    observation_.time() = ros::Time::now().toSec();
    optimalState_ = observation_.state();
    initialTime_ = observation_.time();
    ROS_INFO("wholebody state publisher is connected.");
    initializeCostDesiredTrajectory();
    setCurrentObservation(observation_);
  }
  else
  {
    observation_.state() = ocs2Interface_->getInitialState();
    observation_.time() = ros::Time::now().toSec();
    optimalState_ = observation_.state();
    initialTime_ = observation_.time();
    std::cerr << "initial state" << observation_.state().transpose() << std::endl;
    setCurrentObservation(observation_);
  }

  initializeCostDesiredTrajectory();
  ROS_INFO_STREAM("Starting from initial state: " << observation_.state().transpose());
  ROS_INFO_STREAM("Initial time (delta): " << observation_.time() - initialTime_);
  
  std::cerr << wholebodyStateSubscriber_.getNumPublishers() << std::endl;
  desiredEndEffectorPoseSubscriber_ =
      nh_.subscribe("/perceptive_mpc/desired_end_effector_pose", 1, &KinematicSimulation::desiredEndEffectorPoseCb, this);
  desiredEndEffectorWrenchPoseTrajectorySubscriber_ = nh_.subscribe("/perceptive_mpc/desired_end_effector_wrench_pose_trajectory", 1,
                                                                    &KinematicSimulation::desiredWrenchPoseTrajectoryCb, this);
  endEffectorPosePublisher_ = nh_.advertise<geometry_msgs::PoseStamped>("measured_end_effector_pose", 100);

  pointsOnRobotPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/perceptive_mpc/collision_points", 1, false);
  frontEndVisualizePublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/perceptive_mpc/front_end_trajectory", 1, false);

  comPublisher_ = nh_.advertise<geometry_msgs::PointStamped>("/perceptive_mpc/com", 1, false);
  zmpPublisher_ = nh_.advertise<geometry_msgs::PointStamped>("/perceptive_mpc/zmp", 1, false);
  
  cameraTransformPublisher_ = nh_.advertise<geometry_msgs::TransformStamped>("/perceptive_mpc/odomToCamera", 1, false);
  // Tracker worker
  std::thread trackerWorker(&KinematicSimulation::trackerLoop, this, ros::Rate(controlLoopFrequency_));

  // Mpc update worker
  mpcUpdateFrequency_ = (mpcUpdateFrequency_ == -1) ? 100 : mpcUpdateFrequency_;
  std::thread mpcUpdateWorker(&KinematicSimulation::mpcUpdate, this, ros::Rate(mpcUpdateFrequency_));

  // TF update worker
  std::thread tfUpdateWorker(&KinematicSimulation::tfUpdate, this, ros::Rate(tfUpdateFrequency_));

  ros::spin();
  trackerWorker.join();
  mpcUpdateWorker.join();
  tfUpdateWorker.join();
  return true;
}

void KinematicSimulation::loadTransforms() {
  UR5Kinematics<double> kinematics(kinematicInterfaceConfig_);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  {
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped = tfBuffer.lookupTransform("base_link",kinematics.armMountLinkName(), ros::Time(0), ros::Duration(1.0));
      // transformStamped = tfBuffer.lookupTransform("base_footprint","base_link" , ros::Time(0), ros::Duration(1.0));
  
    } catch (tf2::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    kinematicInterfaceConfig_.transformBase_X_ArmMount = Eigen::Matrix4d::Identity();
    kinematicInterfaceConfig_.transformBase_X_ArmMount.block<3, 3>(0, 0) = quat.toRotationMatrix();

    kinematicInterfaceConfig_.transformBase_X_ArmMount(0, 3) = transformStamped.transform.translation.x;
    kinematicInterfaceConfig_.transformBase_X_ArmMount(1, 3) = transformStamped.transform.translation.y;
    kinematicInterfaceConfig_.transformBase_X_ArmMount(2, 3) = transformStamped.transform.translation.z;
    ROS_INFO_STREAM("baseToArmMount_: " << std::endl << kinematicInterfaceConfig_.transformBase_X_ArmMount);
  }

  {
    geometry_msgs::TransformStamped transformStamped;
    try {
      // transformStamped = tfBuffer.lookupTransform(kinematics.toolMountLinkName(), "ENDEFFECTOR", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      throw;
    }
    Eigen::Quaterniond quat(1, 0,
                            0,0);
    // Eigen::AngleAxisd ax(0.0,Eigen::Vector3d::UnitZ());
    // Eigen::Quaterniond quat(ax);

    kinematicInterfaceConfig_.transformToolMount_X_Endeffector = Eigen::Matrix4d::Identity();
    
    kinematicInterfaceConfig_.transformToolMount_X_Endeffector.block<3, 3>(0, 0)<< 0,0,1,-1,0,0,0,-1,0;

    // kinematicInterfaceConfig_.transformToolMount_X_Endeffector(0, 3) = transformStamped.transform.translation.x;
    // kinematicInterfaceConfig_.transformToolMount_X_Endeffector(1, 3) = transformStamped.transform.translation.y;
    // kinematicInterfaceConfig_.transformToolMount_X_Endeffector(2, 3) = transformStamped.transform.translation.z;
    kinematicInterfaceConfig_.transformToolMount_X_Endeffector(0, 3) = 0.0;
    kinematicInterfaceConfig_.transformToolMount_X_Endeffector(1, 3) = 0.0;
    kinematicInterfaceConfig_.transformToolMount_X_Endeffector(2, 3) = 0.0;
    kinematicInterfaceConfig_.transformToolMount_X_Endeffector(3, 3) = 1.0; //dont forget this!!!
    ROS_INFO_STREAM("wrist2ToEETransform_: " << std::endl << kinematicInterfaceConfig_.transformToolMount_X_Endeffector);
  }
}

void KinematicSimulation::parseParameters() {
  ros::NodeHandle pNh("~");

  mpcTaskFile_ = pNh.param<std::string>("mpc_task_file", "task.info");

  mpcUpdateFrequency_ = pNh.param<double>("mpc_update_frequency", 100);
  tfUpdateFrequency_ = pNh.param<double>("tf_update_frequency", 10);
  maxLinearVelocity_ = pNh.param<double>("max_linear_velocity", 1.0);
  maxAngularVelocity_ = pNh.param<double>("max_angular_velocity", 1.0);
  controlLoopFrequency_ = pNh.param<double>("control_loop_frequency", 200);

  infoRate_ = pNh.param<double>("info_rate", 3.0);

  isPureSimulation_ = pNh.param<bool>("isPureSimulation", true);
  std::cerr << "isPureSimulation_" << isPureSimulation_ <<std::endl;
  verbose_ = pNh.param<bool>("verbose", false);
  auto defaultForceStd = pNh.param<std::vector<double>>("default_external_force", std::vector<double>());
  if (defaultForceStd.size() == 3) {
    defaultForce_ = Eigen::Vector3d::Map(defaultForceStd.data(), 3);
  }
  auto defaultTorqueStd = pNh.param<std::vector<double>>("default_external_torque", std::vector<double>());
  if (defaultTorqueStd.size() == 3) {
    defaultTorque_ = Eigen::Vector3d::Map(defaultTorqueStd.data(), 3);
  }
}

bool KinematicSimulation::trackerLoop(ros::Rate rate) {
  auto LastPeroidTime = std::chrono::steady_clock::now(); 
  auto CurrentPeroidTime = std::chrono::steady_clock::now(); 
  while (ros::ok()) {
    CurrentPeroidTime = std::chrono::steady_clock::now();
    // ROS_INFO_STREAM_THROTTLE(1,"trackerLoop Peroid: " << std::chrono::duration_cast<std::chrono::milliseconds>(CurrentPeroidTime - LastPeroidTime).count() << "ms");
    LastPeroidTime = CurrentPeroidTime;
    try {
      if (mpcUpdateFailed_) {
        ROS_ERROR_STREAM("Mpc update failed, stop.");
        return false;
      }

      if (!planAvailable_) {
        rate.sleep();
        continue;
      }
      // std::cerr << "tracker loop2" << std::endl;
      // use the optimal state as the next observation initialized with first observation
      // TODO: for integration on hardware, write the current observation from the state estimator instead
      Observation observation;

      if(!isPureSimulation_)
      {
        boost::unique_lock<boost::shared_mutex> lockGuard(observationMutex_);
        observation_.state() = observationBuffer_.state();
        // observation_.state() = optimalState_;
        observation_.time() = ros::Time::now().toSec();
        observation = observation_;
        // std::cerr << "observation_" << observation_.state().transpose() << std::endl;
      }
      else
      {
        boost::unique_lock<boost::shared_mutex> lockGuard(observationMutex_);
        // observation_.state() = observationBuffer_.state();
        observation_.state() = optimalState_;
        observation_.time() = ros::Time::now().toSec();
        observation = observation_;
        // std::cerr << "observation_" << observation_.state().transpose() << std::endl;        
      }

      MpcInterface::input_vector_t controlInput;
      MpcInterface::state_vector_t optimalState;
      
      dynamic_vector_t desiredPose =  costDesiredTrajectories_.desiredStateTrajectory()[1];
      auto currentEndEffectorPose = getEndEffectorPose();
      auto currentEndEffectorPoseInArmFr = getEndEffectorPoseInArmFr();
      Eigen::Vector3d currentPosition = currentEndEffectorPose.getPosition().toImplementation();
      Eigen::Vector3d currentPositionInArmFr = currentEndEffectorPoseInArmFr.getPosition().toImplementation();
      Eigen::AngleAxisd angleAxie(currentEndEffectorPoseInArmFr.getRotation().toImplementation());


      size_t subsystem;
      try {
        mpcInterface_->updatePolicy();
        mpcInterface_->evaluatePolicy(observation.time(), observation.state(), optimalState, controlInput, subsystem);
        // TODO: for integration on hardware, send the computed control inputs to the motor controllers
      } catch (const std::runtime_error& ex) {
        ROS_ERROR_STREAM("runtime_error occured11!");
        ROS_ERROR_STREAM("Caught exception while calling [mpcInterface_->evaluatePolicy]. Message11: " << ex.what());
        // return false;
      } catch (const std::exception& ex) {
        ROS_ERROR_STREAM("exception occured1!");
        ROS_ERROR_STREAM("Caught exception while calling [mpcInterface_->evaluatePolicy]. Message1: " << ex.what());
        return false;
      }
      ROS_INFO_STREAM_THROTTLE(infoRate_, std::endl
                                        << "    time:          " << observation.time() - initialTime_ << std::endl
                                        << "    current_state: " << observation.state().transpose() << std::endl
                                        << "    optimalState:  " << optimalState.transpose() << std::endl
                                        << "    controlInput:  " << controlInput.transpose() << std::endl
                                        << "    actual pos:   "  << currentPosition.transpose() << std::endl
                                        << "    desired pos:  " << desiredPose.head<Definitions::POSE_DIM>().tail<3>().transpose() << std::endl
                                        << "EE position in ARM:"<< currentPositionInArmFr.transpose() << std::endl
                                        // << "EE orientation in ARM:"<< (angleAxie.axis()*angleAxie.angle()).transpose() << std::endl
                                        << "    actual rot:   "  << currentEndEffectorPose.getRotation().toImplementation().coeffs().transpose()<< std::endl
                                        // << "    desired rot:   " << desiredPose.head<Definitions::POSE_DIM>().head<4>().transpose() << std::endl
                                        << "EEOrientation atan:"<< getEEAtan() << std::endl
                                        << "  manipulability:  "<< getManipulability()<< std::endl
                                        << std::endl);
      optimalState_ = optimalState;

    } catch (const std::runtime_error& ex) {
      ROS_ERROR_STREAM("runtime_error occured!");
      ROS_ERROR_STREAM("Caught exception while calling [KinematicSimulation::trackerLoop]. Message: " << ex.what());
      return false;
    } catch (const std::exception& ex) {
      ROS_ERROR_STREAM("exception occured!");
      ROS_ERROR_STREAM("Caught exception while calling [KinematicSimulation::trackerLoop]. Message: " << ex.what());
      return false;
    }
    rate.sleep();
  }
  return true;
}

bool KinematicSimulation::mpcUpdate(ros::Rate rate) {

  auto LastPeroidTime = std::chrono::steady_clock::now(); 
  auto CurrentPeroidTime = std::chrono::steady_clock::now(); 
  auto MidPeroidTime = std::chrono::steady_clock::now(); 

  while (ros::ok()) {
    CurrentPeroidTime = std::chrono::steady_clock::now();
    ROS_INFO_STREAM_THROTTLE(0.5,"mpcLoop Peroid:" << std::chrono::duration_cast<std::chrono::milliseconds>(CurrentPeroidTime - LastPeroidTime).count() << "ms");
    LastPeroidTime = CurrentPeroidTime;    
    if (mpcUpdateFailed_) {
      rate.sleep();
      continue;
    }
  
      MpcInterface::input_vector_t controlInput;
      MpcInterface::state_vector_t optimalState;
      
      dynamic_vector_t desiredPose =  costDesiredTrajectories_.desiredStateTrajectory()[1];
      auto currentEndEffectorPose = getEndEffectorPose();
      auto currentEndEffectorPoseInArmFr = getEndEffectorPoseInArmFr();
      Eigen::Vector3d currentPosition = currentEndEffectorPose.getPosition().toImplementation();
      Eigen::Vector3d currentPositionInArmFr = currentEndEffectorPoseInArmFr.getPosition().toImplementation();
      Eigen::AngleAxisd angleAxie(currentEndEffectorPoseInArmFr.getRotation().toImplementation());

    try {
      {
        // TODO: uncomment for admittance control on hardware:
        //        auto adaptedCostDesiredTrajectory = costDesiredTrajectories_;
        //        kindr::WrenchD measuredWrench; // input measured wrench from sensor here
        //        admittanceReferenceModule.adaptPath(rate.cycleTime().toSec(), adaptedCostDesiredTrajectory.desiredStateTrajectory(),
        //        measuredWrench); mpcInterface_->setTargetTrajectories(adaptedCostDesiredTrajectory);
        boost::shared_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
        mpcInterface_->setTargetTrajectories(costDesiredTrajectories_);
      }
      {
        boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_);
        setCurrentObservation(observation_);
      }
      if (esdfCachingServer_) {
        boost::unique_lock<boost::shared_mutex> intepolatorLock(intepolatorMutex_);

        esdfCachingServer_->updateInterpolator();
      }
    MidPeroidTime = std::chrono::steady_clock::now();
    ROS_INFO_STREAM_THROTTLE(0.5,"mpcLoop Peroid:(mid) " << std::chrono::duration_cast<std::chrono::milliseconds>(MidPeroidTime - LastPeroidTime).count() << "ms");

      {
        boost::unique_lock<boost::shared_mutex> intepolatorLock(intepolatorMutex_);
        mpcInterface_->advanceMpc();
      }
    } catch (const std::runtime_error& ex) {
      ROS_ERROR_STREAM("runtime_error occured!");
      ROS_ERROR_STREAM("Caught exception while calling [KinematicSimulation::mpcUpdate]. Message: " << ex.what());
      mpcUpdateFailed_ = true;
      return false;
    } catch (const std::exception& ex) {
      ROS_ERROR_STREAM("exception occured!");
      ROS_ERROR_STREAM("Caught exception while calling [KinematicSimulation::mpcUpdate]. Message: " << ex.what());
      mpcUpdateFailed_ = true;
      return false;
    }
    planAvailable_ = true;
    rate.sleep();
  }
  return true;
}

bool KinematicSimulation::tfUpdate(ros::Rate rate) {
  
  auto LastPeroidTime = std::chrono::steady_clock::now(); 
  auto CurrentPeroidTime = std::chrono::steady_clock::now(); 
  while (ros::ok()) {
    CurrentPeroidTime = std::chrono::steady_clock::now();
    // ROS_INFO_STREAM_THROTTLE(1,"tfLoop Peroid:" << std::chrono::duration_cast<std::chrono::milliseconds>(CurrentPeroidTime - LastPeroidTime).count() << "ms");
    LastPeroidTime = CurrentPeroidTime;


    if (mpcUpdateFailed_) {
      rate.sleep();
      continue;
    }

    try {
      Observation currentObservation;
      {
        boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_);
        currentObservation = observation_;
        // std::cerr << "observation_222" << observation_.state().transpose() << std::endl;
      }
      publishBaseTransform(currentObservation);
      publishArmState(currentObservation);
      publishEndEffectorPose();
      publishCameraTransform(currentObservation);
      if (pointsOnRobot_) {
        pointsOnRobotPublisher_.publish(pointsOnRobot_->getVisualization(currentObservation.state()));
      }

      ocs2::CostDesiredTrajectories costDesiredTrajectories;
      {
        boost::shared_lock<boost::shared_mutex> lock(costDesiredTrajectoryMutex_);
        costDesiredTrajectories = costDesiredTrajectories_;
      }

      // publishZmp(currentObservation, costDesiredTrajectories);
    } catch (const std::runtime_error& ex) {
      ROS_ERROR_STREAM("runtime_error occured!");
      ROS_ERROR_STREAM("Caught exception while calling [KinematicSimulation::tfUpdate]. Message: " << ex.what());
      return false;
    } catch (const std::exception& ex) {
      ROS_ERROR_STREAM("exception occured!");
      ROS_ERROR_STREAM("Caught exception while calling [KinematicSimulation::tfUpdate]. Message: " << ex.what());
      return false;
    }
    rate.sleep();
  }
  std::cerr << "TF LOOP FINISH" << std::endl;
  return true;
}

void KinematicSimulation::setCurrentObservation(const Observation& observation) {
  // the quaternion is not closed under addition
  // the state integration will make the quaternion non unique as the simulatoin goes on
  Eigen::Quaterniond currentBaseRotation = Eigen::Quaterniond(observation.state().head<4>());
  currentBaseRotation.normalize();
  mpcInterface_->setCurrentObservation(observation);
}

void KinematicSimulation::initializeCostDesiredTrajectory() {
  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  costDesiredTrajectories_.clear();
  reference_vector_t reference = reference_vector_t::Zero();
  auto currentEndEffectorPose = getEndEffectorPose();
  reference.head<Definitions::POSE_DIM>().head<4>() = currentEndEffectorPose.getRotation().getUnique().toImplementation().coeffs();
  reference.head<Definitions::POSE_DIM>().tail<3>() = currentEndEffectorPose.getPosition().toImplementation();
  reference.tail<Definitions::WRENCH_DIM>().head<3>() = defaultForce_;
  reference.tail<Definitions::WRENCH_DIM>().tail<3>() = defaultTorque_;

  std::cerr << "initial observation: "<<observation_.state().transpose() << std::endl;
  // std::cerr << "initial reference pos: "<<getEndEffectorPoseInArmFr().getPosition().toImplementation().transpose() << std::endl;
  Eigen::AngleAxisd ax(getEndEffectorPoseInArmFr().getRotation().toImplementation());

  std::cerr << "initial reference rot: "<< (ax.axis()*ax.angle()).transpose() << std::endl;
  
  Observation observation;
  {
    boost::shared_lock<boost::shared_mutex> observationLock(observationMutex_);
    observation = observation_;
  }
  costDesiredTrajectories_.desiredTimeTrajectory().push_back(observation.time());
  costDesiredTrajectories_.desiredTimeTrajectory().push_back(observation.time() + 1);
  costDesiredTrajectories_.desiredStateTrajectory().push_back(reference);
  costDesiredTrajectories_.desiredStateTrajectory().push_back(reference);
  costDesiredTrajectories_.desiredInputTrajectory().push_back(InputVector::Zero());
  costDesiredTrajectories_.desiredInputTrajectory().push_back(InputVector::Zero());
}

void KinematicSimulation::desiredEndEffectorPoseCb(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
  geometry_msgs::Pose currentEndEffectorPose;
  kindr_ros::convertToRosGeometryMsg(getEndEffectorPose(), currentEndEffectorPose);

  perceptive_mpc::WrenchPoseTrajectory wrenchPoseTrajectory;
  wrenchPoseTrajectory.header.stamp = ros::Time::now();
  wrenchPoseTrajectory.posesWrenches.resize(2);
  wrenchPoseTrajectory.posesWrenches[0].header.stamp = wrenchPoseTrajectory.header.stamp;
  wrenchPoseTrajectory.posesWrenches[0].pose = currentEndEffectorPose;
  tf2::toMsg(defaultForce_, wrenchPoseTrajectory.posesWrenches[0].wrench.force);
  tf2::toMsg(defaultTorque_, wrenchPoseTrajectory.posesWrenches[0].wrench.torque);

  wrenchPoseTrajectory.posesWrenches[1].header.stamp = wrenchPoseTrajectory.header.stamp;
  wrenchPoseTrajectory.posesWrenches[1].pose = msgPtr->pose;
  tf2::toMsg(defaultForce_, wrenchPoseTrajectory.posesWrenches[1].wrench.force);
  tf2::toMsg(defaultTorque_, wrenchPoseTrajectory.posesWrenches[1].wrench.torque);

  desiredWrenchPoseTrajectoryCb(wrenchPoseTrajectory);



  //add by yq(frontEnd)
  Eigen::Matrix<double,7,1> start;
  auto currentPose = getEndEffectorPose();
  Eigen::Vector3d currentPosition = currentPose.getPosition().toImplementation();
  Eigen::Quaterniond currentRotation = currentPose.getRotation().getUnique().toImplementation();


  start << currentRotation.coeffs(),currentPosition;
  Eigen::Matrix<double,7,1> end;
  end << msgPtr->pose.orientation.x,msgPtr->pose.orientation.y,msgPtr->pose.orientation.z,msgPtr->pose.orientation.w,
  msgPtr->pose.position.x,msgPtr->pose.position.y,msgPtr->pose.position.z;
  Eigen::Matrix<double,Eigen::Dynamic,7> desired_trajectory; 
  
  std::cerr << "(debugging start)" <<  start.transpose() << std::endl;
  std::cerr << "(debugging end)" <<  end.transpose() << std::endl;
  frontEndOMPLRRTStar_.reset(new FrontEndOMPLRRTStar(frontEndOMPLRRTStarConfig_)); //debugging
  bool is_success = frontEndOMPLRRTStar_->Plan(start,end,desired_trajectory);
  std::cerr << "debugging traj" << desired_trajectory << std::endl;


  visualization_msgs::MarkerArray frontEndTraj;
  for(int i = 0; i < desired_trajectory.rows(); i++){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.seq = i;
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = desired_trajectory(i,4);    
    marker.pose.position.y = desired_trajectory(i,5); 
    marker.pose.position.z = desired_trajectory(i,6); 
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.g = 1.0;

    frontEndTraj.markers.push_back(marker);
  }
  std::cerr << "publish marker array" << std::endl;
  frontEndVisualizePublisher_.publish(frontEndTraj);
}

void KinematicSimulation::desiredWrenchPoseTrajectoryCb(const perceptive_mpc::WrenchPoseTrajectory& wrenchPoseTrajectory) {
  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  costDesiredTrajectories_.clear();
  int N = wrenchPoseTrajectory.posesWrenches.size();
  costDesiredTrajectories_.desiredStateTrajectory().resize(N);
  costDesiredTrajectories_.desiredTimeTrajectory().resize(N);
  costDesiredTrajectories_.desiredInputTrajectory().resize(N);
  kindr::HomTransformQuatD lastPose;
  for (int i = 0; i < N; i++) {
    kindr::HomTransformQuatD desiredPose;
    reference_vector_t reference;
    kindr_ros::convertFromRosGeometryMsg(wrenchPoseTrajectory.posesWrenches[i].pose, desiredPose);
    reference.head<Definitions::POSE_DIM>().head<4>() = desiredPose.getRotation().toImplementation().coeffs();
    reference.head<Definitions::POSE_DIM>().tail<3>() = desiredPose.getPosition().toImplementation();
    Eigen::Vector3d force;
    tf2::fromMsg(wrenchPoseTrajectory.posesWrenches[i].wrench.force, force);
    reference.tail<Definitions::WRENCH_DIM>().head<3>() = force;
    Eigen::Vector3d torque;
    tf2::fromMsg(wrenchPoseTrajectory.posesWrenches[i].wrench.torque, torque);
    reference.tail<Definitions::WRENCH_DIM>().tail<3>() = torque;
    costDesiredTrajectories_.desiredStateTrajectory()[i] = reference;

    costDesiredTrajectories_.desiredInputTrajectory()[i] = MpcInterface::input_vector_t::Zero();

    if (i == 0) {
      costDesiredTrajectories_.desiredTimeTrajectory()[i] = ros::Time::now().toSec();
    } else {
      auto minTimeLinear = (desiredPose.getPosition() - lastPose.getPosition()).norm() / maxLinearVelocity_;
      auto minTimeAngular = std::abs(desiredPose.getRotation().getDisparityAngle(lastPose.getRotation())) / maxAngularVelocity_;

      auto lastOriginalTimeStamp = ros::Time(wrenchPoseTrajectory.posesWrenches[i - 1].header.stamp).toSec();
      auto currentOriginalTimeStamp = ros::Time(wrenchPoseTrajectory.posesWrenches[i].header.stamp).toSec();
      double originalTimingDiff = currentOriginalTimeStamp - lastOriginalTimeStamp;
      double segmentDuration = std::max(originalTimingDiff, std::max(minTimeLinear, minTimeAngular));

      costDesiredTrajectories_.desiredTimeTrajectory()[i] = costDesiredTrajectories_.desiredTimeTrajectory()[i - 1] + segmentDuration;
    }

    lastPose = desiredPose;
  }
}

void KinematicSimulation::publishBaseTransform(const Observation& observation) {
  geometry_msgs::TransformStamped base_transform;
  base_transform.header.frame_id = "odom";
  base_transform.child_frame_id = "base_link";

  const Eigen::Quaterniond currentRotation = Eigen::Quaterniond(observation.state().head<Definitions::BASE_STATE_DIM_>().head<4>());
  const Eigen::Matrix<double, 3, 1> currentPosition = observation.state().head<Definitions::BASE_STATE_DIM_>().tail<3>();

  base_transform.transform.translation.x = currentPosition(0);
  base_transform.transform.translation.y = currentPosition(1);
  base_transform.transform.translation.z = currentPosition(2);

  base_transform.transform.rotation.x = currentRotation.coeffs()(0);
  base_transform.transform.rotation.y = currentRotation.coeffs()(1);
  base_transform.transform.rotation.z = currentRotation.coeffs()(2);
  base_transform.transform.rotation.w = currentRotation.coeffs()(3);

  base_transform.header.stamp = ros::Time::now();
  tfBroadcaster_.sendTransform(base_transform);


}

void KinematicSimulation::publishCameraTransform(const Observation& observation)
{
  geometry_msgs::TransformStamped base_transform;
  base_transform.header.frame_id = "odom";
  base_transform.child_frame_id = "camera_depth_optical_frame";

  auto currentEndEffectorPose = getEndEffectorPose();
  Eigen::Vector3d currentPosition = currentEndEffectorPose.getPosition().toImplementation();
  Eigen::Quaterniond currentRotation = currentEndEffectorPose.getRotation().getUnique().toImplementation();


  Eigen::Matrix<double,3,3> gripper_R_rgb;
  gripper_R_rgb << -0.71379096,-0.6997661,0.02880736,0.70026391, -0.71376734,  0.01290856,0.01152878,  0.02938676,  0.99950163;
  Eigen::Matrix<double,3,1> gripper_t_rgb;
  gripper_t_rgb << 0.11962443, 0.11887692,-0.06505127;

  Eigen::Isometry3d gripper_T_rgb = Eigen::Isometry3d::Identity();
  gripper_T_rgb.rotate(gripper_R_rgb);
  gripper_T_rgb.pretranslate(gripper_t_rgb);
  

  Eigen::Isometry3d world_T_gripper = Eigen::Isometry3d::Identity();
  world_T_gripper.rotate(currentRotation);
  world_T_gripper.pretranslate(currentPosition);  //use pretranslate but not translate

  Eigen::Isometry3d world_T_rgb =  world_T_gripper * gripper_T_rgb;
  
  Eigen::Vector3d cameraPosition = world_T_rgb.translation();
  Eigen::Quaterniond cameraRotation(world_T_rgb.rotation());
  
  // std::cerr << "gripper_T_rgb" << gripper_T_rgb.matrix() << std::endl;
  // std::cerr << "world_T_gripper" << world_T_gripper.matrix() << std::endl;
  // std::cerr << "camera position:" << cameraPosition.transpose() << std::endl;
  // std::cerr << "cameraRotation:" << cameraRotation.coeffs().transpose() << std::endl;


  // for test only by yq
  base_transform.transform.translation.x = cameraPosition(0);
  base_transform.transform.translation.y = cameraPosition(1);
  base_transform.transform.translation.z = cameraPosition(2);

  base_transform.transform.rotation.x = cameraRotation.coeffs()(0);
  base_transform.transform.rotation.y = cameraRotation.coeffs()(1);
  base_transform.transform.rotation.z = cameraRotation.coeffs()(2);
  base_transform.transform.rotation.w = cameraRotation.coeffs()(3);
  base_transform.header.stamp = ros::Time::now();

  // ROS_INFO_STREAM_THROTTLE(infoRate_,"Pubisher cameraYransform position:" << cameraPosition.transpose());


  cameraTransformPublisher_.publish(base_transform);
  tfBroadcaster_.sendTransform(base_transform);

}
void KinematicSimulation::publishArmState(const Observation& observation) {
  sensor_msgs::JointState armState;
  armState.header.stamp = ros::Time::now();
  armState.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  armState.position.resize(Definitions::ARM_STATE_DIM_);
  Eigen::VectorXd armConfiguration = observation.state().tail<6>();
  for (size_t joint_idx = 0; joint_idx < Definitions::ARM_STATE_DIM_; joint_idx++) {
    armState.position[joint_idx] = armConfiguration(joint_idx);
  }
  armStatePublisher_.publish(armState);
}

void KinematicSimulation::publishEndEffectorPose() {
  geometry_msgs::PoseStamped endEffectorPoseMsg;
  static int endEffectorPoseCounter = 0;
  auto currentEndEffectorPose = getEndEffectorPose();
  Eigen::Vector3d currentPosition = currentEndEffectorPose.getPosition().toImplementation();
  Eigen::Quaterniond currentRotation = currentEndEffectorPose.getRotation().getUnique().toImplementation();

  // fill msg
  endEffectorPoseMsg.header.stamp = ros::Time::now();
  endEffectorPoseMsg.header.frame_id = "odom";
  endEffectorPoseMsg.header.seq = endEffectorPoseCounter++;
  endEffectorPoseMsg.pose.position.x = currentPosition(0);
  endEffectorPoseMsg.pose.position.y = currentPosition(1);
  endEffectorPoseMsg.pose.position.z = currentPosition(2);
  endEffectorPoseMsg.pose.orientation.x = currentRotation.coeffs()(0);
  endEffectorPoseMsg.pose.orientation.y = currentRotation.coeffs()(1);
  endEffectorPoseMsg.pose.orientation.z = currentRotation.coeffs()(2);
  endEffectorPoseMsg.pose.orientation.w = currentRotation.coeffs()(3);
  endEffectorPosePublisher_.publish(endEffectorPoseMsg);
}

void KinematicSimulation::publishZmp(const Observation& observation, const ocs2::CostDesiredTrajectories& costDesiredTrajectories) {
  UR5Kinematics<double> kinematicsInterface(kinematicInterfaceConfig_);
  Eigen::Vector3d com = kinematicsInterface.getCOMBaseFrame(observation.state());
  geometry_msgs::PointStamped comMsg;
  comMsg.header.frame_id = "base_link";
  comMsg.header.stamp = ros::Time::now();
  comMsg.point = tf2::toMsg(com);
  comPublisher_.publish(comMsg);

  auto interpolatedPose = interpolatePoseTrajectory(costDesiredTrajectories.desiredTimeTrajectory(),
                                                    costDesiredTrajectories.desiredStateTrajectory(), observation.time());
  auto interpolatedWrench = interpolateWrenchTrajectory(costDesiredTrajectories.desiredTimeTrajectory(),
                                                        costDesiredTrajectories.desiredStateTrajectory(), observation.time());
  Eigen::Vector3d zmp = kinematicsInterface.getZMPBaseFrame(observation.state(), interpolatedPose, interpolatedWrench);
  geometry_msgs::PointStamped zmpMsg;
  zmpMsg.header = comMsg.header;
  zmpMsg.point = tf2::toMsg(zmp);
  zmpPublisher_.publish(zmpMsg);
}

kindr::HomTransformQuatD KinematicSimulation::getEndEffectorPose() {
  {
    boost::shared_lock<boost::shared_mutex> lock(observationMutex_);
    Eigen::Matrix<double, 4, 4> endEffectorToWorldTransform;
    Eigen::VectorXd currentState = observation_.state();

    UR5Kinematics<double> kinematics(kinematicInterfaceConfig_);
    kinematics.computeState2EndeffectorTransform(endEffectorToWorldTransform, currentState);
    Eigen::Quaterniond eigenBaseRotation(endEffectorToWorldTransform.topLeftCorner<3, 3>());
    return kindr::HomTransformQuatD(kindr::Position3D(endEffectorToWorldTransform.topRightCorner<3, 1>()),
                                    kindr::RotationQuaternionD(eigenBaseRotation));
  }
}

double KinematicSimulation::getEEAtan(){
  {
    boost::shared_lock<boost::shared_mutex> lock(observationMutex_);
    Eigen::VectorXd currentState = observation_.state().tail<6>();
    UR5Kinematics<double> kinematics(kinematicInterfaceConfig_);
    double EEOrientation =  kinematics.getEEOrientationAtan(currentState);
    
    return EEOrientation;
  }  
}
double KinematicSimulation::getManipulability(){
  {
    boost::shared_lock<boost::shared_mutex> lock(observationMutex_);
    Eigen::VectorXd currentState = observation_.state().tail<6>();
    UR5Kinematics<double> kinematics(kinematicInterfaceConfig_);
    Eigen::Matrix<double,6,6> jacobian =  kinematics.getArmJacobian(currentState);
    
    return sqrt((jacobian.transpose()*jacobian).determinant());
  }  

}

// added by yq
kindr::HomTransformQuatD KinematicSimulation::getEndEffectorPoseInArmFr() {
  {
    boost::shared_lock<boost::shared_mutex> lock(observationMutex_);
    Eigen::Matrix<double, 4, 4> endEffectorToBaseTransform;
    Eigen::VectorXd currentState = observation_.state();
    
    UR5Kinematics<double> kinematics(kinematicInterfaceConfig_);
    endEffectorToBaseTransform = kinematics.getArmTransform(currentState.tail<6>());

    Eigen::Quaterniond eigenBaseRotation(endEffectorToBaseTransform.topLeftCorner<3, 3>());
    return kindr::HomTransformQuatD(kindr::Position3D(endEffectorToBaseTransform.topRightCorner<3, 1>()),
                                    kindr::RotationQuaternionD(eigenBaseRotation));
  }
}
std::shared_ptr<VoxbloxCostConfig> KinematicSimulation::configureCollisionAvoidance(
    std::shared_ptr<KinematicsInterfaceAD> kinematicInterface) {
  ros::NodeHandle pNh("~");
  std::shared_ptr<VoxbloxCostConfig> voxbloxCostConfig = nullptr;

  if (pNh.hasParam("collision_points")) {
    perceptive_mpc::PointsOnRobot::points_radii_t pointsAndRadii(8);
    using pair_t = std::pair<double, double>;

    XmlRpc::XmlRpcValue collisionPoints;
    pNh.getParam("collision_points", collisionPoints);
    if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("collision_points parameter is not of type array.");
      return voxbloxCostConfig;
    }
    for (int i = 0; i < collisionPoints.size(); i++) {
      if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN_STREAM("collision_points[" << i << "] parameter is not of type array.");
        return voxbloxCostConfig;
      }
      for (int j = 0; j < collisionPoints[i].size(); j++) {
        if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray) {
          ROS_WARN_STREAM("collision_points[" << i << "][" << j << "] parameter is not of type array.");
          return voxbloxCostConfig;
        }
        if (collisionPoints[i][j].size() != 2) {
          ROS_WARN_STREAM("collision_points[" << i << "][" << j << "] does not have 2 elements.");
          return voxbloxCostConfig;
        }
        double segmentId = collisionPoints[i][j][0];
        double radius = collisionPoints[i][j][1];
        pointsAndRadii[i].push_back(pair_t(segmentId, radius));
        ROS_INFO_STREAM("segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius);
      }
    }
    perceptive_mpc::PointsOnRobotConfig config;
    config.pointsAndRadii = pointsAndRadii;
    using ad_type = CppAD::AD<CppAD::cg::CG<double>>;
    config.kinematics = kinematicInterface;
    pointsOnRobot_.reset(new perceptive_mpc::PointsOnRobot(config));

    if (pointsOnRobot_->numOfPoints() > 0) {
      voxbloxCostConfig.reset(new VoxbloxCostConfig());
      voxbloxCostConfig->pointsOnRobot = pointsOnRobot_;

      esdfCachingServer_.reset(new voxblox::EsdfCachingServer(ros::NodeHandle(), ros::NodeHandle("~")));
      voxbloxCostConfig->interpolator = esdfCachingServer_->getInterpolator();

      pointsOnRobot_->initialize("points_on_robot");
    } else {
      // if there are no points defined for collision checking, set this pointer to null to disable the visualization
      pointsOnRobot_ = nullptr;
    }
  }
  return voxbloxCostConfig;
}

void KinematicSimulation::wholebodyStateCb(const std_msgs::Float64MultiArray& msg)
{

  std::vector<double> msg_in = msg.data;
  Eigen::VectorXd pf = Eigen::Map<Eigen::Matrix<double,9,1>>(msg_in.data(), msg_in.size());

  kindr::EulerAnglesRpyD rpy(0,0,pf.coeff(2));
  
  kindr::RotationQuaternionD quad(rpy);
  observationBuffer_.state().head<3>() = quad.vector().tail<3>();
  observationBuffer_.state()(3) = quad.vector()(0);
  observationBuffer_.state().head<Definitions::BASE_STATE_DIM_>().tail<3>().head<2>() = pf.head<2>();
  observationBuffer_.state().head<Definitions::BASE_STATE_DIM_>().tail<3>()(2)= 0;
  observationBuffer_.state().tail<6>() = pf.tail<6>();

  // std::cerr << "observationBuff" << observationBuffer_.state().transpose() << std::endl;
  observationBuffer_.time() = ros::Time::now().toSec();
  isFirstObservationReceived_ = true;
}
