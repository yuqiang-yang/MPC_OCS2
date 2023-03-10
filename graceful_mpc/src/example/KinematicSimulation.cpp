
#include <geometry_msgs/TransformStamped.h>

#include <example/KinematicSimulation.h>
#include <graceful_mpc/kinematics/ur5/UR5Kinematics.hpp>
#include <tf/transform_broadcaster.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <graceful_mpc/costs/InterpolatePoseTrajectory.h>

#include <kindr_ros/kindr_ros.hpp>
#include <Fiesta.h>
#include <iostream>
using namespace graceful_mpc;

KinematicSimulation::KinematicSimulation(const ros::NodeHandle& nh)
    : nh_(nh), mpcUpdateFailed_(false), planAvailable_(false), kinematicInterfaceConfig_(),isFirstObservationReceived_(false) {
        waitForESDFReady_ = false;
        ESDFUpdateCnt_ = 0;
    }

bool KinematicSimulation::run() {

  parseParameters();
  loadTransforms();
  initRosTopic();

  //Fiesta
  ros::NodeHandle node("~");
  fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr> esdf(node);

  GracefulMpcInterfaceConfig config;
  config.taskFileName = mpcTaskFile_;
  config.kinematicsInterface = std::make_shared<UR5Kinematics<ad_scalar_t>>(kinematicInterfaceConfig_);
  config.fiestaConfig = configureCollisionAvoidance(config.kinematicsInterface);
  if(config.fiestaConfig) //not nullptr
  {
    config.fiestaConfig->esdfMap.reset(esdf.esdf_map_); //set the esdf map
    frontEndOMPLRRTStarConfig_.esdf_map.reset(esdf.esdf_map_);  //set the FrontEnd
    frontEndOMPLRRTStar_.reset(new FrontEndOMPLRRTStar(frontEndOMPLRRTStarConfig_));
  }
  //set the ovservation to the tf 
  observation_.time() = ros::Time::now().toSec();
  observation_.state() = isPureSimulation_ ? initialState_ : observationBuffer_.state();
  MidObservation_ = observation_.state();
  FinalObservation_ = observation_.state();
  publishMarkerPose(observation_);
  std::thread tfUpdateWorker(&KinematicSimulation::tfUpdate, this, ros::Rate(tfUpdateFrequency_));
  // wait for a stable esdf map 
  while(esdf.esdfUpdateCnt_ < 5 /*&& !isPureSimulation_*/ && realsenseActivate_)
  {
    ROS_INFO_STREAM("wait until the esdf become stable! " << esdf.esdfUpdateCnt_);
    observation_.time() = ros::Time::now().toSec();
    ros::spinOnce();
    ros::Rate(10).sleep();
  }

  //construct Graceful MPC and set initial state
  ocs2Interface_.reset(new GracefulMpcInterface(config));
  observation_.time() = ros::Time::now().toSec();
  observation_.state() = isPureSimulation_ ? initialState_ : observationBuffer_.state();
  observationBuffer_.time() = ros::Time::now().toSec();
  initialTime_ = observation_.time();
  ocs2Interface_->setInitialState(observation_.state());  
  mpcInterface_ = std::make_shared<MpcInterface>(ocs2Interface_->getMpc());
  mpcInterface_->reset();
  setCurrentObservation(observation_);
  optimalState_ = observation_.state();
  publishMarkerPose(observation_);

  //set after the observation_ is initialized. 
  initializeCostDesiredTrajectory();


  std::thread mpcUpdateWorker(&KinematicSimulation::mpcUpdate, this, ros::Rate(mpcUpdateFrequency_));
  std::thread trackerWorker(&KinematicSimulation::trackerLoop, this, ros::Rate(controlLoopFrequency_));

  ros::spin();
  trackerWorker.join();
  mpcUpdateWorker.join();
  tfUpdateWorker.join();
  return true;
}
void KinematicSimulation::initRosTopic(){
  desiredEndEffectorPoseSubscriber_ =
      nh_.subscribe("/graceful_mpc/desired_end_effector_pose", 1, &KinematicSimulation::desiredEndEffectorPoseCb, this);
  endEffectorPosePublisher_ = nh_.advertise<geometry_msgs::PoseStamped>("measured_end_effector_pose", 100);
  markerPosePublisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/graceful_mpc/set_marker_pose", 1);

  pointsOnRobotPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/graceful_mpc/collision_points", 1, false);
  frontEndVisualizePublisher_ = nh_.advertise<visualization_msgs::Marker>("/graceful_mpc/front_end_trajectory", 1, false);
  
  cameraTransformPublisher_ = nh_.advertise<geometry_msgs::TransformStamped>("/graceful_mpc/odomToCamera", 1, false);
  armStatePublisher_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  midStatePublisher_ = nh_.advertise<sensor_msgs::JointState>("/mid/joint_states", 10);
  finalStatePublisher_ = nh_.advertise<sensor_msgs::JointState>("/final/joint_states", 10);

  ROS_INFO("Waiting for joint states subscriber ...");
  while (ros::ok() && armStatePublisher_.getNumSubscribers() == 0) {
    ros::Rate(100).sleep();
  }
  ROS_INFO("Joint state subscriber is connected.");
  ROS_INFO("Waiting for wholebodyState publisher.");
  wholebodyControlPublisher_ = nh_.advertise<std_msgs::Float64MultiArray>("/wholebodycontrol", 1, false);
  wholebodyStateSubscriber_ = nh_.subscribe("/wholebodystate", 1, &KinematicSimulation::wholebodyStateCb, this);
  while(ros::ok() && wholebodyStateSubscriber_.getNumPublishers() == 0 && !isPureSimulation_){
     ros::Rate(100).sleep();
  }
  ROS_INFO("WholebodyState publisher is connected.");
  while(ros::ok() && !isPureSimulation_){
    ros::spinOnce();
    ros::Rate(10).sleep();
    if(isFirstObservationReceived_) break;
  }
  if(isPureSimulation_){
    odomPublisher_ = nh_.advertise<nav_msgs::Odometry>("/graceful_mpc/odom", 1, false);
  }

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
    // ROS_INFO_STREAM("baseToArmMount_: " << std::endl << kinematicInterfaceConfig_.transformBase_X_ArmMount);
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
    // ROS_INFO_STREAM("wrist2ToEETransform_: " << std::endl << kinematicInterfaceConfig_.transformToolMount_X_Endeffector);
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
  filter_coeff_ = pNh.param<double>("filter_coeff", 0.1);
  infoRate_ = pNh.param<double>("info_rate", 3.0);

  isPureSimulation_ = pNh.param<bool>("isPureSimulation", true);
  urControlActivate_ = pNh.param<bool>("ur_control_activate", true);
  realsenseActivate_ = pNh.param<bool>("realsenseActivate", true);


  std::cerr << "isPureSimulation_" << isPureSimulation_ <<std::endl;
  verbose_ = pNh.param<bool>("verbose", false);
  auto defaultLinear = pNh.param<std::vector<double>>("default_linear_velocity", std::vector<double>());
  if (defaultLinear.size() == 3) {
    defaultLinear_ = Eigen::Vector3d::Map(defaultLinear.data(), 3);
  }
  auto defaultAngular = pNh.param<std::vector<double>>("default_angular_velocity", std::vector<double>());
  if (defaultAngular.size() == 3) {
    defaultAngular_ = Eigen::Vector3d::Map(defaultAngular.data(), 3);
  }

  std::string packagePath = ros::package::getPath("graceful_mpc");
  ocs2::loadData::loadCppDataType(packagePath + "/config/" +mpcTaskFile_, "frontEndOMPLRRTStar.planning_time", frontEndOMPLRRTStarConfig_.planning_time);
  ocs2::loadData::loadCppDataType(packagePath + "/config/" +mpcTaskFile_, "frontEndOMPLRRTStar.margin_x", frontEndOMPLRRTStarConfig_.margin_x);
  ocs2::loadData::loadCppDataType(packagePath + "/config/" +mpcTaskFile_, "frontEndOMPLRRTStar.margin_y", frontEndOMPLRRTStarConfig_.margin_y);
  ocs2::loadData::loadCppDataType(packagePath + "/config/" +mpcTaskFile_, "frontEndOMPLRRTStar.margin_z", frontEndOMPLRRTStarConfig_.margin_z);
  ocs2::loadData::loadCppDataType(packagePath + "/config/" +mpcTaskFile_, "frontEndOMPLRRTStar.obstacle_margin", frontEndOMPLRRTStarConfig_.obstacle_margin);
  ocs2::loadData::loadCppDataType(packagePath + "/config/" +mpcTaskFile_, "frontEndOMPLRRTStar.collisionCheckerResolution", frontEndOMPLRRTStarConfig_.collisionCheckerResolution);
  ocs2::loadData::loadCppDataType(packagePath + "/config/" +mpcTaskFile_, "frontEndOMPLRRTStar.distance_gain", frontEndOMPLRRTStarConfig_.distance_gain);

  ocs2::loadData::loadEigenMatrix(packagePath + "/config/" +mpcTaskFile_, "initialState", initialState_);
  ocs2::loadData::loadCppDataType(packagePath + "/config/" +mpcTaskFile_, "mpcTimeHorizon.timehorizon", horizon_);
  
  frontEndOMPLRRTStarConfig_.maxVel = maxLinearVelocity_;
  std::cerr << "horizon_" << horizon_ <<std::endl;
  
}


bool KinematicSimulation::trackerLoop(ros::Rate rate) {
  while (ros::ok()) {
    try {
      if (mpcUpdateFailed_) {
        ROS_ERROR_STREAM("Mpc update failed, stop.");
        return false;
      }
      if (!planAvailable_) {
        rate.sleep();
        continue;
      }
      // use the optimal state as the next observation initialized with first observation
      // TODO: for integration on hardware, write the current observation from the state estimator instead
      Observation observation;
      if(!isPureSimulation_)
      {
        boost::unique_lock<boost::shared_mutex> lockGuard(observationMutex_);
        observation_.state() = observationBuffer_.state();
        observation_.time() = observationBuffer_.time();
        // observation_.state() = optimalState_;
        // observation_.time() = ros::Time::now().toSec();
        observation = observation_;
        // std::cerr << "observation_" << observation_.state().transpose() << std::endl;
      }
      else
      {
        boost::unique_lock<boost::shared_mutex> lockGuard(observationMutex_);
        // observation_.state() = observationBuffer_.state();
        // if(input_.norm() > 5e-3){
        observation_.state() = optimalState_;
        // }
        observation_.time() = ros::Time::now().toSec();
        observation = observation_;
        // std::cerr << "observation_" << observation_.state().transpose() << std::endl;        
      }

      MpcInterface::input_vector_t controlInput;
      MpcInterface::input_vector_t finalInput;

      MpcInterface::state_vector_t optimalState;
      
      int N = costDesiredTrajectories_.desiredStateTrajectory().size();
      dynamic_vector_t desiredPose =  costDesiredTrajectories_.desiredStateTrajectory()[N-1];
      auto currentEndEffectorPose = getEndEffectorPose();
      auto currentEndEffectorPoseInArmFr = getEndEffectorPoseInArmFr();
      Eigen::Vector3d currentPosition = currentEndEffectorPose.getPosition().toImplementation();
      Eigen::Vector3d currentPositionInArmFr = currentEndEffectorPoseInArmFr.getPosition().toImplementation();
      Eigen::AngleAxisd angleAxie(currentEndEffectorPoseInArmFr.getRotation().toImplementation());


      size_t subsystem;
      double finalTime;

      try {
        mpcInterface_->updatePolicy();
        mpcInterface_->evaluatePolicy(observation.time() + horizon_ / 4.0, observation.state(), MidObservation_, controlInput, subsystem);
        mpcInterface_->evaluatePolicy(observation.time() + horizon_+1, observation.state(), FinalObservation_, controlInput, subsystem);
        // mpcInterface_->getPolicyFinalState(finalTime,FinalObservation_,finalInput,subsystem);
        mpcInterface_->evaluatePolicy(observation.time(), observation.state(), optimalState, controlInput, subsystem);
        input_ = controlInput;
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
                                        // << "    finalTime:     " << finalTime - initialTime_ << std::endl
                                        << "    current_state: " << observation.state().transpose() << std::endl
                                        // << "    mid_state:     " << MidObservation_.transpo`se() << std::endl
                                        // << "    final_state:   " << FinalObservation_.transpose() << std::endl
                                        << "    optimalState:  " << optimalState.transpose() << std::endl
                                        << "    controlInput:  " << controlInput.transpose() << std::endl
                                        // << "    finalInput:    " << finalInput.transpose() << std::endl
                                        << "    actual pos:   "  << currentPosition.transpose() << std::endl
                                        << "    desired pos:  " << desiredPose.head<Definitions::POSE_DIM>().tail<3>().transpose() << std::endl
                                        << "    atan          " << getEEAtan() << std::endl
                                        << "  manipulability:  "<< getManipulability()<< std::endl
                                        << timing::Timing::Print()  
                                        << std::endl);

      optimalState_ = optimalState;


      if(!isPureSimulation_)
      {
        std_msgs::Float64MultiArray msg;
        msg.data.resize(8);
        for(int i = 0; i < 8;i ++)
        {
          msg.data[i] = controlInput(i);
        }
        wholebodyControlPublisher_.publish(msg);
      }
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
  timing::Timer mpcTimer("mpcLoop");
  while (ros::ok()) {
    mpcTimer.Start();
    if (mpcUpdateFailed_) {
      rate.sleep();
      continue;
    }
  
    try {
      {
        boost::shared_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
        mpcInterface_->setTargetTrajectories(costDesiredTrajectories_);
      }
      {
        boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_);
        setCurrentObservation(observation_);
      }

      {
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
    mpcTimer.Stop();

    planAvailable_ = true;
    rate.sleep();
  }
  return true;
}

bool KinematicSimulation::tfUpdate(ros::Rate rate) {
  
  while (ros::ok()) {

    if (mpcUpdateFailed_) {
      rate.sleep();
      continue;
    }

    try {
      Observation currentObservation;
      Observation midObservation;
      Observation finalObservation;
      {
        boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_);
        currentObservation = observation_;
        midObservation.time() = ros::Time::now().toSec();
        midObservation.state() = MidObservation_;
        finalObservation.time() = ros::Time::now().toSec();
        finalObservation.state() = FinalObservation_;
        // std::cerr << "observation_222" << observation_.state().transpose() << std::endl;
      }
      
      publishBaseTransform(currentObservation,"");
      publishArmState(currentObservation,"");
      publishBaseTransform(midObservation,"mid");
      publishArmState(midObservation,"mid");
      publishBaseTransform(finalObservation,"final");
      publishArmState(finalObservation,"final");
      publishEndEffectorPose();
      // if(isPureSimulation_)
        publishCameraTransform(currentObservation);
      // else
        // publishCameraTransform(observationBuffer_);
      if (pointsOnRobot_) {
        pointsOnRobotPublisher_.publish(pointsOnRobot_->getVisualization(currentObservation.state()));
      }

      ocs2::CostDesiredTrajectories costDesiredTrajectories;
      {
        boost::shared_lock<boost::shared_mutex> lock(costDesiredTrajectoryMutex_);
        costDesiredTrajectories = costDesiredTrajectories_;
      }

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
  mpcInterface_->setCurrentObservation(observation);
}
//unchange
void KinematicSimulation::initializeCostDesiredTrajectory() {
  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  costDesiredTrajectories_.clear();
  reference_vector_t reference = reference_vector_t::Zero();
  std::cerr << "initial observation: "<<observation_.state().transpose() << std::endl;
  auto currentEndEffectorPose = getEndEffectorPose();
  reference.head<Definitions::POSE_DIM>().head<4>() = currentEndEffectorPose.getRotation().getUnique().toImplementation().coeffs();
  reference.head<Definitions::POSE_DIM>().tail<3>() = currentEndEffectorPose.getPosition().toImplementation();
  reference.tail<Definitions::VELOCITY_DIM_>().head<3>() = defaultLinear_;
  reference.tail<Definitions::VELOCITY_DIM_>().tail<3>() = defaultAngular_;

  std::cerr << "initial reference: "<<reference.transpose() << std::endl;
  std::cerr << "initial reference pos: "<<getEndEffectorPoseInArmFr().getPosition().toImplementation().transpose() << std::endl;
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

  //add by yq(frontEnd)
  Eigen::Matrix<double,7,1> start;
  auto currentPose = getEndEffectorPose();
  Eigen::Vector3d currentPosition = currentPose.getPosition().toImplementation();
  Eigen::Quaterniond currentRotation = currentPose.getRotation().getUnique().toImplementation();
  Observation currentObservation;
  {
    boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_);
    currentObservation = observation_;
  }

  start << currentRotation.coeffs(),currentPosition;
  Eigen::Matrix<double,7,1> end;
  end << msgPtr->pose.orientation.x,msgPtr->pose.orientation.y,msgPtr->pose.orientation.z,msgPtr->pose.orientation.w,
  msgPtr->pose.position.x,msgPtr->pose.position.y,msgPtr->pose.position.z;
  Eigen::Matrix<double,Eigen::Dynamic,7> desired_trajectory; 
  Eigen::Matrix<double,Eigen::Dynamic,3> velocity_trajectory; 

  // std::cerr << "(debugging start)" <<  start.transpose() << std::endl;
  // std::cerr << "(debugging end)" <<  end.transpose() << std::endl;
  frontEndOMPLRRTStar_.reset(new FrontEndOMPLRRTStar(frontEndOMPLRRTStarConfig_)); //debugging
  Eigen::VectorXd time_traj;

  try{
    bool is_success = frontEndOMPLRRTStar_->Plan(start,end,desired_trajectory,time_traj,velocity_trajectory);
    // std::cerr << "desired_trajectory" << std::endl<<desired_trajectory<< std::endl;
    if(!is_success)
    {
      ROS_ERROR("Found no frontEnd solution");
      return;
    }
  }
  catch(const std::exception ex){
    std::cerr << "frontend raise an error:" << ex.what() << std::endl;
  }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    // marker.header.seq = i;
    marker.id = 999;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::DELETE;
    frontEndVisualizePublisher_.publish(marker);

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.scale.x = 0.005;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
  for(int i = 0; i < desired_trajectory.rows(); i++){
    geometry_msgs::Point pt;

    pt.x = desired_trajectory(i,4);    
    pt.y = desired_trajectory(i,5); 
    pt.z = desired_trajectory(i,6); 
    marker.points.push_back(pt);
  }

  // std::cerr << "publish marker array" << std::endl;
  frontEndVisualizePublisher_.publish(marker);

  graceful_mpc::PoseVelocityTrajectory poseVelocityTrajectory;
  poseVelocityTrajectory.header.stamp = ros::Time::now();
  poseVelocityTrajectory.posesVelocity.resize(desired_trajectory.rows());

  for(int i = 0; i < desired_trajectory.rows(); i++){
  geometry_msgs::Pose tmpPose;
  tmpPose.orientation.x = desired_trajectory(i,0);
  tmpPose.orientation.y = desired_trajectory(i,1);
  tmpPose.orientation.z = desired_trajectory(i,2);
  tmpPose.orientation.w = desired_trajectory(i,3);
  tmpPose.position.x = desired_trajectory(i,4);
  tmpPose.position.y = desired_trajectory(i,5);
  tmpPose.position.z = desired_trajectory(i,6);
  
  poseVelocityTrajectory.posesVelocity[i].header.stamp = ros::Time(poseVelocityTrajectory.header.stamp.toSec() + time_traj(i)/2.0);

  poseVelocityTrajectory.posesVelocity[i].pose = tmpPose;
  if(i == 0){
      poseVelocityTrajectory.posesVelocity[i].velocity.force.x = currentObservation.state()[2];
  }
  poseVelocityTrajectory.posesVelocity[i].velocity.force.x =(1-filter_coeff_) * poseVelocityTrajectory.posesVelocity[i-1].velocity.force.x + filter_coeff_* std::atan2(velocity_trajectory(i,1),velocity_trajectory(i,0));
  tf2::toMsg(defaultAngular_, poseVelocityTrajectory.posesVelocity[i].velocity.torque);  //angular part
  
  }
  desiredPoseVelocityTrajectoryCb(poseVelocityTrajectory);
}

void KinematicSimulation::desiredPoseVelocityTrajectoryCb(const graceful_mpc::PoseVelocityTrajectory& poseVelocityTrajectory) {
  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  
  costDesiredTrajectories_.clear();
  int N = poseVelocityTrajectory.posesVelocity.size();
  costDesiredTrajectories_.desiredStateTrajectory().resize(N);
  costDesiredTrajectories_.desiredTimeTrajectory().resize(N);
  costDesiredTrajectories_.desiredInputTrajectory().resize(N);
  kindr::HomTransformQuatD lastPose;
  for (int i = 0; i < N; i++) {
    kindr::HomTransformQuatD desiredPose;
    reference_vector_t reference;
    kindr_ros::convertFromRosGeometryMsg(poseVelocityTrajectory.posesVelocity[i].pose, desiredPose);
    reference.head<Definitions::POSE_DIM>().head<4>() = desiredPose.getRotation().toImplementation().coeffs();
    reference.head<Definitions::POSE_DIM>().tail<3>() = desiredPose.getPosition().toImplementation();
    Eigen::Vector3d linearVelocity;
    tf2::fromMsg(poseVelocityTrajectory.posesVelocity[i].velocity.force, linearVelocity);
    reference.tail<Definitions::VELOCITY_DIM_>().head<3>() = linearVelocity;
    Eigen::Vector3d angularVelocity;
    tf2::fromMsg(poseVelocityTrajectory.posesVelocity[i].velocity.torque, angularVelocity);
    reference.tail<Definitions::VELOCITY_DIM_>().tail<3>() = angularVelocity;
    costDesiredTrajectories_.desiredStateTrajectory()[i] = reference;

    costDesiredTrajectories_.desiredInputTrajectory()[i] = MpcInterface::input_vector_t::Zero();
    costDesiredTrajectories_.desiredTimeTrajectory()[i] = poseVelocityTrajectory.posesVelocity[i].header.stamp.toSec();
    // if (i == 0) {
    //   costDesiredTrajectories_.desiredTimeTrajectory()[i] = ros::Time::now().toSec();
    // } else {
    //   auto minTimeLinear = (desiredPose.getPosition() - lastPose.getPosition()).norm() / maxLinearVelocity_;
    //   auto minTimeAngular = std::abs(desiredPose.getRotation().getDisparityAngle(lastPose.getRotation())) / maxAngularVelocity_;

    //   auto lastOriginalTimeStamp = ros::Time(poseVelocityTrajectory.posesVelocity[i - 1].header.stamp).toSec();
    //   auto currentOriginalTimeStamp = ros::Time(poseVelocityTrajectory.posesVelocity[i].header.stamp).toSec();
    //   double originalTimingDiff = currentOriginalTimeStamp - lastOriginalTimeStamp;
    //   double segmentDuration = std::max(originalTimingDiff, std::max(minTimeLinear, minTimeAngular));

    //   costDesiredTrajectories_.desiredTimeTrajectory()[i] = costDesiredTrajectories_.desiredTimeTrajectory()[i - 1] + segmentDuration;
    // }

    lastPose = desiredPose;
  }
  // costDesiredTrajectories_.display();
  ROS_INFO_STREAM("current time");
}

void KinematicSimulation::publishMarkerPose(const Observation& observation){
  auto currentEndEffectorPose = getEndEffectorPose();
  Eigen::Vector3d currentPosition = currentEndEffectorPose.getPosition().toImplementation();
  Eigen::Quaterniond currentRotation = currentEndEffectorPose.getRotation().getUnique().toImplementation();
  geometry_msgs::PoseStamped poseStamp;
  poseStamp.header.stamp = ros::Time::now();
  poseStamp.pose.position.x = currentPosition(0);
  poseStamp.pose.position.y = currentPosition(1);
  poseStamp.pose.position.z = currentPosition(2);  

  poseStamp.pose.orientation.x = currentRotation.x();
  poseStamp.pose.orientation.y = currentRotation.y();
  poseStamp.pose.orientation.z = currentRotation.z();
  poseStamp.pose.orientation.w = currentRotation.w();

  markerPosePublisher_.publish(poseStamp);
  }

void KinematicSimulation::publishBaseTransform(const Observation& observation,std::string tf_prefix) {
  geometry_msgs::TransformStamped base_transform;
  base_transform.header.frame_id = "odom";
  base_transform.child_frame_id = tf_prefix + "base_footprint";

  Eigen::AngleAxisd ax(observation.state()[2],Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond currentRotation(ax);
  Eigen::Matrix<double, 3, 1> currentPosition;
  currentPosition << observation.state()[0], observation.state()[1], 0.0;

  base_transform.transform.translation.x = currentPosition(0);
  base_transform.transform.translation.y = currentPosition(1);
  base_transform.transform.translation.z = currentPosition(2);

  base_transform.transform.rotation.x = currentRotation.coeffs()(0);
  base_transform.transform.rotation.y = currentRotation.coeffs()(1);
  base_transform.transform.rotation.z = currentRotation.coeffs()(2);
  base_transform.transform.rotation.w = currentRotation.coeffs()(3);

  base_transform.header.stamp = ros::Time::now();
  tfBroadcaster_.sendTransform(base_transform);

  //publish odom for simulation
  if(isPureSimulation_){
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = currentPosition(0);
    odom.pose.pose.position.y = currentPosition(1);
    odom.pose.pose.position.z = currentPosition(2);
    odomPublisher_.publish(odom);
  }

}

void KinematicSimulation::publishCameraTransform(const Observation& observation)
{
  geometry_msgs::TransformStamped base_transform;
  base_transform.header.frame_id = "odom";
  base_transform.child_frame_id = "camera_depth_optical_frame";
  base_transform.header.stamp = ros::Time::now();

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

  
  // tfBroadcaster_.sendTransform(base_transform);
  // use the StampedTransform type to facilate the presentation of tf tree, or the publish rate is 10000(undefined).
    tfBroadcaster_.sendTransform( 
        tf::StampedTransform(		  
          tf::Transform(tf::Quaternion(cameraRotation.coeffs()(0), cameraRotation.coeffs()(1), cameraRotation.coeffs()(2), cameraRotation.coeffs()(3)), tf::Vector3(cameraPosition(0),cameraPosition(1),cameraPosition(2))),
          ros::Time(observation.time()),"odom", "camera_depth_optical_frame")); 
          
  // for test only by yq
  // world_T_rgb = world_T_rgb.inverse();
  // cameraPosition = world_T_rgb.translation();
  // cameraRotation = Eigen::Quaterniond(world_T_rgb.rotation());

  // base_transform.transform.translation.x = cameraPosition(0);
  // base_transform.transform.translation.y = cameraPosition(1);
  // base_transform.transform.translation.z = cameraPosition(2);

  // base_transform.transform.rotation.x = cameraRotation.coeffs()(0);
  // base_transform.transform.rotation.y = cameraRotation.coeffs()(1);
  // base_transform.transform.rotation.z = cameraRotation.coeffs()(2);
  // base_transform.transform.rotation.w = cameraRotation.coeffs()(3);
  // base_transform.header.stamp = ros::Time::now();
  cameraTransformPublisher_.publish(base_transform);


}
void KinematicSimulation::publishArmState(const Observation& observation,std::string tf_prefix) {
  sensor_msgs::JointState armState;
  armState.header.stamp = ros::Time::now();
  armState.name = {tf_prefix +"shoulder_pan_joint", tf_prefix +"shoulder_lift_joint", tf_prefix +"elbow_joint", tf_prefix +"wrist_1_joint", tf_prefix +"wrist_2_joint", tf_prefix +"wrist_3_joint"};
  armState.position.resize(Definitions::ARM_STATE_DIM_);
  Eigen::VectorXd armConfiguration = observation.state().head<Definitions::POSITION_STATE_DIM_>().tail<6>();
  for (size_t joint_idx = 0; joint_idx < Definitions::ARM_STATE_DIM_; joint_idx++) {
    armState.position[joint_idx] = armConfiguration(joint_idx);
  }
  if(tf_prefix.length() == 0)
  {
    armStatePublisher_.publish(armState);
  }
  else if(tf_prefix == "mid")
  {
        midStatePublisher_.publish(armState);
  }
  else if(tf_prefix == "final")
  {
    finalStatePublisher_.publish(armState);
  }

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
//unchanged
kindr::HomTransformQuatD KinematicSimulation::getEndEffectorPose() {
  {
    boost::shared_lock<boost::shared_mutex> lock(observationMutex_);
    Eigen::Matrix<double, 4, 4> endEffectorToWorldTransform;
    Eigen::VectorXd currentState = observation_.state();
    // std::cerr << "currentState" << currentState.transpose() << std::endl;
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
    Eigen::VectorXd currentState = observation_.state();
    UR5Kinematics<double> kinematics(kinematicInterfaceConfig_);
    double EEOrientation =  kinematics.getEEOrientationAtan(currentState);
    
    return EEOrientation;
  }  
}
double KinematicSimulation::getManipulability(){
  {
    boost::shared_lock<boost::shared_mutex> lock(observationMutex_);
    Eigen::VectorXd currentState = observation_.state().head<POSITION_STATE_DIM_>().tail<6>();
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
    endEffectorToBaseTransform = kinematics.getArmTransform(currentState.head<POSITION_STATE_DIM_>().tail<6>());

    Eigen::Quaterniond eigenBaseRotation(endEffectorToBaseTransform.topLeftCorner<3, 3>());
    return kindr::HomTransformQuatD(kindr::Position3D(endEffectorToBaseTransform.topRightCorner<3, 1>()),
                                    kindr::RotationQuaternionD(eigenBaseRotation));
  }
}
std::shared_ptr<FiestaCostConfig> KinematicSimulation::configureCollisionAvoidance(
    std::shared_ptr<KinematicsInterfaceAD> kinematicInterface) {
  ros::NodeHandle pNh("~");
  std::shared_ptr<FiestaCostConfig> fiestaCostConfig = nullptr;
  if (pNh.hasParam("collision_points")) {
    graceful_mpc::PointsOnRobot::points_radii_t pointsAndRadii(8);
    using pair_t = std::pair<double, double>;

    XmlRpc::XmlRpcValue collisionPoints;
    pNh.getParam("collision_points", collisionPoints);
    if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("collision_points parameter is not of type array.");
      return fiestaCostConfig;
    }
    for (int i = 0; i < collisionPoints.size(); i++) {
      if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN_STREAM("collision_points[" << i << "] parameter is not of type array.");
        return fiestaCostConfig;
      }
      for (int j = 0; j < collisionPoints[i].size(); j++) {
        if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray) {
          ROS_WARN_STREAM("collision_points[" << i << "][" << j << "] parameter is not of type array.");
          return fiestaCostConfig;
        }
        if (collisionPoints[i][j].size() != 2) {
          ROS_WARN_STREAM("collision_points[" << i << "][" << j << "] does not have 2 elements.");
          return fiestaCostConfig;
        }
        double segmentId = collisionPoints[i][j][0];
        double radius = collisionPoints[i][j][1];
        pointsAndRadii[i].push_back(pair_t(segmentId, radius));
        // ROS_INFO_STREAM("segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius);
      }
    }
    graceful_mpc::PointsOnRobotConfig config;
    config.pointsAndRadii = pointsAndRadii;
    using ad_type = CppAD::AD<CppAD::cg::CG<double>>;
    config.kinematics = kinematicInterface;
    pointsOnRobot_.reset(new graceful_mpc::PointsOnRobot(config));

    if (pointsOnRobot_->numOfPoints() > 0) {
      fiestaCostConfig.reset(new FiestaCostConfig());
      fiestaCostConfig->pointsOnRobot = pointsOnRobot_;
      // jointSpaceRRTConfig_.pointsOnRobot = pointsOnRobot_;
      pointsOnRobot_->initialize("points_on_robot");
    } else {
      // if there are no points defined for collision checking, set this pointer to null to disable the visualization
      pointsOnRobot_ = nullptr;
    }
  }
  return fiestaCostConfig;
}

void KinematicSimulation::wholebodyStateCb(const std_msgs::Float64MultiArray& msg)
{

  std::vector<double> msg_in = msg.data;
  Eigen::VectorXd pf = Eigen::Map<Eigen::Matrix<double,18,1>>(msg_in.data(), msg_in.size());

  kindr::EulerAnglesRpyD rpy(0,0,pf.coeff(2));
  
  observationBuffer_.state() = pf.head<17>();
  observationBuffer_.time() = pf(17);

  latestObservationTime_ = ros::Time(pf(17));
  // std::cerr << "observationBuff state" << observationBuffer_.state().transpose()  << std::endl;
  // std::cerr << "observationBuff time" << observationBuffer_.time() << " " << latestObservationTime_ << std::endl;
  observationBuffer_.time() = latestObservationTime_.toSec();
  isFirstObservationReceived_ = true;
}
