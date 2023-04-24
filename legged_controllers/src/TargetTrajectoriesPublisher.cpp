//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/TargetTrajectoriesPublisher.h"
#include <angles/angles.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace legged;

namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
// zwt add trajectory tracking
scalar_t _time_interval = 0.1;
}  // namespace

//estimateTimeToTarget 默认是匀速运动
scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

// zwt 这里非常重要，将目标点转换为时间轨迹，状态轨迹，输入轨迹,但是这就只是两个点,开始点和目标点
// 如果我们的工作是轨迹跟踪,那么是不是就把这里转化为多个点的轨迹呢?
// 但是需要搞清楚的是,stateTrajectory 的具体含义是什么呢
// 具体含义见 task.info : initialState
TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime) {
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  currentPose(2) = COM_HEIGHT;
  currentPose(4) = 0;
  currentPose(5) = 0;
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));
  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = COM_HEIGHT;
    target(3) = goal(3);
    target(4) = 0;
    target(5) = 0;
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}

//Tag: zwt targetTrajectories from cmdVel base on current_observation
TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  // rotation matrix from current pose to cmdVel
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);
  // TIME_TO_TARGET from task.info map.timeHorizon
  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = COM_HEIGHT;
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // target reaching duration
  //################################################################################################
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}

// zwt #############################################################################################
TargetTrajectories navSeqToTargetTrajectories(const vector_t& navSeq, const SystemObservation& observation)
{
  int SeqSize = navSeq.size()/6;// 当前点+目标点
  // desired time trajectory
  scalar_array_t timeTrajectory(SeqSize, 1);
  for (int idx = 0; idx < SeqSize; idx++)
  {
    timeTrajectory.at(idx) =  observation.time + idx * _time_interval;
  }
  // desired state trajectory
  vector_t curVel = observation.state.segment<6>(0);
  vector_t curPose = observation.state.segment<6>(6);
  vector_t midVel = curVel;
  midVel(2) = 0; // z_
  midVel(3) = 0; // L_x
  midVel(4) = 0; // L_y
  vector_t midPose = curPose;
  midPose(2) = COM_HEIGHT;
  midPose(4) = 0; //theta_y
  midPose(5) = 0; //theta_x
  scalar_t delta_x=0,delta_y=0,delta_q=0,_last_q=0;
  delta_x = curPose(0) - navSeq(0); // x odometry basis
  delta_y = curPose(1) - navSeq(1); // y odometry basis
  delta_q = curPose(3) - navSeq(2); // q odometry basis
  vector_array_t stateTrajectory(SeqSize, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << midVel, midPose, DEFAULT_JOINT_STATE;
  _last_q = midPose(3);
  for (int idx = 1; idx < SeqSize; idx++)
  {
    // vel 还是世界坐标系下的数值
    midVel(0)=navSeq(idx*6+3);midVel(1)=navSeq(idx*6+4);midVel(5)=navSeq(idx*6+5);
    // pose 是相对于当前位置的数值 + 当前位置
    midPose(0) = navSeq(idx*6+0) + delta_x;
    midPose(1) = navSeq(idx*6+1) + delta_y;
    scalar_t _q_ref = navSeq(idx*6+2) + delta_q;
    // _q_ref = angles::normalize_angle(_q_ref);
    _q_ref = angles::shortest_angular_distance(_last_q, _q_ref);
    midPose(3) = _last_q + _q_ref;
    _last_q = midPose(3); // update last q
    stateTrajectory[idx] << midVel, midPose, DEFAULT_JOINT_STATE;
  }

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(SeqSize, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}
// zwt #############################################################################################
int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string referenceFile;
  std::string taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/taskFile", taskFile);

  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories, &navSeqToTargetTrajectories);

  ros::spin();
  // Successful exit
  return 0;
}
