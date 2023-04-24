//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace legged {
using namespace ocs2;

class TargetTrajectoriesPublisher final {
 public:
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories,CmdToTargetTrajectories navSeqToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        navSeqToTargetTrajectories_(std::move(navSeqToTargetTrajectories)),
        tf2_(buffer_) {
    // Trajectories publisher
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    // goal subscriber
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      try {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("Failure %s\n", ex.what());
        return;
      }

      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    // cmd_vel subscriber
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }

      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x;
      cmdVel[1] = msg->linear.y;
      cmdVel[2] = msg->linear.z;
      cmdVel[3] = msg->angular.z;

      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };
// zwt #############################################################################################
    // nav_seq subscriber
    auto navSeqCallback = [this](const nav_msgs::Path::ConstPtr & msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }
    //使用 geometry_msgs::Pose 来存储[x,y,q]和[dx,dy,dq] 并且[x,y,q]是相对于当前baselink/odometry的偏移量;速度是世界坐标系下的速度
    // 对应的数据格式: x->pose.position.x, y->pose.position.y, q->pose.position.z
    //              dx->pose.orientation.x, dy->pose.orientation.y, dq->pose.orientation.z
    // geometry_msgs::PoseStamped: http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html
    vector_t navSeq = vector_t::Zero(msg->poses.size()*6);
    for (int idx=0;idx < msg->poses.size();idx++){
      navSeq[idx*6+0] = msg->poses[idx].pose.position.x;
      navSeq[idx*6+1] = msg->poses[idx].pose.position.y;
      navSeq[idx*6+2] = msg->poses[idx].pose.position.z;
      navSeq[idx*6+3] = msg->poses[idx].pose.orientation.x;
      navSeq[idx*6+4] = msg->poses[idx].pose.orientation.y;
      navSeq[idx*6+5] = msg->poses[idx].pose.orientation.z;
    }
    const auto trajectories = navSeqToTargetTrajectories_(navSeq, latestObservation_);
    targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };
// zwt #############################################################################################
    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);
    // nav_msgs::Path reference: http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html
    navseqSub_ = nh.subscribe<nav_msgs::Path>("/nav_seq", 1, navSeqCallback);
  }

 private:
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
  // zwt add : for nav_seq trajectory tracking
  CmdToTargetTrajectories navSeqToTargetTrajectories_;
  ::ros::Subscriber navseqSub_;
  double _time_interval=0.1;
};

}  // namespace legged
