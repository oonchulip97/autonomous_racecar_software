// Copyright 2021 Oon Chu Lip

#include <algorithm>
#include <cmath>
#include <iterator>

#include "costmap_2d/costmap_2d.h"
#include "geometry_msgs/Quaternion.h"
#include "pluginlib/class_list_macros.h"
#include "ros/ros.h"
#include "tf2/utils.h"

#include "auto_racecar/pure_pursuit_planner.h"

PLUGINLIB_EXPORT_CLASS(auto_racecar::PurePursuitPlanner,
                       nav_core::BaseLocalPlanner)

namespace auto_racecar {
PurePursuitPlanner::PurePursuitPlanner() {}

PurePursuitPlanner::~PurePursuitPlanner() {}

void PurePursuitPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                                    costmap_2d::Costmap2DROS* costmap_ros) {
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("wheel_base", wheel_base_, 0.3);
  private_nh.param("lookahead_dist", lookahead_dist_, 0.5);
  private_nh.param("linear_vel", linear_vel_, 2.0);
  private_nh.param("min_scaling_vel", min_scaling_vel_, 1.5);
  private_nh.param("min_scaling_radius", min_scaling_radius_, 0.5);
  private_nh.param("max_angular_pos", max_angular_pos_, 0.5);
  private_nh.param("transform_tolerance", transform_tolerance_, 0.2);
  private_nh.param("goal_tolerance", goal_tolerance_, 0.2);
  private_nh.param("collision_projected_time", collision_projected_time_, 0.4);
  private_nh.param("enable_vel_scaling", enable_vel_scaling_, false);
  private_nh.param("enable_collision_checking", enable_collision_checking_,
                   false);

  is_plan_received_ = false;
  is_goal_reached_ = false;

  local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  target_point_pub_ =
      private_nh.advertise<geometry_msgs::PointStamped>("target_point", 1);
  collision_arc_pub_ = private_nh.advertise<nav_msgs::Path>("collision_arc", 1);

  ROS_INFO("PurePursuitPlanner initialized");
}

bool PurePursuitPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  plan_ = plan;
  const bool is_plan_empty = plan_.empty();
  if (!is_plan_empty) {
    ROS_INFO("PurePursuitPlanner: New plan Received");
    is_plan_received_ = true;
  } else {
    ROS_ERROR("PurePursuitPlanner: Cannot set new plan");
    is_plan_received_ = false;
  }
  is_goal_reached_ = false;
  return !is_plan_empty;
}

bool PurePursuitPlanner::computeVelocityCommands(
    geometry_msgs::Twist& cmd_vel) {
  // Get robot pose
  updateRobotPose();

  // Whether goal has been reached
  if (!updateGoalReached()) {
    ROS_ERROR("PurePursuitPlanner: Cannot update whether goal is reached");
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped target_point;
  double angular_error, actual_dist;
  // Get plan in robot's frame
  if (transformPlan(plan)) {
    // Get target point in robot's frame
    target_point = getTargetPoint(plan);
    // Get angular error in robot's frame
    angular_error =
        std::atan2(target_point.pose.position.y, target_point.pose.position.x);
    // Get actual lookahead distance
    actual_dist = std::hypot(target_point.pose.position.x,
                             target_point.pose.position.y);

    // Publish for visualization
    local_plan_pub_.publish(createPlanMsg(plan));
    target_point_pub_.publish(createTargetMsg(target_point));
  } else {
    ROS_ERROR("PurePursuitPlanner: Cannot transform plan");
    return false;
  }

  // Update command velocity
  if (is_plan_received_ && !is_goal_reached_) {
    // Scale velocity based on curvature
    if (enable_vel_scaling_) {
      cmd_vel.linear.x = computeLinearVel(angular_error, actual_dist);
    } else {
      cmd_vel.linear.x = linear_vel_;
    }
    cmd_vel.angular.z = computeSteeringAngle(angular_error, actual_dist);
  } else {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
  }

  // Clamp angular steer
  cmd_vel.angular.z =
      std::max(-1 * std::abs(max_angular_pos_),
               std::min(std::abs(max_angular_pos_), cmd_vel.angular.z));

  // Check for collision
  if (enable_collision_checking_) {
    return !checkImminentCollision(cmd_vel);
  }
  return true;
}

bool PurePursuitPlanner::isGoalReached() {
  if (is_goal_reached_) {
    ROS_INFO("PurePursuitPlanner: Goal reached");
  }
  return is_goal_reached_;
}

double PurePursuitPlanner::getEuclideanDistance(
    const geometry_msgs::PoseStamped& pose1,
    const geometry_msgs::PoseStamped& pose2) const {
  const double dx = pose1.pose.position.x - pose2.pose.position.x;
  const double dy = pose1.pose.position.y - pose2.pose.position.y;
  return std::hypot(dx, dy);
}

void PurePursuitPlanner::updateRobotPose() {
  costmap_ros_->getRobotPose(robot_pose_);
}

bool PurePursuitPlanner::updateGoalReached() {
  geometry_msgs::PoseStamped robot_pose;

  // Get robot pose in plan's frame
  if (!transformPose(plan_[0].header.frame_id, robot_pose_, robot_pose)) {
    ROS_ERROR("PurePursuitPlanner: Cannot transform robot pose");
    return false;
  }

  // Check whether last point is far away
  const double distance_to_goal =
      getEuclideanDistance(plan_.back(), robot_pose);
  if (distance_to_goal <= goal_tolerance_) {
    is_goal_reached_ = true;
  } else {
    is_goal_reached_ = false;
  }
  return true;
}

bool PurePursuitPlanner::transformPose(
    const std::string& frame, const geometry_msgs::PoseStamped& in_pose,
    geometry_msgs::PoseStamped& out_pose) const {
  // No transformation if frames are identical
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform<geometry_msgs::PoseStamped>(
        in_pose, out_pose, frame, ros::Duration(transform_tolerance_));
    return true;
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("PurePursuitPlanner: Exception in transform pose - %s",
              ex.what());
  }
  return false;
}

bool PurePursuitPlanner::transformPlan(
    std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
  // Get robot pose in plan's frame
  geometry_msgs::PoseStamped robot_pose;
  if (!transformPose(plan_[0].header.frame_id, robot_pose_, robot_pose)) {
    ROS_ERROR("PurePursuitPlanner: Cannot transform robot pose");
    return false;
  }

  // Get maximum transform distance from costmap
  const costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  const double max_transform_dist =
      std::max(costmap->getSizeInMetersX(), costmap->getSizeInMetersY()) / 2.0;

  // Find closest point from plan to robot
  std::vector<geometry_msgs::PoseStamped>::iterator closest = plan_.begin();
  double min_dist_plan_robot = getEuclideanDistance(*closest, robot_pose);
  double dist_plan_robot;
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = plan_.begin();
       it != plan_.end(); it++) {
    dist_plan_robot = getEuclideanDistance(*it, robot_pose);
    if (dist_plan_robot <= min_dist_plan_robot) {
      closest = it;
      min_dist_plan_robot = dist_plan_robot;
    }
  }

  // Find far away points to discard
  std::vector<geometry_msgs::PoseStamped>::iterator furthest = plan_.end();
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = closest;
       it != plan_.end(); it++) {
    if (getEuclideanDistance(*it, robot_pose) > max_transform_dist) {
      furthest = it;
      break;
    }
  }

  // Get plan in robot's frame
  // NOTE: We need to update timestamp of points before transform
  // because points of plan is generated long time ago
  geometry_msgs::PoseStamped stamped_point, transformed_point;
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = closest;
       it != furthest; it++) {
    stamped_point = *it;
    stamped_point.header.stamp = robot_pose.header.stamp;
    if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_point,
                       transformed_point)) {
      ROS_ERROR("PurePursuitPlanner: Cannot transform plan point");
      return false;
    }
    transformed_plan.push_back(transformed_point);
  }

  // Prune the plan
  plan_.erase(plan_.begin(), closest);

  // Check whether transformed plan is too far away
  if (transformed_plan.empty()) {
    ROS_ERROR("PurePursuitPlanner: Transformed plan is empty.");
    return false;
  }

  return true;
}

geometry_msgs::PoseStamped PurePursuitPlanner::getTargetPoint(
    const std::vector<geometry_msgs::PoseStamped>& plan) const {
  // Find first point which is further than lookahead distance
  std::vector<geometry_msgs::PoseStamped>::const_iterator target_point =
      plan.end();
  for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan.begin();
       it != plan.end(); it++) {
    if (std::hypot((*it).pose.position.x, (*it).pose.position.y) >=
        lookahead_dist_) {
      target_point = it;
      break;
    }
  }

  // No point is further than lookahead distance, take last point
  if (target_point == plan.end()) {
    target_point = std::prev(plan.end());
  }

  return *target_point;
}

nav_msgs::Path PurePursuitPlanner::createPlanMsg(
    const std::vector<geometry_msgs::PoseStamped>& plan) const {
  nav_msgs::Path msg;
  msg.header.stamp = plan[0].header.stamp;
  msg.header.frame_id = plan[0].header.frame_id;
  msg.poses = plan;
  return msg;
}

geometry_msgs::PointStamped PurePursuitPlanner::createTargetMsg(
    const geometry_msgs::PoseStamped& target) const {
  geometry_msgs::PointStamped msg;
  msg.header.stamp = target.header.stamp;
  msg.header.frame_id = target.header.frame_id;
  msg.point.x = target.pose.position.x;
  msg.point.y = target.pose.position.y;
  msg.point.z = 0.1;  // Hover over map to stand out
  return msg;
}

double PurePursuitPlanner::computeLinearVel(double angular_error,
                                            double actual_dist) const {
  // Limit linear velocity by curvature
  const double curvature = std::fabs(2 * std::sin(angular_error) / actual_dist);
  const double radius = 1 / curvature;
  double curvature_vel = linear_vel_;
  if (radius < min_scaling_radius_) {
    curvature_vel *= radius / min_scaling_radius_;
  }
  // Ensure linear velocity is at least minimum
  return std::max(curvature_vel, min_scaling_vel_);
}

double PurePursuitPlanner::computeSteeringAngle(double angular_error,
                                                double actual_dist) const {
  if (angular_error > 1.57){
    return max_angular_pos_;
  } else if (angular_error < -1.57) {
    return -max_angular_pos_;
  }
  return std::atan2(2 * wheel_base_ * std::sin(angular_error), actual_dist);
}

bool PurePursuitPlanner::checkImminentCollision(
    const geometry_msgs::Twist& cmd_vel) const {
  // Check current position is not in collision
  if (inCollision(robot_pose_)) {
    ROS_INFO("PurePursuitPlanner: Collision detected at current position");
    return true;
  }

  // Do not check projected trajectory if robot is stationary
  if (std::fabs(cmd_vel.linear.x) <= 0.01) {
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> collision_arc;
  geometry_msgs::PoseStamped collision_point;
  collision_point.header.stamp = robot_pose_.header.stamp;
  collision_point.header.frame_id = costmap_ros_->getGlobalFrameID();

  const double cell_time =
      costmap_ros_->getCostmap()->getResolution() / cmd_vel.linear.x;

  // Project trajectory forward in time and check for collision
  double curr_x = robot_pose_.pose.position.x;
  double curr_y = robot_pose_.pose.position.y;
  double curr_theta =
      tf2::getYaw<geometry_msgs::Quaternion>(robot_pose_.pose.orientation);
  size_t idx = 1;
  while (true) {
    // Only project trajectory up to maximum defined time
    if (idx * cell_time > collision_projected_time_) {
      break;
    }

    // Project trajectory using rear wheel reference on bicycle model
    curr_x += cell_time * cmd_vel.linear.x * cos(curr_theta);
    curr_y += cell_time * cmd_vel.linear.x * sin(curr_theta);
    curr_theta += cell_time * (cmd_vel.linear.x * std::tan(cmd_vel.angular.z))
                  / wheel_base_;

    collision_point.pose.position.x = curr_x;
    collision_point.pose.position.y = curr_y;
    collision_point.pose.position.z = 0.1;  // Hover over map to stand out
    collision_arc.push_back(collision_point);

    // Check for collision
    if (inCollision(collision_point)) {
      collision_arc_pub_.publish(createPlanMsg(collision_arc));
      ROS_INFO("PurePursuitPlanner: Collision detected at projected path");
      return true;
    }
    idx++;
  }
  // No collision
  collision_arc_pub_.publish(createPlanMsg(collision_arc));
  return false;
}

bool PurePursuitPlanner::inCollision(
    const geometry_msgs::PoseStamped& pose) const {
  unsigned int mx, my;
  const costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
  const unsigned char cost = costmap->getCost(mx, my);

  return cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
         cost != costmap_2d::NO_INFORMATION;
}
}  // namespace auto_racecar
