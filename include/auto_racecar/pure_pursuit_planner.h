// Copyright 2021 Oon Chu Lip

#ifndef INCLUDE_AUTO_RACECAR_PURE_PURSUIT_PLANNER_H_
#define INCLUDE_AUTO_RACECAR_PURE_PURSUIT_PLANNER_H_

#include <string>
#include <vector>

#include "costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_core/base_local_planner.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"

namespace auto_racecar {
class PurePursuitPlanner : public nav_core::BaseLocalPlanner {
 public:
  /**
   * \brief Constructor.
   */
  PurePursuitPlanner();

  /**
   * \brief Destructor.
   */
  ~PurePursuitPlanner();

  /**
   * \brief Constructs local planner.
   *
   * \param name name to give this instance of local planner
   * \param tf transform listener
   * \param costmap_ros cost map used to assigning costs to local plan
   */
  void initialize(std::string name, tf2_ros::Buffer* tf,
                          costmap_2d::Costmap2DROS* costmap_ros) override;

  /**
   * \brief Sets plan the local planner is following. Ensure plan is filled.
   *
   * \param plan to pass to local planner
   *
   * \return true if plan is updated successfully, false otherwise
   */
  bool setPlan(
      const std::vector<geometry_msgs::PoseStamped>& plan) override;

  /**
   * \brief Given current position, orientation and velocity of robot,
   *        computes command velocity
   *
   * \param cmd_vel command velocity to be sent to robot base
   *
   * \return true if valid command is found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

  /**
   * \brief Checks if goal pose has been achieved by local planner
   *
   * \return true if achieved, false otherwise
   */
  bool isGoalReached() override;

 private:
  /**
   * \brief Calculates Euclidean distance between two points
   *
   * \param pose1 position 1
   * \param pose2 position 2
   *
   * \return Euclidean distance between two points
   */
  double getEuclideanDistance(const geometry_msgs::PoseStamped& pose1,
                              const geometry_msgs::PoseStamped& pose2) const;

  /**
   * \brief Updates current pose of robot
   */
  void updateRobotPose();

  /**
   * \brief Updates whether goal pose has been achieved by local planner
   *
   * \return true if update is successful, false otherwise
   */
  bool updateGoalReached();

  /**
   * \brief Transforms pose from one frame to another
   *
   * \param frame frame ID to transform into
   * \param in_pose pose to be transformed
   * \param out_pose pose after transformation
   *
   * \return true if transformation is successful, false otherwise
   */
  bool transformPose(const std::string& frame,
                     const geometry_msgs::PoseStamped& in_pose,
                     geometry_msgs::PoseStamped& out_pose) const;

  /**
   * \brief Transforms global plan from plan's frame into robot's frame
   *
   * \param transformed_plan plan after transformation
   *
   * \return true if transformation is successful, false otherwise
   */
  bool transformPlan(
      std::vector<geometry_msgs::PoseStamped>& transformed_plan);

  /**
   * \brief Calculates target point
   *
   * \param plan plan in robot's frame
   *
   * \return target point
   *
   */
  geometry_msgs::PoseStamped getTargetPoint(
      const std::vector<geometry_msgs::PoseStamped>& plan) const;

  /**
   * \brief Convert local plan message type for visualization
   *
   * \param plan current local plan
   *
   * \return converted local plan
   */
  nav_msgs::Path createPlanMsg(
      const std::vector<geometry_msgs::PoseStamped>& plan) const;

  /**
   * \brief Convert target point message type for visualization
   *
   * \param target current target point
   *
   * \return converted target point
   */
  geometry_msgs::PointStamped createTargetMsg(
      const geometry_msgs::PoseStamped& target) const;

  /**
   * \brief Computes linear velocity based on curvature
   *
   * The higher the curvature to target point, the slower the linear velocity
   *
   * \param angular_error error from robot's heading to target's heading
   *                      in robot's frame
   * \param actual_dist distance from robot pose to target point
   *
   * \return linear_vel linear velocity of robot
   */
  double computeLinearVel(double angular_error, double actual_dist) const;

  /**
   * \brief Computes steering angle for velocity command
   *
   * \param angular_error error from robot's heading to target's heading
   *                      in robot's frame
   * \param actual_dist distance from robot pose to target point
   *
   * \return steering angle to drive robot back to plan
   */
  double computeSteeringAngle(double angular_error, double actual_dist) const;

  /**
   * \brief Checks collision using projected trajectory from velocity command
   *
   * \param cmd_vel velocity command to project trajectory
   *
   * \return true if there is collision, false otherwise
   */
  bool checkImminentCollision(const geometry_msgs::Twist& cmd_vel) const;

  /**
   * \brief Checks collision at the position using the costmap
   *
   * \param pose position in costmap's frame
   *
   * \return true if there is collision, false otherwise
   */
  bool inCollision(const geometry_msgs::PoseStamped& pose) const;

  std::string name_;                      // name of local planner
  tf2_ros::Buffer* tf_;                   // transform buffer
  costmap_2d::Costmap2DROS* costmap_ros_;  // costmap
  double wheel_base_;           // wheel base between front and back axles
  double lookahead_dist_;   // lookahead distance to target point
  double linear_vel_;           // desired linear velocity
  double min_scaling_vel_;      // minimum linear velocity after scaling
  double min_scaling_radius_;   // turning radius which scaling is triggered
  double max_angular_pos_;      // maximum angular position for steering
  double transform_tolerance_;  // transform tolerance
  double goal_tolerance_;       // goal tolerance
  double collision_projected_time_;  // project trajectory how far into future
  bool enable_vel_scaling_;          // scale linear velocity based on curvature
  bool enable_collision_checking_;   // stop if collision within projected path
  ros::Publisher local_plan_pub_;    // publisher for local plan
  ros::Publisher target_point_pub_;  // publisher for target point
  ros::Publisher collision_arc_pub_;              // publisher for collision arc
  bool is_plan_received_;                          // whether plan is received
  bool is_goal_reached_;                           // whether goal is reached
  std::vector<geometry_msgs::PoseStamped> plan_;  // plan in plan's frame
  geometry_msgs::PoseStamped robot_pose_;         // robot in costmap's frame
}; // class PurePursuitPlanner
}  // namespace auto_racecar

#endif  // INCLUDE_AUTO_RACECAR_PURE_PURSUIT_PLANNER_H_
