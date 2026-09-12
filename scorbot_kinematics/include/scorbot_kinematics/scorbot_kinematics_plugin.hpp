// MoveIt 2 kinematics plugin wrapping the closed-form solver in analytic_ik.hpp.
//
// Geometry (link lengths, axis signs, joint limits, tool offset) is read from the
// loaded robot model at initialize(), so the same plugin serves the ER-V and the
// ER-4pc with any link prefix and no hard-coded numbers.

#pragma once

#if __has_include(<moveit/kinematics_base/kinematics_base.hpp>)
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#else
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#endif

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "scorbot_kinematics/analytic_ik.hpp"

namespace scorbot_kinematics
{

class ScorbotKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames, double search_discretization) override;

  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options =
                         kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, const std::vector<double>& consistency_limits,
                        std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, const std::vector<double>& consistency_limits,
                        std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override;

  const std::vector<std::string>& getJointNames() const override { return joint_names_; }
  const std::vector<std::string>& getLinkNames() const override { return link_names_; }

private:
  /// Common implementation behind every IK entry point.
  bool solve(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
             const std::vector<double>& consistency_limits, std::vector<double>& solution,
             const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
             const kinematics::KinematicsQueryOptions& options) const;

  /// Derive Geometry from the robot model at the zero pose. Logs and returns false on a
  /// chain that does not match the Scorbot layout.
  bool extractGeometry(const moveit::core::RobotModel& robot_model, const std::string& tip_frame);

  Geometry geometry_;
  const moveit::core::JointModelGroup* jmg_{nullptr};
  std::string arm_base_link_;                ///< parent link of the base joint (base_link)
  Eigen::Isometry3d base_frame_to_arm_base_;  ///< T(base_frame -> base_link), fixed
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  double orientation_tolerance_{3.2};        ///< rad; larger errors fail unless approximate allowed
  rclcpp::Logger logger_{rclcpp::get_logger("scorbot_kinematics")};
};

}  // namespace scorbot_kinematics
