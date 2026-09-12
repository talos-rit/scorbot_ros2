#include "scorbot_kinematics/scorbot_kinematics_plugin.hpp"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace scorbot_kinematics
{
namespace
{

Mat3 toMat3(const Eigen::Matrix3d& e)
{
  Mat3 m;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      m.m[i][j] = e(i, j);
  return m;
}

bool nearZero(double v, double tol = 1e-6) { return std::fabs(v) < tol; }

/// Read a solver parameter the way MoveIt namespaces kinematics.yaml entries:
/// robot_description_kinematics.<group>.<name>. Falls back to `default_value`.
double readDoubleParam(const rclcpp::Node::SharedPtr& node, const std::string& group,
                       const std::string& name, double default_value, const rclcpp::Logger& logger)
{
  const std::string key = "robot_description_kinematics." + group + "." + name;
  try
  {
    if (!node->has_parameter(key))
      node->declare_parameter<double>(key, default_value);
    return node->get_parameter(key).as_double();
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(logger, "Could not read parameter '%s' (%s); using %g", key.c_str(), e.what(), default_value);
    return default_value;
  }
}

}  // namespace

bool ScorbotKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node,
                                         const moveit::core::RobotModel& robot_model,
                                         const std::string& group_name, const std::string& base_frame,
                                         const std::vector<std::string>& tip_frames,
                                         double search_discretization)
{
  node_ = node;
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  logger_ = node->get_logger().get_child("scorbot_kinematics");

  jmg_ = robot_model.getJointModelGroup(group_name);
  if (!jmg_)
  {
    RCLCPP_ERROR(logger_, "Joint model group '%s' does not exist", group_name.c_str());
    return false;
  }
  if (tip_frames.size() != 1)
  {
    RCLCPP_ERROR(logger_, "Expected exactly one tip frame, got %zu", tip_frames.size());
    return false;
  }

  const auto& active = jmg_->getActiveJointModels();
  if (active.size() != static_cast<size_t>(kNumJoints))
  {
    RCLCPP_ERROR(logger_, "Group '%s' has %zu active joints; the Scorbot solver needs %d",
                 group_name.c_str(), active.size(), kNumJoints);
    return false;
  }
  joint_names_.clear();
  for (const auto* jm : active)
    joint_names_.push_back(jm->getName());
  link_names_ = jmg_->getLinkModelNames();

  orientation_tolerance_ = readDoubleParam(node, group_name, "orientation_tolerance", 3.2, logger_);

  if (!extractGeometry(robot_model, tip_frames[0]))
    return false;

  RCLCPP_INFO(logger_,
              "Scorbot analytic IK ready for group '%s': z0=%.4f a1=%.4f h1=%.4f L1=%.4f L2=%.4f dt=%.4f "
              "signs=[%g %g %g %g %g] base_frame='%s' tip='%s'",
              group_name.c_str(), geometry_.z0, geometry_.a1, geometry_.h1, geometry_.L1, geometry_.L2,
              geometry_.dt, geometry_.sign[0], geometry_.sign[1], geometry_.sign[2], geometry_.sign[3],
              geometry_.sign[4], base_frame.c_str(), tip_frames[0].c_str());
  return true;
}

bool ScorbotKinematicsPlugin::extractGeometry(const moveit::core::RobotModel& robot_model,
                                              const std::string& tip_frame)
{
  const auto& joints = jmg_->getActiveJointModels();

  // Axis pattern: z, -y, -y, -y, x (any sign). Record the signs.
  const std::array<std::array<double, 3>, kNumJoints> expected{{{0, 0, 1}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {1, 0, 0}}};
  for (int i = 0; i < kNumJoints; ++i)
  {
    const auto* rev = dynamic_cast<const moveit::core::RevoluteJointModel*>(joints[i]);
    if (!rev)
    {
      RCLCPP_ERROR(logger_, "Joint '%s' is not revolute", joints[i]->getName().c_str());
      return false;
    }
    const Eigen::Vector3d axis = rev->getAxis().normalized();
    const Eigen::Vector3d exp(expected[i][0], expected[i][1], expected[i][2]);
    const double dot = axis.dot(exp);
    if (!nearZero(std::fabs(dot) - 1.0, 1e-4))
    {
      RCLCPP_ERROR(logger_, "Joint '%s' axis (%.3f %.3f %.3f) is not along (%g %g %g)",
                   joints[i]->getName().c_str(), axis.x(), axis.y(), axis.z(), exp.x(), exp.y(), exp.z());
      return false;
    }
    geometry_.sign[i] = dot > 0 ? 1.0 : -1.0;

    const auto& bounds = rev->getVariableBounds();
    if (bounds.empty())
    {
      RCLCPP_ERROR(logger_, "Joint '%s' has no bounds", joints[i]->getName().c_str());
      return false;
    }
    geometry_.lower[i] = bounds[0].min_position_;
    geometry_.upper[i] = bounds[0].max_position_;
  }

  // Frames at the zero pose, expressed in the arm base link (parent of the base joint).
  (void)robot_model;  // robot_model_ (stored by storeValues) is the same model as a shared_ptr
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.updateLinkTransforms();

  arm_base_link_ = joints[0]->getParentLinkModel()->getName();
  const Eigen::Isometry3d T_root_armbase = state.getGlobalLinkTransform(arm_base_link_);
  const Eigen::Isometry3d T_root_baseframe = state.getGlobalLinkTransform(base_frame_);
  base_frame_to_arm_base_ = T_root_baseframe.inverse() * T_root_armbase;

  auto in_arm_base = [&](const std::string& link) {
    return T_root_armbase.inverse() * state.getGlobalLinkTransform(link);
  };

  std::array<Eigen::Isometry3d, kNumJoints> T;
  for (int i = 0; i < kNumJoints; ++i)
  {
    T[i] = in_arm_base(joints[i]->getChildLinkModel()->getName());
    const double misalignment = Eigen::AngleAxisd(T[i].rotation()).angle();
    if (!nearZero(misalignment, 1e-4))
    {
      RCLCPP_ERROR(logger_, "Link '%s' is rotated %.4f rad from the base at the zero pose; the solver "
                   "expects every arm frame aligned with the base at zero",
                   joints[i]->getChildLinkModel()->getName().c_str(), misalignment);
      return false;
    }
  }
  const Eigen::Isometry3d T_tip = in_arm_base(tip_frame);

  const Eigen::Vector3d p1 = T[0].translation();  // base joint
  const Eigen::Vector3d p2 = T[1].translation();  // shoulder
  const Eigen::Vector3d p3 = T[2].translation();  // elbow
  const Eigen::Vector3d p4 = T[3].translation();  // wrist pitch
  const Eigen::Vector3d p5 = T[4].translation();  // wrist roll
  const Eigen::Vector3d pt = T_tip.translation();

  if (!nearZero(p1.x(), 1e-4) || !nearZero(p1.y(), 1e-4) || !nearZero(p2.y(), 1e-4) ||
      !nearZero(p3.y(), 1e-4) || !nearZero(p4.y(), 1e-4))
  {
    RCLCPP_ERROR(logger_, "Arm joints are not in the x-z plane at the zero pose");
    return false;
  }
  if (!nearZero(p3.z() - p2.z(), 1e-4) || !nearZero(p4.z() - p3.z(), 1e-4) || p3.x() <= p2.x() || p4.x() <= p3.x())
  {
    RCLCPP_ERROR(logger_, "Upper arm and forearm must lie along +x at the zero pose");
    return false;
  }
  if ((p5 - p4).norm() > 1e-4)
  {
    RCLCPP_ERROR(logger_, "Wrist roll axis must intersect the wrist pitch axis (offset %.4f m)", (p5 - p4).norm());
    return false;
  }
  const Eigen::Vector3d tool_offset = pt - p5;
  if (!nearZero(tool_offset.y(), 1e-4) || !nearZero(tool_offset.z(), 1e-4))
  {
    RCLCPP_ERROR(logger_, "Tip frame must sit on the flange axis (offset y=%.4f z=%.4f). Put camera mount "
                 "offsets on a separate link and keep tool0 on the axis.", tool_offset.y(), tool_offset.z());
    return false;
  }

  geometry_.z0 = p1.z();
  geometry_.a1 = p2.x();
  geometry_.h1 = p2.z() - p1.z();
  geometry_.L1 = p3.x() - p2.x();
  geometry_.L2 = p4.x() - p3.x();
  geometry_.dt = tool_offset.x();
  geometry_.R_tool = toMat3(T_tip.rotation());
  return true;
}

bool ScorbotKinematicsPlugin::solve(const geometry_msgs::msg::Pose& ik_pose,
                                    const std::vector<double>& ik_seed_state,
                                    const std::vector<double>& consistency_limits,
                                    std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                    moveit_msgs::msg::MoveItErrorCodes& error_code,
                                    const kinematics::KinematicsQueryOptions& options) const
{
  if (ik_seed_state.size() != static_cast<size_t>(kNumJoints))
  {
    RCLCPP_ERROR(logger_, "Seed state has %zu values, expected %d", ik_seed_state.size(), kNumJoints);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  // Requested tool pose: base_frame -> arm base link.
  Eigen::Isometry3d T_pose;
  tf2::fromMsg(ik_pose, T_pose);
  const Eigen::Isometry3d T = base_frame_to_arm_base_.inverse() * T_pose;

  const Vec3 p{T.translation().x(), T.translation().y(), T.translation().z()};
  const Mat3 R = toMat3(T.rotation());
  std::array<double, kNumJoints> seed{};
  for (int i = 0; i < kNumJoints; ++i)
    seed[i] = ik_seed_state[i];

  const auto candidates = inverseKinematics(geometry_, p, R, seed);
  for (const auto& cand : candidates)
  {
    if (cand.orientation_error > orientation_tolerance_ && !options.return_approximate_solution)
      continue;

    bool consistent = true;
    for (size_t i = 0; i < consistency_limits.size() && i < static_cast<size_t>(kNumJoints); ++i)
      consistent = consistent && std::fabs(cand.q[i] - seed[i]) <= consistency_limits[i];
    if (!consistent)
      continue;

    solution.assign(cand.q.begin(), cand.q.end());
    if (solution_callback)
    {
      solution_callback(ik_pose, solution, error_code);
      if (error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        continue;  // rejected (collision etc.), try the next branch
    }
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

bool ScorbotKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state,
                                            std::vector<double>& solution,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const
{
  return solve(ik_pose, ik_seed_state, {}, solution, IKCallbackFn(), error_code, options);
}

bool ScorbotKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state, double /*timeout*/,
                                               std::vector<double>& solution,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  return solve(ik_pose, ik_seed_state, {}, solution, IKCallbackFn(), error_code, options);
}

bool ScorbotKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state, double /*timeout*/,
                                               const std::vector<double>& consistency_limits,
                                               std::vector<double>& solution,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  return solve(ik_pose, ik_seed_state, consistency_limits, solution, IKCallbackFn(), error_code, options);
}

bool ScorbotKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state, double /*timeout*/,
                                               std::vector<double>& solution,
                                               const IKCallbackFn& solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  return solve(ik_pose, ik_seed_state, {}, solution, solution_callback, error_code, options);
}

bool ScorbotKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state, double /*timeout*/,
                                               const std::vector<double>& consistency_limits,
                                               std::vector<double>& solution,
                                               const IKCallbackFn& solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  return solve(ik_pose, ik_seed_state, consistency_limits, solution, solution_callback, error_code, options);
}

bool ScorbotKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                            const std::vector<double>& joint_angles,
                                            std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (joint_angles.size() != static_cast<size_t>(kNumJoints))
    return false;

  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(jmg_, joint_angles);
  state.updateLinkTransforms();

  const Eigen::Isometry3d T_root_baseframe = state.getGlobalLinkTransform(base_frame_);
  poses.clear();
  poses.reserve(link_names.size());
  for (const auto& link : link_names)
  {
    if (!robot_model_->hasLinkModel(link))
      return false;
    poses.push_back(tf2::toMsg(T_root_baseframe.inverse() * state.getGlobalLinkTransform(link)));
  }
  return true;
}

}  // namespace scorbot_kinematics

PLUGINLIB_EXPORT_CLASS(scorbot_kinematics::ScorbotKinematicsPlugin, kinematics::KinematicsBase)
