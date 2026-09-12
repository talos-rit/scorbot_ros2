#include "scorbot_hardware/calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

namespace scorbot_hardware
{

namespace
{

template <typename T>
bool readField(const YAML::Node& joint, const std::string& joint_name, const char* key, T& out,
               std::string& error)
{
  const YAML::Node n = joint[key];
  if (!n || !n.IsScalar())
  {
    error = "joint '" + joint_name + "': missing '" + key + "'";
    return false;
  }
  try
  {
    out = n.as<T>();
  }
  catch (const YAML::Exception& e)
  {
    error = "joint '" + joint_name + "': '" + key + "': " + e.what();
    return false;
  }
  return true;
}

bool convert(const YAML::Node& root, CalibrationFile& out, std::string& error)
{
  if (!root || !root.IsMap())
  {
    error = "calibration must be a mapping with 'robot_type' and 'joints'";
    return false;
  }
  if (root["robot_type"] && root["robot_type"].IsScalar())
    out.robot_type = root["robot_type"].as<std::string>();
  const YAML::Node joints = root["joints"];
  if (!joints || !joints.IsMap() || joints.size() == 0)
  {
    error = "calibration needs a non-empty 'joints' mapping";
    return false;
  }
  for (const auto& kv : joints)
  {
    const std::string name = kv.first.as<std::string>();
    const YAML::Node j = kv.second;
    if (!j.IsMap())
    {
      error = "joint '" + name + "' must be a mapping";
      return false;
    }
    scorbot_protocol::JointCalibration c = scorbot_v1_JointCalibration_init_zero;
    int counts = 0, direction = 0;
    double gear = 0, home = 0, smin = 0, smax = 0, vmax = 0, amax = 0, imax = 0;
    bool invert = false, encoder_invert = false;
    if (!readField(j, name, "counts_per_motor_rev", counts, error) || !readField(j, name, "gear_ratio", gear, error) ||
        !readField(j, name, "home_offset_rad", home, error) || !readField(j, name, "home_direction", direction, error) ||
        !readField(j, name, "soft_limit_min_rad", smin, error) || !readField(j, name, "soft_limit_max_rad", smax, error) ||
        !readField(j, name, "max_velocity_rad_s", vmax, error) || !readField(j, name, "max_accel_rad_s2", amax, error) ||
        !readField(j, name, "current_limit_a", imax, error) || !readField(j, name, "invert", invert, error) ||
        !readField(j, name, "encoder_invert", encoder_invert, error))
      return false;
    if (counts <= 0 || gear <= 0.0 || vmax <= 0.0 || amax <= 0.0 || imax <= 0.0)
    {
      error = "joint '" + name + "': counts, ratios and limits must be positive";
      return false;
    }
    if (!(smin < smax))
    {
      error = "joint '" + name + "': soft_limit_min_rad must be below soft_limit_max_rad";
      return false;
    }
    if (direction != 1 && direction != -1)
    {
      error = "joint '" + name + "': home_direction must be 1 or -1";
      return false;
    }
    c.counts_per_motor_rev = counts;
    c.gear_ratio = static_cast<float>(gear);
    c.home_offset_rad = static_cast<float>(home);
    c.home_direction = direction;
    c.soft_limit_min_rad = static_cast<float>(smin);
    c.soft_limit_max_rad = static_cast<float>(smax);
    c.max_velocity_rad_s = static_cast<float>(vmax);
    c.max_accel_rad_s2 = static_cast<float>(amax);
    c.current_limit_a = static_cast<float>(imax);
    c.invert = invert;
    c.encoder_invert = encoder_invert;
    out.joints[name] = c;
  }
  return true;
}

}  // namespace

bool loadCalibrationFile(const std::string& path, CalibrationFile& out, std::string& error)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(path);
  }
  catch (const YAML::Exception& e)
  {
    error = "cannot read calibration '" + path + "': " + e.what();
    return false;
  }
  return convert(root, out, error);
}

bool parseCalibration(const std::string& yaml_text, CalibrationFile& out, std::string& error)
{
  YAML::Node root;
  try
  {
    root = YAML::Load(yaml_text);
  }
  catch (const YAML::Exception& e)
  {
    error = std::string("cannot parse calibration: ") + e.what();
    return false;
  }
  return convert(root, out, error);
}

std::string stripPrefix(const std::string& name, const std::string& prefix)
{
  if (!prefix.empty() && name.compare(0, prefix.size(), prefix) == 0)
    return name.substr(prefix.size());
  return name;
}

bool orderCalibration(const CalibrationFile& file, const std::vector<std::string>& joint_names,
                      const std::string& prefix, std::vector<scorbot_protocol::JointCalibration>& out,
                      std::string& error)
{
  out.clear();
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    const std::string key = stripPrefix(joint_names[i], prefix);
    const auto it = file.joints.find(key);
    if (it == file.joints.end())
    {
      error = "calibration has no entry for joint '" + key + "'";
      return false;
    }
    scorbot_protocol::JointCalibration c = it->second;
    c.index = static_cast<uint32_t>(i);
    out.push_back(c);
  }
  return true;
}

}  // namespace scorbot_hardware
