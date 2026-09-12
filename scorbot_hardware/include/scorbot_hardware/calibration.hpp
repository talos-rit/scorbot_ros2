// Calibration YAML (scorbot_description/config/<robot>_calibration.yaml) to the
// JointCalibration messages pushed to the controller at configure time.

#pragma once

#include <map>
#include <string>
#include <vector>

#include "scorbot_protocol/messages.hpp"

namespace scorbot_hardware
{

struct CalibrationFile
{
  std::string robot_type;
  /// Keyed by the unprefixed joint name as written in the file.
  std::map<std::string, scorbot_protocol::JointCalibration> joints;
};

/// Parse `path`. Every joint needs every field (no silent defaults: a missing number
/// on a real robot is a bug). Returns false with `error` set on any problem.
bool loadCalibrationFile(const std::string& path, CalibrationFile& out, std::string& error);

/// Same, from YAML text (for tests and embedded configs).
bool parseCalibration(const std::string& yaml_text, CalibrationFile& out, std::string& error);

/// Arrange the file's joints in `joint_names` order (ros2_control order, possibly
/// prefixed: `prefix` is removed before the lookup), setting `index`. Returns false if
/// a joint is missing from the file.
bool orderCalibration(const CalibrationFile& file, const std::vector<std::string>& joint_names,
                      const std::string& prefix, std::vector<scorbot_protocol::JointCalibration>& out,
                      std::string& error);

/// `name` without `prefix` if it starts with it, else unchanged.
std::string stripPrefix(const std::string& name, const std::string& prefix);

}  // namespace scorbot_hardware
