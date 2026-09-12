#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>

#include "scorbot_hardware/calibration.hpp"

using namespace scorbot_hardware;

namespace
{
const char* kGood = R"(
robot_type: er_4pc
joints:
  base_joint:
    counts_per_motor_rev: 80
    gear_ratio: 635.5
    home_offset_rad: 0.1
    home_direction: -1
    soft_limit_min_rad: -2.7
    soft_limit_max_rad: 2.7
    max_velocity_rad_s: 0.35
    max_accel_rad_s2: 1.0
    current_limit_a: 2.0
    invert: false
    encoder_invert: true
  shoulder_joint:
    counts_per_motor_rev: 80
    gear_ratio: 457.56
    home_offset_rad: 0.0
    home_direction: 1
    soft_limit_min_rad: -0.61
    soft_limit_max_rad: 2.27
    max_velocity_rad_s: 0.45
    max_accel_rad_s2: 1.0
    current_limit_a: 2.0
    invert: true
    encoder_invert: false
)";
}

TEST(Calibration, ParsesEveryField)
{
  CalibrationFile f;
  std::string err;
  ASSERT_TRUE(parseCalibration(kGood, f, err)) << err;
  EXPECT_EQ(f.robot_type, "er_4pc");
  ASSERT_EQ(f.joints.size(), 2u);
  const auto& b = f.joints.at("base_joint");
  EXPECT_EQ(b.counts_per_motor_rev, 80);
  EXPECT_FLOAT_EQ(b.gear_ratio, 635.5f);
  EXPECT_FLOAT_EQ(b.home_offset_rad, 0.1f);
  EXPECT_EQ(b.home_direction, -1);
  EXPECT_FLOAT_EQ(b.soft_limit_max_rad, 2.7f);
  EXPECT_FALSE(b.invert);
  EXPECT_TRUE(b.encoder_invert);
  EXPECT_TRUE(f.joints.at("shoulder_joint").invert);
  EXPECT_FALSE(f.joints.at("shoulder_joint").encoder_invert);
}

TEST(Calibration, MissingFieldIsAnError)
{
  std::string text = kGood;
  text.erase(text.find("    current_limit_a: 2.0\n"), sizeof("    current_limit_a: 2.0\n") - 1);
  CalibrationFile f;
  std::string err;
  EXPECT_FALSE(parseCalibration(text, f, err));
  EXPECT_NE(err.find("current_limit_a"), std::string::npos) << err;
}

TEST(Calibration, ValidatesRanges)
{
  CalibrationFile f;
  std::string err;
  std::string text = kGood;
  text.replace(text.find("home_direction: -1"), sizeof("home_direction: -1") - 1, "home_direction: 2");
  EXPECT_FALSE(parseCalibration(text, f, err));
  EXPECT_NE(err.find("home_direction"), std::string::npos) << err;

  text = kGood;
  text.replace(text.find("soft_limit_max_rad: 2.7"), sizeof("soft_limit_max_rad: 2.7") - 1, "soft_limit_max_rad: -3.0");
  EXPECT_FALSE(parseCalibration(text, f, err));
  EXPECT_NE(err.find("soft_limit"), std::string::npos) << err;

  EXPECT_FALSE(parseCalibration("robot_type: er_v\n", f, err));
  EXPECT_FALSE(parseCalibration(": [", f, err));
}

TEST(Calibration, OrdersByPrefixedJointNames)
{
  CalibrationFile f;
  std::string err;
  ASSERT_TRUE(parseCalibration(kGood, f, err)) << err;
  std::vector<scorbot_protocol::JointCalibration> ordered;
  ASSERT_TRUE(orderCalibration(f, {"bluey_shoulder_joint", "bluey_base_joint"}, "bluey_", ordered, err)) << err;
  ASSERT_EQ(ordered.size(), 2u);
  EXPECT_EQ(ordered[0].index, 0u);
  EXPECT_FLOAT_EQ(ordered[0].gear_ratio, 457.56f);
  EXPECT_EQ(ordered[1].index, 1u);
  EXPECT_FLOAT_EQ(ordered[1].gear_ratio, 635.5f);

  EXPECT_FALSE(orderCalibration(f, {"base_joint", "elbow_joint"}, "", ordered, err));
  EXPECT_NE(err.find("elbow_joint"), std::string::npos) << err;
  EXPECT_EQ(stripPrefix("x_a", "x_"), "a");
  EXPECT_EQ(stripPrefix("a", "x_"), "a");
  EXPECT_EQ(stripPrefix("a", ""), "a");
}

TEST(Calibration, LoadsFromFile)
{
  const std::string path = std::string(::getenv("TMPDIR") ? ::getenv("TMPDIR") : "/tmp") + "/scorbot_hw_test_cal.yaml";
  {
    std::ofstream out(path);
    out << kGood;
  }
  CalibrationFile f;
  std::string err;
  ASSERT_TRUE(loadCalibrationFile(path, f, err)) << err;
  EXPECT_EQ(f.joints.size(), 2u);
  EXPECT_FALSE(loadCalibrationFile(path + ".missing", f, err));
  EXPECT_NE(err.find("cannot read"), std::string::npos) << err;
}

// The shipped calibration files must load and cover the five v1 joints. CMake points
// SCORBOT_CALIBRATION_DIR at scorbot_description's config directory.
TEST(Calibration, ShippedFilesLoad)
{
  const char* dir = ::getenv("SCORBOT_CALIBRATION_DIR");
  if (!dir)
    GTEST_SKIP() << "SCORBOT_CALIBRATION_DIR not set";
  for (const char* robot : {"er_4pc", "er_v"})
  {
    CalibrationFile f;
    std::string err;
    const std::string path = std::string(dir) + "/" + robot + "_calibration.yaml";
    ASSERT_TRUE(loadCalibrationFile(path, f, err)) << err;
    EXPECT_EQ(f.robot_type, robot);
    std::vector<scorbot_protocol::JointCalibration> ordered;
    EXPECT_TRUE(orderCalibration(
        f, {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"}, "", ordered, err))
        << err;
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
