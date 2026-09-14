// Unit tests for the ROS-free closed-form solver. Uses the ER-4pc geometry from
// scorbot_description/config/er_4pc_kinematics.yaml.

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "scorbot_kinematics/analytic_ik.hpp"

using namespace scorbot_kinematics;

namespace
{

Geometry er4pc()
{
  Geometry g;
  g.z0 = 0.14833;
  g.a1 = 0.029;
  g.h1 = 0.19766;
  g.L1 = 0.22;
  g.L2 = 0.22;
  g.dt = 0.0;
  g.R_tool = Mat3::rotY(kPi / 2.0);  // tool0 rpy (0, pi/2, 0): z out along the flange x
  g.sign = {1, 1, 1, 1, 1};
  const double d = kPi / 180.0;
  g.lower = {-155 * d, -35 * d, -130 * d, -130 * d, -570 * d};
  g.upper = {155 * d, 130 * d, 130 * d, 130 * d, 570 * d};
  return g;
}

Geometry erv()
{
  Geometry g = er4pc();
  g.dt = 0.12;
  return g;
}

bool close(const Vec3& a, const Vec3& b, double tol = 1e-6) { return norm(a - b) < tol; }

}  // namespace

TEST(AnalyticIk, ForwardKinematicsAtZero)
{
  const Geometry g = er4pc();
  Vec3 p;
  Mat3 R;
  forwardKinematics(g, {0, 0, 0, 0, 0}, p, R);
  EXPECT_NEAR(p.x, 0.469, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  EXPECT_NEAR(p.z, 0.34599, 1e-9);
  // tool z axis points along +x of the base
  EXPECT_NEAR(R.m[0][2], 1.0, 1e-9);
  EXPECT_NEAR(R.m[1][2], 0.0, 1e-9);
  EXPECT_NEAR(R.m[2][2], 0.0, 1e-9);
}

TEST(AnalyticIk, PositiveShoulderLifts)
{
  const Geometry g = er4pc();
  Vec3 p0, p1;
  Mat3 R;
  forwardKinematics(g, {0, 0, 0, 0, 0}, p0, R);
  forwardKinematics(g, {0, 0.3, 0, 0, 0}, p1, R);
  EXPECT_GT(p1.z, p0.z);
  EXPECT_LT(p1.x, p0.x);
}

TEST(AnalyticIk, ExactRoundTripOnReachablePoses)
{
  for (const Geometry& g : {er4pc(), erv()})
  {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    int tested = 0;
    for (int n = 0; n < 2000; ++n)
    {
      std::array<double, kNumJoints> q{};
      for (int i = 0; i < kNumJoints; ++i)
        q[i] = g.lower[i] + u(rng) * (g.upper[i] - g.lower[i]);
      // keep the roll inside one turn so the nearest-turn choice is unambiguous
      q[4] = wrapPi(q[4]);

      Vec3 p;
      Mat3 R;
      forwardKinematics(g, q, p, R);

      // Seed near the truth: the solver must return the same branch.
      std::array<double, kNumJoints> seed = q;
      for (auto& s : seed)
        s += 0.05 * (u(rng) - 0.5);

      const auto sols = inverseKinematics(g, p, R, seed);
      ASSERT_FALSE(sols.empty()) << "pose from a valid state must be solvable";
      const auto& best = sols.front();
      EXPECT_LT(best.orientation_error, 1e-6);

      // With the elbow nearly straight the two elbow branches give the same tool pose
      // and the perturbed seed may legitimately pick the mirror; elsewhere the exact
      // joint values must come back.
      if (std::fabs(q[2]) > 0.1)
      {
        for (int i = 0; i < kNumJoints; ++i)
          EXPECT_NEAR(best.q[i], q[i], 1e-6) << "joint " << i;
      }

      Vec3 p2;
      Mat3 R2;
      forwardKinematics(g, best.q, p2, R2);
      EXPECT_TRUE(close(p, p2, 1e-9));
      EXPECT_LT(rotationDistance(R, R2), 1e-6);
      ++tested;
    }
    EXPECT_EQ(tested, 2000);
  }
}

TEST(AnalyticIk, UnreachableYawIsProjectedNotRejected)
{
  const Geometry g = er4pc();
  std::array<double, kNumJoints> q{0.2, 0.4, -0.8, 0.4, 0.1};
  Vec3 p;
  Mat3 R;
  forwardKinematics(g, q, p, R);

  // Twist the request about the world z axis by 30 degrees without moving the point:
  // impossible for a 5-DOF arm. Position must still be met exactly.
  const Mat3 R_twisted = Mat3::rotZ(0.5) * R;
  const auto sols = inverseKinematics(g, p, R_twisted, q);
  ASSERT_FALSE(sols.empty());
  Vec3 p2;
  Mat3 R2;
  forwardKinematics(g, sols.front().q, p2, R2);
  EXPECT_TRUE(close(p, p2, 1e-9));
  EXPECT_GT(sols.front().orientation_error, 0.1);
  EXPECT_LT(sols.front().orientation_error, 0.5 + 1e-6);
  // Base angle is dictated by the point, so it must not change.
  EXPECT_NEAR(sols.front().q[0], q[0], 1e-9);
}

TEST(AnalyticIk, PitchAndRollRequestsAreHonored)
{
  const Geometry g = er4pc();
  std::array<double, kNumJoints> q{0.0, 0.3, -0.6, 0.3, 0.0};
  Vec3 p;
  Mat3 R;
  forwardKinematics(g, q, p, R);

  // Rotate the tool 0.2 rad about the base y axis (a pure pitch change in the arm plane)
  // and 0.4 rad about its own axis: both are reachable, so the error should vanish.
  const Mat3 R_req = Mat3::rotY(-0.2) * R * Mat3::rotZ(0.4);
  const auto sols = inverseKinematics(g, p, R_req, q);
  ASSERT_FALSE(sols.empty());
  EXPECT_LT(sols.front().orientation_error, 1e-6);
  EXPECT_NEAR(sols.front().q[1] + sols.front().q[2] + sols.front().q[3], 0.2, 1e-6);
  EXPECT_NEAR(sols.front().q[4], 0.4, 1e-6);
}

TEST(AnalyticIk, OutOfReachReturnsNothing)
{
  const Geometry g = er4pc();
  const auto sols = inverseKinematics(g, {2.0, 0.0, 0.3}, Mat3::identity(), {0, 0, 0, 0, 0});
  EXPECT_TRUE(sols.empty());
}

TEST(AnalyticIk, SeedSelectsElbowBranchAndBaseBranch)
{
  const Geometry g = er4pc();
  std::array<double, kNumJoints> q_up{0.0, 0.6, -1.0, 0.4, 0.0};
  Vec3 p;
  Mat3 R;
  forwardKinematics(g, q_up, p, R);

  // Elbow-down twin of the same tool pose, if within limits, is further from q_up.
  const auto sols = inverseKinematics(g, p, R, q_up);
  ASSERT_FALSE(sols.empty());
  EXPECT_NEAR(sols.front().q[2], -1.0, 1e-6);
  for (size_t i = 1; i < sols.size(); ++i)
    EXPECT_GE(sols[i].seed_distance, sols.front().seed_distance);
}

TEST(AnalyticIk, JointLimitsAreEnforced)
{
  Geometry g = er4pc();
  g.upper[1] = 0.1;  // shoulder barely allowed to lift
  std::array<double, kNumJoints> q{0.0, 0.8, -0.5, 0.0, 0.0};
  Vec3 p;
  Mat3 R;
  forwardKinematics(g, q, p, R);
  for (const auto& s : inverseKinematics(g, p, R, q))
    for (int i = 0; i < kNumJoints; ++i)
    {
      EXPECT_GE(s.q[i], g.lower[i] - 1e-6);
      EXPECT_LE(s.q[i], g.upper[i] + 1e-6);
    }
}

TEST(AnalyticIk, AxisSignsAreRespected)
{
  Geometry g = er4pc();
  g.sign = {1, -1, -1, -1, -1};  // a URDF with +y pitch axes and -x roll
  std::array<double, kNumJoints> q{0.3, -0.4, 0.7, -0.2, 0.5};
  Vec3 p;
  Mat3 R;
  forwardKinematics(g, q, p, R);
  const auto sols = inverseKinematics(g, p, R, q);
  ASSERT_FALSE(sols.empty());
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(sols.front().q[i], q[i], 1e-6);
}

TEST(AnalyticIk, RollPicksNearestTurn)
{
  const Geometry g = er4pc();
  std::array<double, kNumJoints> q{0.0, 0.2, -0.4, 0.2, 3.0};
  Vec3 p;
  Mat3 R;
  forwardKinematics(g, q, p, R);
  // Seed on the other side of the wrap: the solver should choose 3.0 - 2pi ~ -3.28.
  std::array<double, kNumJoints> seed{0.0, 0.2, -0.4, 0.2, -3.2};
  const auto sols = inverseKinematics(g, p, R, seed);
  ASSERT_FALSE(sols.empty());
  EXPECT_NEAR(sols.front().q[4], 3.0 - 2.0 * kPi, 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
