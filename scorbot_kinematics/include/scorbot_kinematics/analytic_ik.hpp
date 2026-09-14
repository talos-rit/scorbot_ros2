// Closed-form kinematics for the 5-DOF Scorbot (ER-V, ER-4pc) arm.
//
// ROS-free on purpose: this header depends on nothing but the standard library so it
// can be unit tested anywhere and reused outside MoveIt (the camera director will
// need "point the tool at X" solves too). The MoveIt plugin in
// scorbot_kinematics_plugin.cpp is a thin adapter around it.
//
// Kinematic model (matches scorbot_description; every link frame is aligned with
// base_link at the zero pose, arm pointing along +x):
//
//   base_link --[q1 about z]--> turret at (0, 0, z0)
//             --[q2 about -y]--> shoulder axis at (a1, 0, z0 + h1)
//             --[q3 about -y]--> elbow axis, L1 along the upper arm
//             --[q4 about -y]--> wrist pitch axis, L2 along the forearm
//             --[q5 about x]---> wrist roll axis, intersecting the pitch axis
//             --fixed---------> tool0 at dt along the flange x axis, rotated by R_tool
//
// Canonical angles theta_i are "positive lifts" for 2..4, counter-clockwise from above
// for 1, right-handed about the tool axis for 5. Joint values are q_i = sign_i * theta_i,
// where sign_i comes from the URDF axis direction.
//
// The arm has five joints, so the tool orientation has only two free parameters once
// the position is fixed: elevation (pitch) and roll. The requested orientation is
// projected onto what is reachable: yaw is dictated by the base angle, the pitch is the
// elevation of the requested tool direction inside the arm's vertical plane, and the
// roll is the closest rotation about the tool axis. The position is always met exactly
// when it is within reach.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace scorbot_kinematics
{

constexpr int kNumJoints = 5;
constexpr double kPi = 3.14159265358979323846;

struct Vec3
{
  double x{0.0}, y{0.0}, z{0.0};
};

/// Minimal 3x3 rotation matrix, row-major: m[row][col].
struct Mat3
{
  double m[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  static Mat3 identity() { return Mat3{}; }

  static Mat3 rotX(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    Mat3 r;
    r.m[0][0] = 1; r.m[0][1] = 0; r.m[0][2] = 0;
    r.m[1][0] = 0; r.m[1][1] = c; r.m[1][2] = -s;
    r.m[2][0] = 0; r.m[2][1] = s; r.m[2][2] = c;
    return r;
  }
  static Mat3 rotY(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    Mat3 r;
    r.m[0][0] = c;  r.m[0][1] = 0; r.m[0][2] = s;
    r.m[1][0] = 0;  r.m[1][1] = 1; r.m[1][2] = 0;
    r.m[2][0] = -s; r.m[2][1] = 0; r.m[2][2] = c;
    return r;
  }
  static Mat3 rotZ(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    Mat3 r;
    r.m[0][0] = c; r.m[0][1] = -s; r.m[0][2] = 0;
    r.m[1][0] = s; r.m[1][1] = c;  r.m[1][2] = 0;
    r.m[2][0] = 0; r.m[2][1] = 0;  r.m[2][2] = 1;
    return r;
  }

  Mat3 operator*(const Mat3& o) const
  {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
    return r;
  }
  Vec3 operator*(const Vec3& v) const
  {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }
  Mat3 transpose() const
  {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        r.m[i][j] = m[j][i];
    return r;
  }
  Vec3 column(int j) const { return {m[0][j], m[1][j], m[2][j]}; }

  /// Rotation angle of this matrix (0 for identity).
  double angle() const
  {
    const double t = m[0][0] + m[1][1] + m[2][2];
    return std::acos(std::clamp((t - 1.0) / 2.0, -1.0, 1.0));
  }
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(double s, const Vec3& v) { return {s * v.x, s * v.y, s * v.z}; }
inline double norm(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

/// Wrap an angle into (-pi, pi].
inline double wrapPi(double a)
{
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0)
    a += 2.0 * kPi;
  return a - kPi;
}

/// Angle between two rotations.
inline double rotationDistance(const Mat3& a, const Mat3& b) { return (a.transpose() * b).angle(); }

/// Arm geometry, all lengths in meters, all derived from the URDF at start-up.
struct Geometry
{
  double z0{0.0};  ///< base_link to base joint, along z
  double a1{0.0};  ///< base axis to shoulder axis, horizontal
  double h1{0.0};  ///< base joint to shoulder axis, vertical
  double L1{0.0};  ///< upper arm
  double L2{0.0};  ///< forearm
  double dt{0.0};  ///< wrist center to tool0 along the flange x axis
  Mat3 R_tool;     ///< flange -> tool0 rotation
  std::array<double, kNumJoints> sign{{1, 1, 1, 1, 1}};  ///< URDF axis sign per joint
  std::array<double, kNumJoints> lower{{-kPi, -kPi, -kPi, -kPi, -kPi}};
  std::array<double, kNumJoints> upper{{kPi, kPi, kPi, kPi, kPi}};
};

struct Solution
{
  std::array<double, kNumJoints> q{};
  double orientation_error{0.0};  ///< angle between requested and achieved tool orientation
  double seed_distance{0.0};      ///< sum of |q - seed|
};

/// Tool0 pose in base_link coordinates for joint values q.
inline void forwardKinematics(const Geometry& g, const std::array<double, kNumJoints>& q,
                              Vec3& p_tool, Mat3& R_tool)
{
  std::array<double, kNumJoints> t{};
  for (int i = 0; i < kNumJoints; ++i)
    t[i] = g.sign[i] * q[i];
  const double phi = t[1] + t[2] + t[3];

  const Mat3 Rz = Mat3::rotZ(t[0]);
  const Vec3 wrist_in_plane{g.a1 + g.L1 * std::cos(t[1]) + g.L2 * std::cos(t[1] + t[2]), 0.0,
                            g.z0 + g.h1 + g.L1 * std::sin(t[1]) + g.L2 * std::sin(t[1] + t[2])};
  const Vec3 p_w = Rz * wrist_in_plane;
  const Mat3 R_f = Rz * Mat3::rotY(-phi) * Mat3::rotX(t[4]);
  p_tool = p_w + R_f * Vec3{g.dt, 0.0, 0.0};
  R_tool = R_f * g.R_tool;
}

namespace detail
{
inline bool withinLimits(const Geometry& g, int i, double q, double tol = 1e-6)
{
  return q >= g.lower[i] - tol && q <= g.upper[i] + tol;
}

/// Pick q + 2*pi*k closest to `seed` that respects the joint limits; false if none does.
inline bool nearestTurn(const Geometry& g, int i, double q, double seed, double& out)
{
  double best = std::numeric_limits<double>::quiet_NaN();
  double best_dist = std::numeric_limits<double>::infinity();
  for (int k = -3; k <= 3; ++k)
  {
    const double cand = q + 2.0 * kPi * k;
    if (!withinLimits(g, i, cand))
      continue;
    const double d = std::fabs(cand - seed);
    if (d < best_dist)
    {
      best_dist = d;
      best = cand;
    }
  }
  if (std::isnan(best))
    return false;
  out = best;
  return true;
}
}  // namespace detail

/// All limit-respecting solutions for the requested tool0 pose (base_link frame).
/// The position is met exactly by every returned solution; the orientation is the
/// closest reachable one (see file header). Empty if the position is out of reach.
inline std::vector<Solution> inverseKinematics(const Geometry& g, const Vec3& p_tool,
                                               const Mat3& R_tool_target,
                                               const std::array<double, kNumJoints>& seed)
{
  std::vector<Solution> solutions;

  // Requested flange orientation and the tool direction it implies.
  const Mat3 R_f_target = R_tool_target * g.R_tool.transpose();
  const Vec3 u_target = R_f_target.column(0);

  // Canonical seed angles, used to break ties and choose branches.
  std::array<double, kNumJoints> seed_t{};
  for (int i = 0; i < kNumJoints; ++i)
    seed_t[i] = g.sign[i] * seed[i];

  // Base angle: the tool point lies in the arm's vertical plane. Two branches
  // (facing the point, or reaching back over the base).
  const double radial = std::hypot(p_tool.x, p_tool.y);
  const double theta1_facing = radial < 1e-9 ? seed_t[0] : std::atan2(p_tool.y, p_tool.x);

  for (const double theta1 : {theta1_facing, theta1_facing + kPi})
  {
    const double c1 = std::cos(theta1), s1 = std::sin(theta1);

    // Tool elevation inside the plane: projection of the requested direction.
    const double u_radial = u_target.x * c1 + u_target.y * s1;
    const double phi = std::atan2(u_target.z, u_radial);

    // Roll: closest rotation about the tool axis to what remains of the request.
    const Mat3 R_pitched = Mat3::rotZ(theta1) * Mat3::rotY(-phi);
    const Mat3 R_rem = R_pitched.transpose() * R_f_target;
    const double theta5 = std::atan2(R_rem.m[2][1] - R_rem.m[1][2], R_rem.m[1][1] + R_rem.m[2][2]);
    const Mat3 R_f_achieved = R_pitched * Mat3::rotX(theta5);
    const double orientation_error = rotationDistance(R_f_achieved, R_f_target);

    // Wrist center, then the planar two-link problem in the arm's plane.
    const Vec3 u_achieved{std::cos(phi) * c1, std::cos(phi) * s1, std::sin(phi)};
    const Vec3 p_w = p_tool - g.dt * u_achieved;
    const double r = p_w.x * c1 + p_w.y * s1 - g.a1;
    const double z = p_w.z - g.z0 - g.h1;

    double D = (r * r + z * z - g.L1 * g.L1 - g.L2 * g.L2) / (2.0 * g.L1 * g.L2);
    if (D > 1.0 + 1e-9 || D < -1.0 - 1e-9)
      continue;  // out of reach in this branch
    D = std::clamp(D, -1.0, 1.0);

    for (const double theta3 : {std::acos(D), -std::acos(D)})
    {
      const double theta2 = std::atan2(z, r) - std::atan2(g.L2 * std::sin(theta3), g.L1 + g.L2 * std::cos(theta3));
      const double theta4 = phi - theta2 - theta3;

      Solution sol;
      sol.orientation_error = orientation_error;
      const std::array<double, kNumJoints> theta{theta1, theta2, theta3, theta4, theta5};
      bool valid = true;
      for (int i = 0; i < kNumJoints && valid; ++i)
      {
        const double q = wrapPi(g.sign[i] * theta[i]);
        valid = detail::nearestTurn(g, i, q, seed[i], sol.q[i]);
      }
      if (!valid)
        continue;

      sol.seed_distance = 0.0;
      for (int i = 0; i < kNumJoints; ++i)
        sol.seed_distance += std::fabs(sol.q[i] - seed[i]);
      solutions.push_back(sol);
    }
  }

  std::sort(solutions.begin(), solutions.end(),
            [](const Solution& a, const Solution& b) { return a.seed_distance < b.seed_distance; });
  return solutions;
}

}  // namespace scorbot_kinematics
