#include "hybrid_astar_planner/reeds_shepp.hpp"
#include <numbers>
#include <algorithm>
#include <cassert>

namespace hybrid_astar
{
namespace
{

constexpr double PI = std::numbers::pi;
constexpr double TWO_PI = 2.0 * std::numbers::pi;
constexpr double ZERO = 1e-10;

using RS = ReedsSheppSegmentType;

const RS reedsSheppPathType[18][5] = {
  {RS::RS_LEFT, RS::RS_RIGHT, RS::RS_LEFT, RS::RS_NOP, RS::RS_NOP},         // 0
  {RS::RS_RIGHT, RS::RS_LEFT, RS::RS_RIGHT, RS::RS_NOP, RS::RS_NOP},        // 1
  {RS::RS_LEFT, RS::RS_RIGHT, RS::RS_LEFT, RS::RS_RIGHT, RS::RS_NOP},       // 2
  {RS::RS_RIGHT, RS::RS_LEFT, RS::RS_RIGHT, RS::RS_LEFT, RS::RS_NOP},       // 3
  {RS::RS_LEFT, RS::RS_RIGHT, RS::RS_STRAIGHT, RS::RS_LEFT, RS::RS_NOP},    // 4
  {RS::RS_RIGHT, RS::RS_LEFT, RS::RS_STRAIGHT, RS::RS_RIGHT, RS::RS_NOP},   // 5
  {RS::RS_LEFT, RS::RS_STRAIGHT, RS::RS_RIGHT, RS::RS_LEFT, RS::RS_NOP},    // 6
  {RS::RS_RIGHT, RS::RS_STRAIGHT, RS::RS_LEFT, RS::RS_RIGHT, RS::RS_NOP},   // 7
  {RS::RS_LEFT, RS::RS_RIGHT, RS::RS_STRAIGHT, RS::RS_RIGHT, RS::RS_NOP},   // 8
  {RS::RS_RIGHT, RS::RS_LEFT, RS::RS_STRAIGHT, RS::RS_LEFT, RS::RS_NOP},    // 9
  {RS::RS_RIGHT, RS::RS_STRAIGHT, RS::RS_RIGHT, RS::RS_LEFT, RS::RS_NOP},   // 10
  {RS::RS_LEFT, RS::RS_STRAIGHT, RS::RS_LEFT, RS::RS_RIGHT, RS::RS_NOP},    // 11
  {RS::RS_LEFT, RS::RS_STRAIGHT, RS::RS_RIGHT, RS::RS_NOP, RS::RS_NOP},     // 12
  {RS::RS_RIGHT, RS::RS_STRAIGHT, RS::RS_LEFT, RS::RS_NOP, RS::RS_NOP},     // 13
  {RS::RS_LEFT, RS::RS_STRAIGHT, RS::RS_LEFT, RS::RS_NOP, RS::RS_NOP},      // 14
  {RS::RS_RIGHT, RS::RS_STRAIGHT, RS::RS_RIGHT, RS::RS_NOP, RS::RS_NOP},    // 15
  {RS::RS_LEFT, RS::RS_RIGHT, RS::RS_STRAIGHT, RS::RS_LEFT, RS::RS_RIGHT},  // 16
  {RS::RS_RIGHT, RS::RS_LEFT, RS::RS_STRAIGHT, RS::RS_RIGHT, RS::RS_LEFT}   // 17
};

inline double mod2pi(double x) noexcept
{
  double v = std::fmod(x, TWO_PI);
  if (v < -PI) {v += TWO_PI;} else if (v > PI) {v -= TWO_PI;}
  return v;
}

inline void polar(double x, double y, double & r, double & theta) noexcept
{
  r = std::hypot(x, y);
  theta = std::atan2(y, x);
}

inline void tauOmega(
  double u, double v, double xi, double eta, double phi,
  double & tau, double & omega) noexcept
{
  double delta = mod2pi(u - v);
  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.0;
  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  tau = (t2 < 0.0) ? mod2pi(t1 + PI) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

inline bool LpSpLp(double x, double y, double phi, double & t, double & u, double & v) noexcept
{
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
  if (t >= -ZERO) {
    v = mod2pi(phi - t);
    if (v >= -ZERO) {return true;}
  }
  return false;
}

inline bool LpSpRp(double x, double y, double phi, double & t, double & u, double & v) noexcept
{
  double t1, u1;
  polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
  double u1_sq = u1 * u1;
  if (u1_sq >= 4.0) {
    u = std::sqrt(u1_sq - 4.0);
    double theta = std::atan2(2.0, u);
    t = mod2pi(t1 + theta);
    v = mod2pi(t - phi);
    return  t >= -ZERO && v >= -ZERO;
  }
  return false;
}

void CSC(double x, double y, double phi, ReedsSheppPath & path) noexcept
{
  double t = 0.0, u = 0.0, v = 0.0, Lmin = path.total_length, L = 0.0;
  if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[14], t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[14], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[15], t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[15], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[12], t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[12], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[13], t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[13], -t, -u, -v);
  }
}

inline bool LpRmL(double x, double y, double phi, double & t, double & u, double & v) noexcept
{
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  double u1, theta;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.0) {
    u = -2.0 * std::asin(0.25 * u1);
    t = mod2pi(theta + 0.5 * u + PI);
    v = mod2pi(phi - t + u);
    return  t >= -ZERO && u <= ZERO;
  }
  return false;
}

void CCC(double x, double y, double phi, ReedsSheppPath & path) noexcept
{
  double t = 0.0, u = 0.0, v = 0.0, Lmin = path.total_length, L = 0.0;
  if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[0], t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[0], -t, -u, -v);
    Lmin = L;
  }
  if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[1], t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[1], -t, -u, -v);
    Lmin = L;
  }

  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[0], v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[0], -v, -u, -t);
    Lmin = L;
  }
  if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[1], v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[1], -v, -u, -t);
  }
}

inline bool LpRupLumRm(double x, double y, double phi, double & t, double & u, double & v) noexcept
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = 0.25 * (2.0 + std::hypot(xi, eta));
  if (rho <= 1.0) {
    u = std::acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    return  t >= -ZERO && v <= ZERO;
  }
  return false;
}

inline bool LpRumLumRp(double x, double y, double phi, double & t, double & u, double & v) noexcept
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho >= 0.0 && rho <= 1.0) {
    u = -std::acos(rho);
    if (u >= -0.5 * PI) {
      tauOmega(u, u, xi, eta, phi, t, v);
      return  t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}

void CCCC(double x, double y, double phi, ReedsSheppPath & path) noexcept
{
  double t = 0.0, u = 0.0, v = 0.0, Lmin = path.total_length, L = 0.0;
  if (LpRupLumRm(x, y, phi, t, u,
        v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))
  {
    path = ReedsSheppPath(reedsSheppPathType[2], t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, y, -phi, t, u,
        v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))
  {
    path = ReedsSheppPath(reedsSheppPathType[2], -t, -u, u, -v);
    Lmin = L;
  }
  if (LpRupLumRm(x, -y, -phi, t, u,
        v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))
  {
    path = ReedsSheppPath(reedsSheppPathType[3], t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, -y, phi, t, u,
        v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))
  {
    path = ReedsSheppPath(reedsSheppPathType[3], -t, -u, u, -v);
    Lmin = L;
  }

  if (LpRumLumRp(x, y, phi, t, u,
        v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))
  {
    path = ReedsSheppPath(reedsSheppPathType[2], t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, y, -phi, t, u,
        v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))
  {
    path = ReedsSheppPath(reedsSheppPathType[2], -t, -u, -u, -v);
    Lmin = L;
  }
  if (LpRumLumRp(x, -y, -phi, t, u,
        v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))
  {
    path = ReedsSheppPath(reedsSheppPathType[3], t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, -y, phi, t, u,
        v) && Lmin > (L = std::abs(t) + 2.0 * std::abs(u) + std::abs(v)))
  {
    path = ReedsSheppPath(reedsSheppPathType[3], -t, -u, -u, -v);
  }
}

inline bool LpRmSmLm(double x, double y, double phi, double & t, double & u, double & v) noexcept
{
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  double rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.0) {
    double r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = mod2pi(theta + std::atan2(r, -2.0));
    v = mod2pi(phi - 0.5 * PI - t);
    return  t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}

inline bool LpRmSmRm(double x, double y, double phi, double & t, double & u, double & v) noexcept
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho, theta;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.0) {
    t = theta;
    u = 2.0 - rho;
    v = mod2pi(t + 0.5 * PI - phi);
    return  t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}

void CCSC(double x, double y, double phi, ReedsSheppPath & path) noexcept
{
  double t = 0.0, u = 0.0, v = 0.0, Lmin = path.total_length - 0.5 * PI, L = 0.0;
  if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[4], t, -0.5 * PI, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[4], -t, 0.5 * PI, -u, -v);
    Lmin = L;
  }
  if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[5], t, -0.5 * PI, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[5], -t, 0.5 * PI, -u, -v);
    Lmin = L;
  }

  if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[8], t, -0.5 * PI, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[8], -t, 0.5 * PI, -u, -v);
    Lmin = L;
  }
  if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[9], t, -0.5 * PI, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[9], -t, 0.5 * PI, -u, -v);
    Lmin = L;
  }

  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[6], v, u, -0.5 * PI, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[6], -v, -u, 0.5 * PI, -t);
    Lmin = L;
  }
  if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[7], v, u, -0.5 * PI, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[7], -v, -u, 0.5 * PI, -t);
    Lmin = L;
  }

  if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[10], v, u, -0.5 * PI, t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[10], -v, -u, 0.5 * PI, -t);
    Lmin = L;
  }
  if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[11], v, u, -0.5 * PI, t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[11], -v, -u, 0.5 * PI, -t);
  }
}

inline bool LpRmSLmRp(double x, double y, double phi, double & t, double & u, double & v) noexcept
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.0) {
    u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= ZERO) {
      t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = mod2pi(t - phi);
      return  t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}

void CCSCC(double x, double y, double phi, ReedsSheppPath & path) noexcept
{
  double t = 0.0, u = 0.0, v = 0.0, Lmin = path.total_length - PI, L = 0.0;
  if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[16], t, -0.5 * PI, u, -0.5 * PI, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[16], -t, 0.5 * PI, -u, 0.5 * PI, -v);
    Lmin = L;
  }
  if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[17], t, -0.5 * PI, u, -0.5 * PI, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppPath(reedsSheppPathType[17], -t, 0.5 * PI, -u, 0.5 * PI, -v);
  }
}

ReedsSheppPath get_path(double x, double y, double phi) noexcept
{
  ReedsSheppPath path;
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);
  return path;
}

} // namespace

ReedsSheppPath solve_reeds_shepp(
  const Pose2D & start,
  const Pose2D & goal,
  double turning_radius) noexcept
{

  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double c = std::cos(start.theta);
  const double s = std::sin(start.theta);

  // Transform goal to start body frame and normalize by radius
  const double x = (c * dx + s * dy) / turning_radius;
  const double y = (-s * dx + c * dy) / turning_radius;
  const double phi = goal.theta - start.theta;

  return get_path(x, y, phi);
}

double reeds_shepp_distance(
  const Pose2D & start,
  const Pose2D & goal,
  double turning_radius) noexcept
{

  const auto path = solve_reeds_shepp(start, goal, turning_radius);
  return path.total_length * turning_radius;
}

std::vector<Pose2D> sample_reeds_shepp(
  const Pose2D & start,
  const ReedsSheppPath & path,
  double turning_radius,
  double step_size)
{

  std::vector<Pose2D> points;
  if (std::isinf(path.total_length) || path.total_length < 1e-6) {
    points.push_back(start);
    return points;
  }

  const double total_dist = path.total_length * turning_radius;
  const int num_steps = std::max(1, static_cast<int>(std::ceil(total_dist / step_size)));
  points.reserve(num_steps + 1);

  for (int step = 0; step <= num_steps; ++step) {
    const double ratio = static_cast<double>(step) / static_cast<double>(num_steps);
    double seg = ratio * path.total_length;
    double sx = 0.0;
    double sy = 0.0;
    double phi = start.theta;

    for (int i = 0; i < 5 && seg > 0.0; ++i) {
      double v = 0.0;
      if (path.length[i] < 0.0) {
        v = std::max(-seg, path.length[i]);
        seg += v;
      } else {
        v = std::min(seg, path.length[i]);
        seg -= v;
      }
      switch (path.type[i]) {
        case RS::RS_LEFT:
          sx += std::sin(phi + v) - std::sin(phi);
          sy += -std::cos(phi + v) + std::cos(phi);
          phi += v;
          break;
        case RS::RS_RIGHT:
          sx += -std::sin(phi - v) + std::sin(phi);
          sy += std::cos(phi - v) - std::cos(phi);
          phi -= v;
          break;
        case RS::RS_STRAIGHT:
          sx += v * std::cos(phi);
          sy += v * std::sin(phi);
          break;
        case RS::RS_NOP:
          break;
      }
    }

    Pose2D pt;
    pt.x = sx * turning_radius + start.x;
    pt.y = sy * turning_radius + start.y;
    pt.theta = normalize_angle(phi);
    points.push_back(pt);
  }

  return points;
}

} // namespace hybrid_astar
