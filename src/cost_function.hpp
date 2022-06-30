#pragma once
#include <ceres/autodiff_cost_function.h>

#include "./math.hpp"
namespace nobleo_gps_calibration
{
struct CostFunctor
{
public:
  CostFunctor(const Transform & gps_diff, const Transform & odom_diff)
  : gps_diff_(to_2d(gps_diff)), odom_diff_(to_2d(odom_diff))
  {
  }

  template <typename T>
  bool operator()(
    const T * const gps_x, const T * const gps_y, const T * const gps_theta, T * residual) const
  {
    auto gps_pose = create_transform_2d(*gps_x, *gps_y, *gps_theta);

    auto odom_diff_in_gps = gps_pose.inverse() * odom_diff_.cast<T>() * gps_pose;

    auto err = gps_diff_.inverse().cast<T>() * odom_diff_in_gps;

    // Sanity check that the matrix is still affine. If the assert fails there is an uninitialized
    // transform somewhere.
    static_assert(int(err.Mode) == Eigen::Isometry);
    assert(err(2, 0) == T(0));
    assert(err(2, 1) == T(0));
    assert(err(2, 2) == T(1));

    Eigen::Rotation2D<T> rot;
    rot.fromRotationMatrix(err.linear());
    residual[0] = err.translation().x();
    residual[1] = err.translation().y();
    residual[2] = rot.angle();
    return true;
  }

  static ceres::CostFunction * create(const Transform & gps_diff, const Transform & odom_diff)
  {
    return new ceres::AutoDiffCostFunction<CostFunctor, 3, 1, 1, 1>(
      new CostFunctor(gps_diff, odom_diff));
  }

private:
  Eigen::Isometry2d gps_diff_;
  Eigen::Isometry2d odom_diff_;
};
}  // namespace nobleo_gps_calibration
