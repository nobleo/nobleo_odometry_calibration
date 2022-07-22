#pragma once
#include <ceres/jet.h>
#include <ros/message_forward.h>

#include <Eigen/Geometry>

namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(Point)
ROS_DECLARE_MESSAGE(Vector3)
ROS_DECLARE_MESSAGE(Quaternion)
ROS_DECLARE_MESSAGE(Pose)
ROS_DECLARE_MESSAGE(Transform)
}  // namespace geometry_msgs

namespace nobleo_gps_calibration
{
using Point = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
using Transform = Eigen::Isometry3d;

Point from_msg(const geometry_msgs::Point & point);
Point from_msg(const geometry_msgs::Vector3 & vector);
Quaternion from_msg(const geometry_msgs::Quaternion & q);
Transform from_msg(const geometry_msgs::Pose & pose);
Transform from_msg(const geometry_msgs::Transform & transform);
geometry_msgs::Point to_msg(const Point & point);

/**
 * @brief get_yaw
 *
 * The code below is a simplified version of getEulerRPY that only
 * returns the yaw. It is mostly useful in navigation where only yaw
 * matters. The implementation is copied from tf2.
 *
 * @return the computed yaw
 */
double get_yaw(const Eigen::Quaterniond & q);
double get_yaw(const Eigen::Matrix3d & matrix);

Quaternion create_quaternion(double roll, double pitch, double yaw);

Transform create_transform(
  double x, double y, double z, double roll = 0, double pitch = 0, double yaw = 0);

Eigen::Isometry2d to_2d(const Transform & pose);

template <typename T>
Eigen::Matrix<T, 2, 2> create_rotation_matrix_2d(T yaw_radians)
{
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);
  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  return rotation;
}

template <typename T>
Eigen::Transform<T, 2, Eigen::Isometry> create_transform_2d(T x, T y, T theta)
{
  Eigen::Transform<T, 2, Eigen::Isometry> transform;
  transform.makeAffine();  // needed due to bug in Eigen < 3.4
  transform.translation().x() = x;
  transform.translation().y() = y;
  transform.linear() = create_rotation_matrix_2d(theta);
  return transform;
}

/**
 * @brief Calculate linear and angular displacement based on robot movement
 */
std::pair<double, double> inverse_odometry(Eigen::Isometry2d odom_diff);

/**
 * @brief Calculate robot movement based on linear and angular displacement
 *
 * Inspired by:
 * https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/src/odometry.cpp
 */
template <typename T>
Eigen::Transform<T, 2, Eigen::Isometry> odometry(T linear, T angular)
{
  // create translation that is rotated by angular/2
  auto rot = create_rotation_matrix_2d(angular / T(2));
  auto translation = Eigen::Translation<T, 2>{linear, T(0)};

  Eigen::Transform<T, 2, Eigen::Isometry> transform;
  transform.makeAffine();
  transform.translation() = rot * translation.vector();
  transform.linear() = create_rotation_matrix_2d(angular);
  return transform;
}

}  // namespace nobleo_gps_calibration
