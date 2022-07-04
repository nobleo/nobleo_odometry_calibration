#include "./math.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>

#include <utility>
namespace nobleo_gps_calibration
{
Point from_msg(const geometry_msgs::Point & point)
{
  Point result;
  result[0] = point.x;
  result[1] = point.y;
  result[2] = point.z;
  return result;
}

Point from_msg(const geometry_msgs::Vector3 & vector)
{
  Point result;
  result[0] = vector.x;
  result[1] = vector.y;
  result[2] = vector.z;
  return result;
}

Quaternion from_msg(const geometry_msgs::Quaternion & q) { return {q.w, q.x, q.y, q.z}; }

Transform from_msg(const geometry_msgs::Pose & pose)
{
  Transform result;
  result.makeAffine();  // needed due to bug in Eigen < 3.4
  result.translation() = from_msg(pose.position);
  result.linear() = from_msg(pose.orientation).toRotationMatrix();
  return result;
}

Transform from_msg(const geometry_msgs::Transform & transform)
{
  Transform result;
  result.makeAffine();  // needed due to bug in Eigen < 3.4
  result.translation() = from_msg(transform.translation);
  result.linear() = from_msg(transform.rotation).toRotationMatrix();
  return result;
}

double get_yaw(const Eigen::Quaterniond & q)
{
  double yaw;

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();
  sqw = q.w() * q.w();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (q.x() * q.z() - q.w() * q.y()) /
                (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

  if (sarg <= -0.99999) {
    yaw = -2 * atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    yaw = 2 * atan2(q.y(), q.x());
  } else {
    yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), sqw + sqx - sqy - sqz);
  }
  return yaw;
}

double get_yaw(const Eigen::Matrix3d & matrix)
{
  Eigen::Quaterniond q{matrix};
  return get_yaw(q);
}

Quaternion create_quaternion(double roll, double pitch, double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

Transform create_transform(double x, double y, double z, double roll, double pitch, double yaw)
{
  Transform result;
  result.makeAffine();  // needed due to bug in Eigen < 3.4
  result.linear() = create_quaternion(roll, pitch, yaw).toRotationMatrix();
  result.translation() = Eigen::Vector3d{x, y, z};
  return result;
}

Eigen::Isometry2d to_2d(const Transform & pose)
{
  Eigen::Isometry2d result;
  result.makeAffine();  // needed due to bug in Eigen < 3.4
  result.matrix().block<2, 2>(0, 0) = pose.matrix().block<2, 2>(0, 0);
  result.translation().x() = pose.translation().x();
  result.translation().y() = pose.translation().y();
  return result;
}

std::pair<double, double> inverse_odometry(Eigen::Isometry2d odom_diff)
{
  auto linear = odom_diff.translation().norm();
  Eigen::Rotation2Dd rot;
  rot.fromRotationMatrix(odom_diff.linear());
  auto angular = rot.angle();
  return {linear, angular};
}

}  // namespace nobleo_gps_calibration
