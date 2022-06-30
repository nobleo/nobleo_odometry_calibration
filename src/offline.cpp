#include <dynamic_reconfigure/server.h>
#include <gnuplot-iostream.h>
#include <nav_msgs/Odometry.h>
#include <nobleo_gps_calibration/CalibratorConfig.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "./bag_buffer.hpp"
#include "./bag_player.hpp"
#include "./calibrator.hpp"

using nobleo_gps_calibration::BagBuffer;
using nobleo_gps_calibration::BagPlayer;
using nobleo_gps_calibration::Calibrator;
using nobleo_gps_calibration::CalibratorConfig;

constexpr auto name = "offline";

int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "offline");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  ROS_INFO_NAMED(name, "%s started", private_nh.getNamespace().c_str());

  std::vector<std::string> args;
  ros::removeROSArgs(argc, argv, args);
  if (args.size() != 2) {
    puts("usage: offline BAGFILE");
    exit(EXIT_FAILURE);
  }

  BagPlayer player{args[1]};
  player.set_playback_speed(private_nh.param("rate", std::numeric_limits<double>::infinity()));

  auto buffer = std::make_shared<BagBuffer>(player.bag);
  Calibrator calibrator{buffer};

  dynamic_reconfigure::Server<CalibratorConfig> reconfigure_server;
  reconfigure_server.setCallback(
    [&calibrator](const CalibratorConfig & config, uint32_t /*level*/) {
      ROS_INFO_NAMED(name, "reconfigure call");
      calibrator.configure(config);
    });

  ros::WallDuration{0.1}.sleep();  // wait for topics to connect

  player.register_callback<nav_msgs::Odometry>(
    "/odometry/filtered",
    [&calibrator](const nav_msgs::OdometryConstPtr & gps) { calibrator.add(gps); });

  player.start_play();

  auto initial_residuals = calibrator.residuals();
  calibrator.solve();
  auto residuals = calibrator.residuals();

  Gnuplot gp;
  gp << "set multiplot layout 2,1 rowsfirst\n";
  gp << "set autoscale\n";
  gp << "set yrange [-0.2:0.2]\n";
  gp << "plot " << gp.file1d(initial_residuals)
     << " using 1 title 'dx', '' using 2 title 'dy', '' using 3 title 'dt'\n";
  gp << "plot " << gp.file1d(residuals)
     << " using 1 title 'dx', '' using 2 title 'dy', '' using 3 title 'dt'\n";

  ROS_INFO_NAMED(name, "%s finished", private_nh.getNamespace().c_str());
  return EXIT_SUCCESS;
}
