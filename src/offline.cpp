// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include <dynamic_reconfigure/server.h>
#include <gnuplot-iostream.h>
#include <nav_msgs/Odometry.h>
#include <nobleo_gps_calibration/CalibratorConfig.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <tf2_msgs/TFMessage.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/program_options.hpp>
#include <limits>
#include <memory>
#include <nobleo_gps_calibration/bag_buffer.hpp>
#include <nobleo_gps_calibration/bag_player.hpp>
#include <nobleo_gps_calibration/calibrator.hpp>
#include <string>
#include <vector>

using nobleo_gps_calibration::BagBuffer;
using nobleo_gps_calibration::BagPlayer;
using nobleo_gps_calibration::Calibrator;
using nobleo_gps_calibration::CalibratorConfig;
namespace po = boost::program_options;

constexpr auto name = "offline";

po::variables_map parse_args(const std::vector<std::string> & args)
{
  ROS_DEBUG_NAMED(name, "parsing '%s'", boost::algorithm::join(args, " ").c_str());

  po::positional_options_description p;
  p.add("input", 1);

  po::options_description options("Allowed options");
  options.add_options()("help,h", "produce help message")(
    "start,s", po::value<double>()->value_name("SEC"), "start at SEC seconds into the bag file")(
    "end,e", po::value<double>()->value_name("SEC"), "stop at SEC seconds into the bag file")(
    "rate,r",
    po::value<double>()->value_name("FACTOR")->default_value(
      std::numeric_limits<double>::infinity()),
    "multiply the publish rate by FACTOR")("plot", "plot residuals before and after optimization");

  po::options_description all_options;  // so that we can hide `input` from help
  all_options.add_options()("input", "input bagfile");
  all_options.add(options);

  po::variables_map vm;
  po::store(po::command_line_parser{args}.positional(p).options(all_options).run(), vm);
  po::notify(vm);

  if (vm.count("help") != 0 || vm.count("input") != 1) {
    std::cout << "usage: offline [options] BAGFILE\n";
    std::cout << options << std::endl;
    exit(EXIT_FAILURE);
  }
  return vm;
}

int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // gather all options that should be parsed by boost
  std::vector<std::string> args;
  ros::removeROSArgs(argc, argv, args);
  assert(args.size() >= 1);
  args.erase(args.begin());

  po::variables_map options;
  try {
    options = parse_args(args);
  } catch (const boost::program_options::error & e) {
    std::cout << "error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "offline");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  ROS_INFO_NAMED(name, "%s started", private_nh.getNamespace().c_str());

  BagPlayer player{options["input"].as<std::string>()};
  if (!options["start"].empty()) {
    player.set_start(ros::Time{options["start"].as<double>()});
  }
  if (!options["end"].empty()) {
    player.set_end(ros::Time{options["end"].as<double>()});
  }
  player.set_playback_speed(options["rate"].as<double>());

  auto buffer = std::make_shared<BagBuffer>(player.bag());
  Calibrator calibrator{private_nh, buffer};

  dynamic_reconfigure::Server<CalibratorConfig> reconfigure_server;
  reconfigure_server.setCallback(
    [&calibrator](const CalibratorConfig & config, uint32_t /*level*/) {
      ROS_INFO_NAMED(name, "reconfigure call");
      calibrator.configure(config);
    });

  auto gps_pub = nh.advertise<nav_msgs::Odometry>("odometry/filtered", 100);
  auto tf_pub = nh.advertise<tf2_msgs::TFMessage>("/tf", 100);
  auto tf_static_pub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 100, true);

  ros::WallDuration{0.1}.sleep();  // wait for topics to connect

  player.register_callback<nav_msgs::Odometry>(
    "odometry/filtered", [&gps_pub, &calibrator](const nav_msgs::OdometryConstPtr & gps) {
      gps_pub.publish(gps);
      calibrator.add(gps);
    });

  player.register_callback<tf2_msgs::TFMessage>(
    "/tf", [&tf_pub](const tf2_msgs::TFMessageConstPtr & tf) { tf_pub.publish(tf); });

  player.register_callback<tf2_msgs::TFMessage>(
    "/tf_static",
    [&tf_static_pub](const tf2_msgs::TFMessageConstPtr & tf) { tf_static_pub.publish(tf); });

  player.start_play();

  auto initial_residuals = calibrator.residuals();
  if (calibrator.solve()) {
    ROS_INFO("Solver found a usable solution");
  } else {
    ROS_FATAL("Solver could not find a usable solution to optimize");
    return EXIT_FAILURE;
  }
  auto residuals = calibrator.residuals();

  if (options.count("plot") != 0) {
    Gnuplot gp;
    gp << "set multiplot layout 2,1 rowsfirst\n";
    gp << "set autoscale\n";
    gp << "set yrange [-0.2:0.2]\n";
    gp << "plot " << gp.file1d(initial_residuals)
       << " using 1 title 'dx', '' using 2 title 'dy', '' using 3 title 'dt'\n";
    gp << "plot " << gp.file1d(residuals)
       << " using 1 title 'dx', '' using 2 title 'dy', '' using 3 title 'dt'\n";
  }

  ROS_INFO_NAMED(name, "%s finished", private_nh.getNamespace().c_str());
  return EXIT_SUCCESS;
}
