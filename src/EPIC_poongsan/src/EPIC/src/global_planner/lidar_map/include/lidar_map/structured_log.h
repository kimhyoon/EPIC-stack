#pragma once

#include <ros/console.h>

// Keep lidar_map independent from planner packages while using the same named
// rosconsole hierarchy and plain-text prefixes as the EPIC structured console.
#define EPIC_SENSOR_DEBUG(category, ...)                                      \
  ROS_LOG(::ros::console::levels::Debug, category,                           \
          "[" category "] " __VA_ARGS__)

#define EPIC_SENSOR_DEBUG_THROTTLE(period, category, ...)                     \
  ROS_LOG_THROTTLE(period, ::ros::console::levels::Debug, category,          \
                   "[" category "] " __VA_ARGS__)

#define EPIC_SENSOR_INFO(category, ...)                                       \
  ROS_LOG(::ros::console::levels::Info, category,                            \
          "[" category "] " __VA_ARGS__)

#define EPIC_SENSOR_WARN(category, ...)                                       \
  ROS_LOG(::ros::console::levels::Warn, category,                            \
          "[" category "] " __VA_ARGS__)

#define EPIC_SENSOR_WARN_THROTTLE(period, category, ...)                      \
  ROS_LOG_THROTTLE(period, ::ros::console::levels::Warn, category,           \
                   "[" category "] " __VA_ARGS__)

#define EPIC_SENSOR_ERROR(category, ...)                                      \
  ROS_LOG(::ros::console::levels::Error, category,                           \
          "[" category "] " __VA_ARGS__)

#define EPIC_SENSOR_FATAL(category, ...)                                      \
  ROS_LOG(::ros::console::levels::Fatal, category,                           \
          "[" category "] " __VA_ARGS__)
