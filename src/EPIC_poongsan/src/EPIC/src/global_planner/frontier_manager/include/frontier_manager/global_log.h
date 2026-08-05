#pragma once

#include <log4cxx/logger.h>
#include <ros/console.h>

inline bool epicStructuredDebugEnabled(const char *category) {
  return log4cxx::Logger::getLogger(category)->isDebugEnabled();
}

inline bool epicGlobalDebugEnabled(const char *category) {
  return epicStructuredDebugEnabled(category);
}

// Structured global-planning logs.  The category is both the real rosconsole
// logger name (runtime-selectable with set_logger_level/rqt_logger_level) and a
// plain-text message prefix (kept in /rosout, bags, and files).  Colour is added
// later by exploration_node's console appender; never put ANSI escapes here.
#define EPIC_LOG_DEBUG(configured_detail, required_detail, category, ...)      \
  ROS_LOG_COND((configured_detail) >= (required_detail),                      \
               ::ros::console::levels::Debug, category,                      \
               "[" category "] " __VA_ARGS__)

#define EPIC_LOG_INFO(configured_detail, required_detail, category, ...)       \
  ROS_LOG_COND((configured_detail) >= (required_detail),                      \
               ::ros::console::levels::Info, category,                       \
               "[" category "] " __VA_ARGS__)

#define EPIC_LOG_INFO_THROTTLE(period, configured_detail, required_detail,     \
                               category, ...)                                  \
  do {                                                                         \
    if ((configured_detail) >= (required_detail))                              \
      ROS_LOG_THROTTLE(period, ::ros::console::levels::Info, category,         \
                       "[" category "] " __VA_ARGS__);                        \
  } while (false)

#define EPIC_LOG_WARN(category, ...)                                           \
  ROS_LOG(::ros::console::levels::Warn, category,                             \
          "[" category "] " __VA_ARGS__)

#define EPIC_LOG_WARN_THROTTLE(period, category, ...)                          \
  ROS_LOG_THROTTLE(period, ::ros::console::levels::Warn, category,            \
                   "[" category "] " __VA_ARGS__)

#define EPIC_LOG_ERROR(category, ...)                                          \
  ROS_LOG(::ros::console::levels::Error, category,                            \
          "[" category "] " __VA_ARGS__)

#define EPIC_LOG_ERROR_THROTTLE(period, category, ...)                         \
  ROS_LOG_THROTTLE(period, ::ros::console::levels::Error, category,           \
                   "[" category "] " __VA_ARGS__)

// Backward-compatible names for the global-planning instrumentation.
#define EPIC_GLOG_DEBUG EPIC_LOG_DEBUG
#define EPIC_GLOG_INFO EPIC_LOG_INFO
#define EPIC_GLOG_INFO_THROTTLE EPIC_LOG_INFO_THROTTLE
#define EPIC_GLOG_WARN EPIC_LOG_WARN
#define EPIC_GLOG_WARN_THROTTLE EPIC_LOG_WARN_THROTTLE
#define EPIC_GLOG_ERROR EPIC_LOG_ERROR
#define EPIC_GLOG_ERROR_THROTTLE EPIC_LOG_ERROR_THROTTLE
