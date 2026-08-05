#include <frontier_manager/structured_console.h>

#include <log4cxx/appenderskeleton.h>
#include <log4cxx/helpers/pool.h>
#include <log4cxx/level.h>
#include <log4cxx/logger.h>
#include <log4cxx/spi/loggingevent.h>
#include <ros/console.h>
#include <ros/rosout_appender.h>

#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

// roscpp owns this appender.  It is intentionally not exposed in the public
// header, but is exported by libroscpp and used here so the custom terminal
// formatter can keep /rosout messages plain (without terminal ANSI codes).
namespace ros {
extern ROSOutAppender *g_rosout_appender;
}

namespace epic_logging {
namespace {

class StructuredConsoleAppender final : public log4cxx::AppenderSkeleton {
public:
  explicit StructuredConsoleAppender(bool use_color) : use_color_(use_color) {}

  void close() override { closed = true; }
  bool requiresLayout() const override { return false; }

protected:
  void append(const log4cxx::spi::LoggingEventPtr &event,
              log4cxx::helpers::Pool &) override {
    const std::string message(event->getRenderedMessage().begin(),
                              event->getRenderedMessage().end());
    const int level = event->getLevel()->toInt();

    const char *severity = "DEBUG";
    const char *body_color = "\033[90m"; // grey
    FILE *stream = stdout;
    if (level >= log4cxx::Level::FATAL_INT) {
      severity = "FATAL";
      body_color = "\033[1;97;41m"; // white on red
      stream = stderr;
    } else if (level >= log4cxx::Level::ERROR_INT) {
      severity = "ERROR";
      body_color = "\033[91m"; // bright red
      stream = stderr;
    } else if (level >= log4cxx::Level::WARN_INT) {
      severity = "WARN";
      body_color = "\033[38;5;208m"; // orange
      stream = stderr;
    } else if (level >= log4cxx::Level::INFO_INT) {
      severity = "INFO";
      body_color = "\033[97m"; // white
    }

    // additivity is disabled for structured loggers to avoid a second,
    // whole-line-coloured console print.  Forward the original message to
    // roscpp's ROSOutAppender explicitly, before adding any terminal colour.
    // This is also the representation recorded by /rosout and rosbag.
    if (ros::g_rosout_appender != nullptr) {
      ros::console::Level ros_level = ros::console::levels::Debug;
      if (level >= log4cxx::Level::FATAL_INT)
        ros_level = ros::console::levels::Fatal;
      else if (level >= log4cxx::Level::ERROR_INT)
        ros_level = ros::console::levels::Error;
      else if (level >= log4cxx::Level::WARN_INT)
        ros_level = ros::console::levels::Warn;
      else if (level >= log4cxx::Level::INFO_INT)
        ros_level = ros::console::levels::Info;

      const auto &location = event->getLocationInformation();
      const std::string function = location.getMethodName();
      ros::g_rosout_appender->log(ros_level, message.c_str(),
                                  location.getFileName(), function.c_str(),
                                  location.getLineNumber());
    }

    const auto now = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()) %
                    1000;
    const std::time_t wall = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm;
    localtime_r(&wall, &local_tm);
    std::ostringstream timestamp;
    timestamp << std::put_time(&local_tm, "%H:%M:%S") << '.' << std::setw(3)
              << std::setfill('0') << ms.count();

    // Every structured message begins with [domain.*]. Colour that header cyan
    // and reset to the severity colour for the body.
    size_t header_end = std::string::npos;
    if (message.compare(0, 8, "[global.") == 0 ||
        message.compare(0, 7, "[local.") == 0 ||
        message.compare(0, 11, "[execution.") == 0 ||
        message.compare(0, 8, "[sensor.") == 0)
      header_end = message.find(']');

    std::lock_guard<std::mutex> lock(output_mutex_);
    if (!use_color_) {
      std::fprintf(stream, "%s [%-5s] %s\n", timestamp.str().c_str(), severity,
                   message.c_str());
    } else if (header_end != std::string::npos) {
      const std::string header = message.substr(0, header_end + 1);
      const std::string body = message.substr(header_end + 1);
      std::fprintf(stream,
                   "\033[90m%s \033[0m%s[%-5s] \033[0m\033[96m%s\033[0m%s%s"
                   "\033[0m\n",
                   timestamp.str().c_str(), body_color, severity,
                   header.c_str(), body_color, body.c_str());
    } else {
      std::fprintf(stream, "\033[90m%s \033[0m%s[%-5s] %s\033[0m\n",
                   timestamp.str().c_str(), body_color, severity,
                   message.c_str());
    }
    std::fflush(stream);
  }

private:
  bool use_color_;
  std::mutex output_mutex_;
};

log4cxx::AppenderPtr structured_console_appender;

log4cxx::LevelPtr configuredLevel(const ros::NodeHandle &nh,
                                  const std::string &domain) {
  std::string value = "INFO";
  nh.param("logging/" + domain + "_level", value, value);
  std::transform(value.begin(), value.end(), value.begin(), ::toupper);
  if (value == "DEBUG")
    return log4cxx::Level::getDebug();
  if (value == "WARN")
    return log4cxx::Level::getWarn();
  if (value == "ERROR")
    return log4cxx::Level::getError();
  if (value != "INFO")
    ROS_WARN("[logging] invalid logging/%s_level=%s; using INFO",
             domain.c_str(), value.c_str());
  return log4cxx::Level::getInfo();
}

} // namespace

void installStructuredConsoleAppender(const ros::NodeHandle &nh) {
  bool use_color = true;
  if (!nh.getParam("logging/color_console", use_color))
    nh.param("logging/global_color_console", use_color, true);
  if (std::getenv("NO_COLOR") != nullptr)
    use_color = false;

  structured_console_appender =
      log4cxx::AppenderPtr(new StructuredConsoleAppender(use_color));
  structured_console_appender->setName(LOG4CXX_STR("epic_structured_console"));

  const std::string domains[] = {"global", "local", "execution", "sensor"};
  for (const auto &domain : domains) {
    log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(domain);
    logger->setAdditivity(false);
    logger->removeAllAppenders();
    logger->addAppender(structured_console_appender);
    logger->setLevel(configuredLevel(nh, domain));
  }

  ROS_DEBUG("[logging] structured console installed for global/local/execution/"
            "sensor: color=%s; stored messages remain ANSI-free",
            use_color ? "on" : "off");
}

} // namespace epic_logging
