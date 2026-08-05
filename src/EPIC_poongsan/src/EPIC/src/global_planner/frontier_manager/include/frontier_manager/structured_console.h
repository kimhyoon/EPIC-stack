#pragma once

#include <ros/node_handle.h>

namespace epic_logging {

// Replaces the local terminal appender for EPIC's structured logger trees
// (`global.*`, `local.*`, `execution.*`, and `sensor.*`).
// The underlying log message remains plain text, so /rosout and bag/file logs
// never receive terminal escape sequences.
void installStructuredConsoleAppender(const ros::NodeHandle &nh);

} // namespace epic_logging
