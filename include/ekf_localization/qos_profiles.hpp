#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

/**
 * @brief QoS profile for fast performance. Most previous message will be used.
 */
const rmw_qos_profile_t FAST_PERFORMANCE = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,             // History policy
  1,                                            // History depth
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,       // Reliability policy
  RMW_QOS_POLICY_DURABILITY_VOLATILE,           // Durability policy
  RMW_QOS_DEADLINE_DEFAULT,                     // Deadline policy
  RMW_QOS_LIFESPAN_DEFAULT,                     // Lifespan policy
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,     // Liveliness policy
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,    // Lease duration
  false                                         // Avoid ROS namespace conventions
};

/**
 * @brief QoS profile for high reliability. Messages will be delivered guaranteed.
 */
const rmw_qos_profile_t HIGH_RELIABILITY = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,             // History policy
  10,                                           // History depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,          // Reliability policy
  RMW_QOS_POLICY_DURABILITY_VOLATILE,           // Durability policy
  RMW_QOS_DEADLINE_DEFAULT,                     // Deadline policy
  RMW_QOS_LIFESPAN_DEFAULT,                     // Lifespan policy
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,     // Liveliness policy
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,    // Lease duration
  false                                         // Avoid ROS namespace conventions
};

