#include "ekf_localization/msgs_bridge.hpp"

void msgs_bridge::fromROS(rclcpp::Time &timestamp, std_msgs::msg::Header _msg)
{
    timestamp = rclcpp::Time {_msg.stamp.sec, _msg.stamp.nanosec};
}

void msgs_bridge::fromROS(Eigen::Vector3d &state, Eigen::Vector3d &control, dv_msgs::msg::StateEstimation _msg, rclcpp::Time last_se_timestamp)
{
    // control
    rclcpp::Time timestamp;
    msgs_bridge::fromROS(timestamp, _msg.header);
    control << _msg.velocity_x, _msg.yaw_rate, (timestamp - last_se_timestamp).seconds();
    // state
    state << _msg.pos_x, _msg.pos_y, _msg.heading;
}

void msgs_bridge::fromROS(Eigen::Vector3d &control, dv_msgs::msg::StateEstimation _msg, rclcpp::Time last_se_timestamp)
{
    rclcpp::Time timestamp;
    msgs_bridge::fromROS(timestamp, _msg.header);
    rclcpp::Duration delta = timestamp - last_se_timestamp;
    control << _msg.velocity_x, _msg.yaw_rate, delta.seconds();
}

void msgs_bridge::fromROS(std::vector<Eigen::Vector3d> &observations, dv_msgs::msg::ConeArrayStamped _msg)
{
    std::vector<Eigen::Vector3d> converted_obervations;
    Eigen::Vector3d obs_vector;
    for(auto obs : _msg.data)
    {
        obs_vector << obs.x, obs.y, obs.type;
        observations.push_back(obs_vector);
    }
}

void msgs_bridge::toROS(dv_msgs::msg::StateEstimation &_msg, Eigen::Vector3d state_vector)
{
    _msg.pos_x = state_vector[0];
    _msg.pos_y = state_vector[1];
    _msg.heading = state_vector[2];
}

void msgs_bridge::toROS(dv_msgs::msg::ConeArrayStamped &_msg, std::vector<Eigen::Vector3d> landmarks)
{
    std::vector<dv_msgs::msg::Cone> converted_landmarks;
    dv_msgs::msg::Cone converted_landmark;
    for(auto landmark : landmarks)
    {
        converted_landmark.x = landmark[0];
        converted_landmark.y = landmark[1];
        converted_landmark.confidence = 1;
        converted_landmark.type = landmark[2];
        converted_landmarks.push_back(converted_landmark);
    }
    _msg.data = converted_landmarks;
}