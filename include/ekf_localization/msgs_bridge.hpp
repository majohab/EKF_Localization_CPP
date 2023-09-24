#pragma once
#include <vector>

#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "dv_msgs/msg/state_estimation.hpp"
#include "dv_msgs/msg/cone_array_stamped.hpp"
#include "dv_msgs/msg/cone.hpp"

namespace msgs_bridge
{
    /**
     * @brief Converts a std_msgs::msg::Header to a rclcpp::Time
     * 
     * @param _msg rclcpp::Time
     * @param _msg std_msgs::msg::Header
     */
    void fromROS(rclcpp::Time &timestamp, std_msgs::msg::Header _msg);
    /**
     * @brief Converts dv_msgs::msg::StateEstimation to control and state vectors represented as Eigen::Vector3d
     * 
     */
    void fromROS(Eigen::Vector3d &state, Eigen::Vector3d &control, dv_msgs::msg::StateEstimation _msg, rclcpp::Time last_se_timestamp);
    /**
     * @brief Converts dv_msgs::msg::StateEstimation to control and state vectors represented as Eigen::Vector3d
     * 
     */
    void fromROS(Eigen::Vector3d &control, dv_msgs::msg::StateEstimation _msg, rclcpp::Time last_se_timestamp);    
    /**
     * @brief Converts dv_msgs::msg::ConeArrayStamped to vector oflandmarks in coordinate representaion
     * 
     * @param _msg 
     */
    void fromROS(std::vector<Eigen::Vector3d> &observations, dv_msgs::msg::ConeArrayStamped _msg);
    /**
     * @brief Updates state estimation message with new state
     * 
     * @param _msg state estimation message
     * @param state_vector current state
     */
    void toROS(dv_msgs::msg::StateEstimation &_msg, Eigen::Vector3d state_vector);
    /**
     * @brief Adds the landmarks to the array of the message
     * 
     * @param _msg 
     * @param landmarks landmarks in coordinate representation
     */
    void toROS(dv_msgs::msg::ConeArrayStamped &_msg, std::vector<Eigen::Vector3d> landmarks);
}