#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Eigen>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"

#include "ekf_localization/ekf.hpp"
#include "ekf_localization/qos_profiles.hpp"
#include "ekf_localization/msgs_bridge.hpp"

#include "std_msgs/msg/bool.hpp"
#include "dv_msgs/msg/state_estimation.hpp"
#include "dv_msgs/msg/cone_array_stamped.hpp"
#include "dv_msgs/msg/mission_selection.hpp"
#include "dv_msgs/msg/float32_stamped.hpp"

/**
 * @brief ROS wrapper for Ekf Class for eSleek23 localization
 * 
 */
class EkfROSWrapper : public rclcpp::Node, public Ekf
{
  public:
    /**
     * @brief Construct a new Ekf ROS Wrapper object
     * 
     */
    EkfROSWrapper();
    /**
     * @brief Destroy the Ekf R O S Wrapper object
     * 
     */
    virtual ~EkfROSWrapper();
    
    /**
     * @brief Parses a map file into a std::vector of Eigen::Vector3d vectors.
     * 
     * ### Map file format
     * For the map a csv file has to be generated. Every line is a new landmark. Every landmark should be in 
     * coordinate representation. That means the x, y and color of the landmark should be given.
     * Example: 
     * ~~~~~~~~~~~~~~~~~~~~~csv
     * 1,1,1
     * 1,-1,2
     * 2,1,1
     * 2,-1,2
     * ~~~~~~~~~~~~~~~~~~~~~
     * 
     * @param path path relative to the share directory of the node found under "install/pkg_name/share/pkg_name/"
     * @return std::vector<Eigen::Vector3d> 
     */
    static std::vector<Eigen::Vector3d> parse_map_csv(std::string path);
    /**
     * @brief Adjusts the track width in case of the acceleration mission.
     * 
     * @param map map with landmarks in coordinate representation
     * @param tracked_landmarks vector of tracked landmarks with the following form [x, y, count] (count meaning the amount of observations).
     * @return std::vector<Eigen::Vector3d> 
     */
    static std::vector<Eigen::Vector3d> adjust_track_width(std::vector<Eigen::Vector3d> map, std::vector<Eigen::Vector3d> tracked_landmarks, Eigen::Vector3d state_vector);

  private:
    /**
     * @brief Hash function to use Eigen::Vector3d as hash in a unordered map
     */
    struct Vector3dHash {
    std::size_t operator()(const Eigen::Vector3d& vec) const {
        std::size_t seed = 0;
        for (Eigen::DenseIndex i = 0; i < vec.size(); ++i) {
            // Combine the hash value with the element hash
            seed ^= std::hash<double>{}(vec(i)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
      }
    };
    std::unordered_map<Eigen::Vector3d, Eigen::Vector3d, Vector3dHash> tracked_landmarks; ///< key value store for track landmarks function
    bool go_signal_flag = false; ///< flag set if go signal occured
    bool tracking_active = true; ///< flag set if in tracking period
    int mission = -1; ///< currently selected mission
    rclcpp::Time tracking_period_end{0}; ///< end timestamp of the 4 second tracking period after go 
    rclcpp::Time last_state_estimation_timestamp{0}; ///< timestamp of last se message 
    rclcpp::Subscription<dv_msgs::msg::ConeArrayStamped>::SharedPtr camera_sub_; ///< subscriber
    rclcpp::Subscription<dv_msgs::msg::ConeArrayStamped>::SharedPtr lidar_sub_; ///< subscriber
    rclcpp::Subscription<dv_msgs::msg::StateEstimation>::SharedPtr state_estimation_sub_; ///< subscriber
    rclcpp::Subscription<dv_msgs::msg::MissionSelection>::SharedPtr mission_selection_sub_; ///< subscriber
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go_signal_sub_; ///< subscriber
    rclcpp::Publisher<dv_msgs::msg::StateEstimation>::SharedPtr state_estimation_pub_;  ///< publisher
    rclcpp::Publisher<dv_msgs::msg::ConeArrayStamped>::SharedPtr cone_array_pub_;  ///< publisher
    rclcpp::Publisher<dv_msgs::msg::Float32Stamped>::SharedPtr likelihood_pub_;  ///< publisher
    /**
     * @brief Callback triggers the correction step.
     * 
     * @param _msg ConeArrayStamped message containing landmarks
     */
    void landmark_callback(const dv_msgs::msg::ConeArrayStamped& _msg);
    /**
     * @brief Callback triggers motion update.
     * 
     * @param _msg StateEstimation message containing odometry information
     */
    void state_estimation_callback(const dv_msgs::msg::StateEstimation& _msg);
    /**
     * @brief Callback triggers the load of a map.
     * 
     * @param _msg MissionSelection
     */
    void mission_selection_callback(const dv_msgs::msg::MissionSelection& _msg);
    /**
     * @brief Callback triggers the initialization routine.
     * 
     * @param _msg Bool
     */
    void go_signal_callback(const std_msgs::msg::Bool& _msg);
    /**
     * @brief tracks landmarks and generates mean
     * 
     * the landmarks in tracked_landmarks are of the format [x, y, count] with count meaning how often they were observed.
     * 
     * @param observations observations in coordinate representation
     */
    void track_landmarks(std::vector<Eigen::Vector3d> observations);
};