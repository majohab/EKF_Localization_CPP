#include "ekf_localization/ekf_localization.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

EkfROSWrapper::EkfROSWrapper() : Node("ekf_localization")
{
    // ROS params
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "The initial covariance sigma at t=0.";
    this->declare_parameter("sigma", std::vector<float>{1, 1, 0.05});
    std::string state_estimation_topic{"/vdc/udp_interface/state_estimation"};
    std::vector<double> sigma = this->get_parameter("sigma").as_double_array();
    this->set_sigma(Eigen::Vector3d{sigma[0], sigma[1], sigma[2]});
    param_desc.description = "The noise alpha in the control space.";
    this->declare_parameter("alpha", std::vector<float>{0.0001, 0.0001, 0.0001, 0.0001});
    std::vector<double> alpha = this->get_parameter("alpha").as_double_array();
    this->set_alpha(Eigen::Vector4d{alpha[0], alpha[1], alpha[2], alpha[3]});
    // QoS
    /**
     * ### QoS
     * The following code is the correct way to defining qos profiles. But it is incompatible with the python qos profiles. So we need 
     * the currently implemented versions of the profiles.
     * @todo fix the stupid python implementation and create a package for shared code over all nodes. Then use the following code:
     * ~~~~~~~~~~~~~~~~~~~cpp
     * //Create the rclcpp::QoSInitialization struct
     * rclcpp::QoSInitialization fast_performance_init = rclcpp::QoSInitialization::from_rmw(FAST_PERFORMANCE);
     * rclcpp::QoSInitialization high_reliability_init = rclcpp::QoSInitialization::from_rmw(HIGH_RELIABILITY);
     * // Create the rclcpp::QoS object from the rclcpp::QoSInitialization struct
     * const rclcpp::QoS fast_performance(fast_performance_init);
     * const rclcpp::QoS high_reliability(high_reliability_init);
     * ~~~~~~~~~~~~~~~~~~~
    */
    //The following code is less elegant but needed because the python implementation of the qos profiles is not compatible with
    //the c++ implementaion (not all parameters are set) so here we use a intermediate solution.
    rclcpp::QoS fast_performance(1);
    fast_performance.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    fast_performance.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    fast_performance.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE);
    rclcpp::QoS high_reliability(10);
    high_reliability.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    high_reliability.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    high_reliability.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE);
    // Subscriber
    this->camera_sub_ = this->create_subscription<dv_msgs::msg::ConeArrayStamped>("/perception/camera/conearraystamped", fast_performance, std::bind(&EkfROSWrapper::landmark_callback, this, _1));
    this->lidar_sub_ = this->create_subscription<dv_msgs::msg::ConeArrayStamped>("/perception/lidar/conearraystamped", fast_performance, std::bind(&EkfROSWrapper::landmark_callback, this, _1));
    this->state_estimation_sub_ = this->create_subscription<dv_msgs::msg::StateEstimation>("/vdc/udp_interface/state_estimation", fast_performance, std::bind(&EkfROSWrapper::state_estimation_callback, this, _1));
    this->mission_selection_sub_ = this->create_subscription<dv_msgs::msg::MissionSelection>("/vdc/udp_interface/mission_selection", high_reliability, std::bind(&EkfROSWrapper::mission_selection_callback, this, _1));
    this->go_signal_sub_ = this->create_subscription<std_msgs::msg::Bool>("/res/udp_interface/goSignal", high_reliability, std::bind(&EkfROSWrapper::go_signal_callback, this, _1));
    // Publisher
    this->state_estimation_pub_ = this->create_publisher<dv_msgs::msg::StateEstimation>("/navigation/slam/state_estimation", fast_performance);
    this->cone_array_pub_ = this->create_publisher<dv_msgs::msg::ConeArrayStamped>("/navigation/slam/conearraystamped", fast_performance);
    RCLCPP_INFO(this->get_logger(), "ekf_localization initialized.");
}

EkfROSWrapper::~EkfROSWrapper()
{
}

std::vector<Eigen::Vector3d> EkfROSWrapper::parse_map_csv(std::string path)
{
    //parse corresponding map file
    std::vector<Eigen::Vector3d> map;
	std::vector<double> row;
	std::string line, value;
    Eigen::Vector3d landmark;
    std::fstream file (path, std::ios::in);
    // enable exceptions
	if(file.is_open())
	{
		while(getline(file, line))
		{
			row.clear();
			std::stringstream str(line);
            //split by delimitor
			while(getline(str, value, ','))
				row.push_back(std::stod(value));
            landmark << row[0], row[1], row[2];
			map.push_back(landmark);
		}
	}
	else
		throw path;
    return map;
}

void EkfROSWrapper::mission_selection_callback(const dv_msgs::msg::MissionSelection & _msg)
{
    std::string path, mission_name;
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ekf_localization");
    //select file path based on mission selection
    if(_msg.selected_mission == dv_msgs::msg::MissionSelection::SKIDPAD) 
    {
        this->mission = dv_msgs::msg::MissionSelection::SKIDPAD;
        mission_name = "SKIDPAD";
        path = package_share_directory + "/SKIDPAD_X.csv";
    }
    else if(_msg.selected_mission == dv_msgs::msg::MissionSelection::ACCELERATION) 
    {
        this->mission = dv_msgs::msg::MissionSelection::SKIDPAD;
        mission_name = "ACCELERATION";
        path = package_share_directory + "/ACCEL_X.csv";
    }
    else RCLCPP_WARN(this->get_logger(), "Received mission %d! Unknown mission!", _msg.selected_mission);
    RCLCPP_INFO(this->get_logger(), "Received mission %s!", mission_name.c_str());
    //parse map csv file
    std::vector<Eigen::Vector3d> parsed_map;
    try
    {
        parsed_map = EkfROSWrapper::parse_map_csv(path);
    }
    catch(std::string e)
    {
        std::cerr << "Could not open file: " << e << std::endl;
    }
    //set map 
    this->set_map(parsed_map);
}

void EkfROSWrapper::go_signal_callback(const std_msgs::msg::Bool & _msg)
{
    if(_msg.data && !this->go_signal_flag && this->mission == dv_msgs::msg::MissionSelection::ACCELERATION)
    {
        //calculate track width
        std::vector<Eigen::Vector3d> vector_tracked_landmarks;
        for(auto v : this->tracked_landmarks) {
            vector_tracked_landmarks.push_back(v.second);  
        }
        std::vector<Eigen::Vector3d> map;
        map = EkfROSWrapper::adjust_track_width(this->get_map(), vector_tracked_landmarks, this->get_state());
        this->set_map(map);
        this->go_signal_flag = true;
    }
}

void EkfROSWrapper::landmark_callback(const dv_msgs::msg::ConeArrayStamped & _msg)
{
    // parse the msg 
    std::vector<Eigen::Vector3d> observations;
    msgs_bridge::fromROS(observations, _msg);
    // trigger correction
    this->correction_step(observations);
}

void EkfROSWrapper::state_estimation_callback(const dv_msgs::msg::StateEstimation & _msg)
{
    // parse msg
    if(this->last_state_estimation_timestamp.nanoseconds() == 0) msgs_bridge::fromROS(this->last_state_estimation_timestamp, _msg.header);
    Eigen::Vector3d control;
    msgs_bridge::fromROS(control, _msg, this->last_state_estimation_timestamp);
    // trigger motion update
    this->motion_update(control);
    //publish update
    rclcpp::Time timestamp = this->get_clock()->now();
    // landmarks 
    std::vector<Eigen::Vector3d> landmarks = this->get_observable_landmarks();
    dv_msgs::msg::ConeArrayStamped cone_array_stamped; 
    cone_array_stamped.header.stamp = timestamp;
    cone_array_stamped.header.frame_id = "navigation";
    msgs_bridge::toROS(cone_array_stamped, landmarks);
    this->cone_array_pub_->publish(cone_array_stamped);
    // convert to ros msgs
    dv_msgs::msg::StateEstimation state_estimation = _msg;
    Eigen::Vector3d state = this->get_state();
    msgs_bridge::toROS(state_estimation, state);
    state_estimation.header.stamp = timestamp;
    state_estimation.header.frame_id = "navigation";
    this->state_estimation_pub_->publish(state_estimation);
}

void EkfROSWrapper::track_landmarks(std::vector<Eigen::Vector3d> observations)
{
    std::vector<Eigen::Vector3d>::iterator iter = observations.begin();
    int count;
    Eigen::Vector3d observation;
    for(iter; iter < observations.end(); iter++)
    {
        observation = *iter;
        auto [z_hat, S, H, p_i] = this->data_association(this->calculate_z_hat(observation));
        auto it = this->tracked_landmarks.find(z_hat);
        if (it == this->tracked_landmarks.end()) 
        {
            //landmark was never tracked
            this->tracked_landmarks[z_hat] = Eigen::Vector3d{ observation[0], observation[1], 1};
        }
        if (it != this->tracked_landmarks.end())
        {
            // landmark is already tracked
            count = this->tracked_landmarks[z_hat][2]; // save count to use vector operations
            this->tracked_landmarks[z_hat] = this->tracked_landmarks[z_hat] * count / (count+1) + observation * 1 / (count+1); // build mean of vector (count is overriden)
            this->tracked_landmarks[z_hat][2] = count+1; // replace overrriden count with updated count
        }
        else RCLCPP_WARN(this->get_logger(), "Landmark [%f, %f, %f] was associated with [%f, %f, %f] yet could not be tracked!", observation[0], observation[1], observation[2], z_hat[0], z_hat[1], z_hat[2]);
    }
}

std::vector<Eigen::Vector3d> EkfROSWrapper::adjust_track_width(std::vector<Eigen::Vector3d> map, std::vector<Eigen::Vector3d> tracked_landmarks, Eigen::Vector3d state_vector)
{
    // Catch case with less than two observations
    if(tracked_landmarks.size() < 2) return map;
    // Calculate distances and keep track of indices
    std::vector<std::pair<double, size_t>> distances_and_indices;
    for (size_t i = 0; i < tracked_landmarks.size(); ++i) {
        // filter for often observed landmarks
        if (tracked_landmarks[i][2] >= 30)
        {
            double distance = (state_vector - tracked_landmarks[i]).norm();
            distances_and_indices.push_back(std::make_pair(distance, i));
        }
        else
        {
            tracked_landmarks.erase(std::next(tracked_landmarks.begin(), i));
        }
    }
    // Sort distances in ascending order
    sort(distances_and_indices.begin(), distances_and_indices.end());
    // Find the two closest vectors
    Eigen::Vector2d closest_landmark_1 = tracked_landmarks[distances_and_indices[0].second].head(2);
    Eigen::Vector2d closest_landmark_2 = tracked_landmarks[distances_and_indices[1].second].head(2);
    // Calculate track width
    double track_width = (closest_landmark_1 - closest_landmark_2).norm();
    // Adjust map
    for (size_t i = 0; i < map.size(); ++i) {
        if(map[i][1] >= 0) map[i] << map[i][0], track_width / 2, map[i][2];
        else map[i] << map[i][0], -track_width / 2, map[i][2];
    }
    return map;
}

