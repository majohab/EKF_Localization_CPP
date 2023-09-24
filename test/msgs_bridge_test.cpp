#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"

#include "ekf_localization/msgs_bridge.hpp"

#include "std_msgs/msg/header.hpp"
#include "dv_msgs/msg/state_estimation.hpp"
#include "dv_msgs/msg/cone_array_stamped.hpp"
#include "dv_msgs/msg/cone.hpp"

TEST(ekf_localization, TestfromROSTimestamp)
{
    // test variables
    rclcpp::Time expected_time{0};
    std_msgs::msg::Header header;
    header.stamp.sec = expected_time.seconds();
    header.stamp.nanosec = expected_time.nanoseconds();
    // run function
    rclcpp::Time time;
    msgs_bridge::fromROS(time, header);

    ASSERT_EQ(time.seconds(), expected_time.seconds());
    ASSERT_EQ(time.nanoseconds(), expected_time.nanoseconds());
}

TEST(ekf_localization, TestfromROSSeStateControl)
{
    // test variables
    dv_msgs::msg::StateEstimation msg;
    msg.pos_x = 1;
    msg.pos_y = 1;
    msg.velocity_x = 1;
    msg.velocity_y = 1;
    msg.heading = 1;
    msg.yaw_rate = 1;
    msg.acceleration_x = 1;
    msg.acceleration_y = 1;
    std_msgs::msg::Header header;
    header.stamp.sec = 1;
    header.stamp.nanosec = 0;
    msg.header = header;
    rclcpp::Time last_se_stamp{0};

    // run function
    Eigen::Vector3d state, control;
    msgs_bridge::fromROS(state, control, msg, last_se_stamp);

    // expected values
    Eigen::Vector3d expected_state, expected_control;
    expected_control << 1, 1, 1;
    expected_state << 1, 1, 1;

    // test 
    ASSERT_EQ(state, expected_state);
    ASSERT_EQ(control, expected_control);
}

TEST(ekf_localization, TestfromROSSeControl)
{
    // test variables
    dv_msgs::msg::StateEstimation msg;
    msg.pos_x = 1;
    msg.pos_y = 1;
    msg.velocity_x = 1;
    msg.velocity_y = 1;
    msg.heading = 1;
    msg.yaw_rate = 1;
    msg.acceleration_x = 1;
    msg.acceleration_y = 1;
    std_msgs::msg::Header header;
    header.stamp.sec = 1;
    header.stamp.nanosec = 0;
    msg.header = header;
    rclcpp::Time last_se_stamp{0};

    // run function
    Eigen::Vector3d control;
    msgs_bridge::fromROS(control, msg, last_se_stamp);

    // expected values
    Eigen::Vector3d expected_control;
    expected_control << 1, 1, 1;

    // test 
    ASSERT_EQ(control, expected_control);
}

TEST(ekf_localization, TestfromROSConeArrayStamped)
{
    // test variables
    dv_msgs::msg::ConeArrayStamped msg;
    dv_msgs::msg::Cone cone;
    cone.x = 1;
    cone.y = 1;
    cone.type = 1;
    msg.data.push_back(cone);
    cone.x = 2;
    cone.y = 2;
    cone.type = 2;
    msg.data.push_back(cone);
    std_msgs::msg::Header header;
    header.stamp.sec = 1;
    header.stamp.nanosec = 0;
    msg.header = header;

    // run function
    std::vector<Eigen::Vector3d> observations;
    msgs_bridge::fromROS(observations, msg);

    // expected values
    Eigen::Vector3d cone_vector0{1, 1, 1};
    Eigen::Vector3d cone_vector1{2, 2, 2};

    // test 
    ASSERT_EQ(observations[0], cone_vector0);
    ASSERT_EQ(observations[1], cone_vector1);
}

TEST(ekf_localization, TesttoROSStateEstimation)
{
    // test variables
    dv_msgs::msg::StateEstimation msg;
    msg.pos_x = 1;
    msg.pos_y = 1;
    msg.velocity_x = 1;
    msg.velocity_y = 1;
    msg.heading = 1;
    msg.yaw_rate = 1;
    msg.acceleration_x = 1;
    msg.acceleration_y = 1;
    std_msgs::msg::Header header;
    header.stamp.sec = 1;
    header.stamp.nanosec = 0;
    msg.header = header;
    rclcpp::Time last_se_stamp{0};
    Eigen::Vector3d updated_state{2, 2, 2};

    // run function
    msgs_bridge::toROS(msg, updated_state);

    // test 
    ASSERT_EQ(msg.pos_x, updated_state[0]);
    ASSERT_EQ(msg.pos_y, updated_state[1]);
    ASSERT_EQ(msg.heading, updated_state[2]);
}

TEST(ekf_localization, TesttoROSConeArrayStamped)
{
    // test variables
    dv_msgs::msg::ConeArrayStamped msg;
    std::vector<Eigen::Vector3d> landmarks;
    landmarks.push_back(Eigen::Vector3d{1, 1, 1});
    landmarks.push_back(Eigen::Vector3d{2, 2, 2});

    // run function
    msgs_bridge::toROS(msg, landmarks);

    // expected values
    dv_msgs::msg::Cone expected_cone0;
    expected_cone0.x = 1;
    expected_cone0.y = 1;
    expected_cone0.confidence = 1;
    expected_cone0.type = 1;
    dv_msgs::msg::Cone expected_cone1;
    expected_cone1.x = 2;
    expected_cone1.y = 2;
    expected_cone1.confidence = 1;
    expected_cone1.type = 2;

    // test 
    ASSERT_EQ(msg.data[0].x, 1);
    ASSERT_EQ(msg.data[0].y, 1);
    ASSERT_EQ(msg.data[0].confidence, 1);
    ASSERT_EQ(msg.data[0].type, 1);
    ASSERT_EQ(msg.data[1].x, 2);
    ASSERT_EQ(msg.data[1].y, 2);
    ASSERT_EQ(msg.data[1].confidence, 1);
    ASSERT_EQ(msg.data[1].type, 2);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
