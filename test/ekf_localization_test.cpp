#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ekf_localization/ekf_localization.hpp"

TEST(ekf_localization, TestParseMapCSV)
{
    // Run the function under test
    std::vector<Eigen::Vector3d> map;
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ekf_localization");
    try
    {
        map = EkfROSWrapper::parse_map_csv(package_share_directory + "/TEST_MAP.csv");
    }
    catch(std::string e)
    {
        std::cerr << "Could not open file: " << e << std::endl;
    }

    // Check the results
    std::vector<Eigen::Vector3d> expected_map;
    expected_map.push_back(Eigen::Vector3d {1, 1, 1});
    expected_map.push_back(Eigen::Vector3d {1,-1, 2});
    expected_map.push_back(Eigen::Vector3d {2, 1, 1});
    expected_map.push_back(Eigen::Vector3d {2,-1, 2});

    ASSERT_EQ(map[0], expected_map[0]);
    ASSERT_EQ(map[1], expected_map[1]);
    ASSERT_EQ(map[2], expected_map[2]);
    ASSERT_EQ(map[3], expected_map[3]);
}

TEST(ekf_localization, TestAdjustTrackWidth)
{
    // Run the function under test
    std::vector<Eigen::Vector3d> input_map;
    input_map.push_back(Eigen::Vector3d {1, 1, 1});
    input_map.push_back(Eigen::Vector3d {1,-1, 2});
    input_map.push_back(Eigen::Vector3d {2, 1, 1});
    input_map.push_back(Eigen::Vector3d {2,-1, 2});

    std::vector<Eigen::Vector3d> tracked_landmarks;
    tracked_landmarks.push_back(Eigen::Vector3d {1, 2, 50});
    tracked_landmarks.push_back(Eigen::Vector3d {1,-2, 30});
    tracked_landmarks.push_back(Eigen::Vector3d {2, 2, 10});
    tracked_landmarks.push_back(Eigen::Vector3d {2,-2, 20});

    Eigen::Vector3d state_vector;
    state_vector << 0, 0, 0;

    // execute function
    std::vector<Eigen::Vector3d> map;
    map = EkfROSWrapper::adjust_track_width(input_map, tracked_landmarks, state_vector);

    // Check the results
    std::vector<Eigen::Vector3d> expected_map;
    expected_map.push_back(Eigen::Vector3d {1, 2, 1});
    expected_map.push_back(Eigen::Vector3d {1,-2, 2});
    expected_map.push_back(Eigen::Vector3d {2, 2, 1});
    expected_map.push_back(Eigen::Vector3d {2,-2, 2});

    ASSERT_EQ(map[0], expected_map[0]);
    ASSERT_EQ(map[1], expected_map[1]);
    ASSERT_EQ(map[2], expected_map[2]);
    ASSERT_EQ(map[3], expected_map[3]);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
