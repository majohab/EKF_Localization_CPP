#include <gtest/gtest.h>
#include <Eigen/Eigen>
#define private public
#define protected public
#include "ekf_localization/ekf.hpp"
#undef private
#undef protected

/**
 * @brief Fixture class for Ekf tests
 * 
 */
class EkfTest : public ::testing::Test {
protected:
    void SetUp() override {
        // called before every test
        Eigen::Vector3d init_sigma{1, 1, 0.05};
        Eigen::Vector4d alpha{0.0001, 0.0001, 0.0001, 0.0001};
        std::vector<Eigen::Vector3d> map{Eigen::Vector3d{1, 1, 0}, Eigen::Vector3d{1, -1, 1}, Eigen::Vector3d{2, 2, 0}, Eigen::Vector3d{2, -2, 1}};
        ekf = new Ekf(init_sigma, alpha, map);
        ekf->mu << 0, 0, 0;
    }

    void TearDown() override {
        // called after every test
        delete ekf;
        ekf = nullptr;
    }

    // object can be used by all tests
    Ekf* ekf;
    };

TEST_F(EkfTest, TestMotionModel)
{
    // Fill in the message with some test data
    // set time delta, ensure it will be 0.1 s
    Eigen::Vector3d control {1, 0.1, 0.1};

    // Run the function under test
    Eigen::Vector3d updated_state = ekf->motion_model(control);

    // Check the results
    // Assuming VehicleState has public x, y, theta fields
    double expected_x = 0.099998;
    double expected_y = 0.0004999958;
    double expected_heading = 0.01;
    double epsilon = 1e-6;

    EXPECT_NEAR(updated_state[0], expected_x, epsilon);
    EXPECT_NEAR(updated_state[1], expected_y, epsilon);
    EXPECT_NEAR(updated_state[2], expected_heading, epsilon);
}

TEST_F(EkfTest, TestCalculateG)
{
    // Fill in the message with some test data
    // set time delta, ensure it will be 0.1 s
    Eigen::Vector3d control {1, 0.1, 0.1};

    // Run the function under test
    Eigen::Matrix3d G = ekf->calculate_G(control);

    // Check the results
    Eigen::Matrix3d expected_G;
    expected_G << 1, 0, -0.000499996,
                  0, 1, 0.0999983,
                  0, 0, 1;
    double epsilon = 1e-6;

    ASSERT_TRUE(G.isApprox(expected_G, epsilon));
}

TEST_F(EkfTest, TestCalculateV)
{
    // Fill in the message with some test data
    // set time delta, ensure it will be 0.1 s
    Eigen::Vector3d control {1, 0.1, 0.1};

    // Run the function under test
    Eigen::Matrix<double, 3, 2> V = ekf->calculate_V(control);

    // Check the results
    Eigen::Matrix<double, 3, 2> expected_V;
    expected_V << 0.0999983, -0.0000333330,
                  0.000499996, 0.00499988,
                  0, 0.1;
    double epsilon = 1e-6;
    ASSERT_TRUE(V.isApprox(expected_V, epsilon));
}

TEST_F(EkfTest, TestCalculateM)
{
    // Fill in the message with some test data
    // set time delta, ensure it will be 0.1 s
    Eigen::Vector3d control {1, 0.1, 0.1};

    // Run the function under test
    Eigen::Matrix2d M = ekf->calculate_M(control[0], control[1]);

    // Check the results
    Eigen::Matrix2d expected_M;
    expected_M << 0.000101, 0,
                  0, 0.000101;
    double epsilon = 1e-7;
    ASSERT_TRUE(M.isApprox(expected_M, epsilon));
}

TEST_F(EkfTest, TestCalculateq)
{
    // Fill in the message with some test data
    // set time delta, ensure it will be 0.1 s
    Eigen::Vector3d landmark {1, 1, 1};

    // Run the function under test
    double q = ekf->calculate_q(landmark);

    // Check the results
    double expected_q = 2;
    double epsilon = 1e-7;
    EXPECT_NEAR(q, expected_q, epsilon);
}

TEST_F(EkfTest, TestCalculateZ_hat)
{
    // Fill in the message with some test data
    // set time delta, ensure it will be 0.1 s
    Eigen::Vector3d landmark {1, 1, 1};

    // Run the function under test
    Eigen::Vector3d z_hat = ekf->calculate_z_hat(landmark);

    // Check the results
    Eigen::Vector3d expected_z_hat;
    expected_z_hat << 1.414213562, 0.785398163, 1;
    double epsilon = 1e-6;
    ASSERT_TRUE(z_hat.isApprox(expected_z_hat, epsilon));
}

TEST_F(EkfTest, TestCalculateH)
{
    // Fill in the message with some test data
    Eigen::Vector3d landmark {1, 1, 1};

    // Run the function under test
    Eigen::Matrix3d H = ekf->calculate_H(landmark);

    // Check the results
    Eigen::Matrix3d expected_H;
    expected_H << -0.707106781, -0.707106781, 0,
                  0.5,          -0.5,        -1,
                  0,            0,            0;
    double epsilon = 1e-6;

    ASSERT_TRUE(H.isApprox(expected_H, epsilon));
}

TEST_F(EkfTest, TestDataAssociation)
{
    // Fill in the message with some test data
    Eigen::Vector3d landmark = ekf->calculate_z_hat(Eigen::Vector3d{1.1, 1.1, 0});

    // Run the function under test
    auto [z_hat, S, H, p_i] = ekf->data_association(landmark);

    // Check the results
    Eigen::Vector3d expected_z_hat = ekf->calculate_z_hat(Eigen::Vector3d{1, 1, 0});
    double epsilon = 1e-6;
    ASSERT_TRUE(z_hat.isApprox(expected_z_hat, epsilon));
}

TEST_F(EkfTest, TestGetObservableLandmarks)
{
    // Fill in the message with some test data
    ekf->mu = Eigen::Vector3d{1.5, 0, 0};

    // Run the function under test
    std::vector<Eigen::Vector3d> landmarks = ekf->get_observable_landmarks();

    // Check the results
    Eigen::Vector3d landmark0 = Eigen::Vector3d{2, 2, 0}; 
    Eigen::Vector3d landmark1 = Eigen::Vector3d{2, -2, 1};
    ASSERT_EQ(landmarks.size(), 2);
    ASSERT_EQ(landmarks[0], landmark0);
    ASSERT_EQ(landmarks[1], landmark1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
