#ifndef EKF_LOCALIZATION__EKF_HPP_
#define EKF_LOCALIZATION__EKF_HPP_

#include "ekf_localization/visibility_control.h"
#include <Eigen/Eigen>
#include <math.h>
#include <numeric>

/**
 * @brief This class implements the EKF Localization with Unknown Correspondences by thrun et al.
 * 
 * The following terminology is used in the code:
 * - The state vector mu is defined as a 3D vector with the x,y and theta (heading). 
 * - The control vector is defined as a 3D vector with v, omega and delta t. v is the translational velocity, omega the rotational velocity and t the time delta between the last and current step.
 * - The landmark vector is used in coordinate ([x, y, s]) and range bearing representation ([r, phi, s]). 
 */
class Ekf
{
  public:
    /**
     * @brief Construct a new Ekf object
     * 
     */
    Ekf();

    /**
     * @brief Construct a new Ekf object
     * 
     * @param init_sigma initial variance for x, y, and theta
     * @param alpha noise of control space v_x, omega_x, v_y, omega_y
     */
    Ekf(Eigen::Vector3d init_sigma, Eigen::Vector4d alpha, std::vector<Eigen::Vector3d> map);
    
    /**
     * @brief Destruct the Ekf object
     */
    virtual ~Ekf();

    /**
     * @brief Carries out the motion update or prediction step.
     * 
     * Includes Formula 4 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * @param control 
     */
    void motion_update(Eigen::Vector3d control);
    /**
     * @brief Carries out the correction step. 
     * 
     * Includes Formula 21 in Table 7.2 probabalistic robotics (thrun et al.).
     * 
     * @param observations vector of observations in coordinate representation
     */
    double correction_step(std::vector<Eigen::Vector3d> observations);
        /**
     * @brief Set the sigma object
     * 
     * @param sigma 
     */
    void set_sigma(Eigen::Vector3d sigma);
    /**
     * @brief Set the alpha object
     * 
     * @param alpha 
     */
    void set_alpha(Eigen::Vector4d alpha);
    /**
     * @brief Set the map object
     * 
     * @param map 
     */
    void set_map(std::vector<Eigen::Vector3d> map);
    /**
     * @brief Set the init flag object
     * 
     */
    void set_init_flag();
    /**
     * @brief Get the map object
     * 
     * @return std::vector<Eigen::Vector3d> 
     */
    std::vector<Eigen::Vector3d> get_map();
    /**
     * @brief Get the state object
     * 
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d get_state();
    /**
     * @brief Set the state object
     * 
     * @param state_vector
     */
    void set_state(Eigen::Vector3d state_vector);
    /**
     * @brief Get the observable landmarks at the current position in the map
     * 
     * It is assumed that all landmarks that are at the time in front of the robot can be observed
     * so all of them are send.
     * 
     * @return std::vector<Eigen::Vector3d> 
     */
    std::vector<Eigen::Vector3d> get_observable_landmarks();
    
  protected:
    /**
     * @brief Calculates the correspondence j between the landmark and the observation via a maximum likelihood estimation.
     * 
     * The correspondence j assigns the most likely landmark k to the observation z.
     * 
     * Line 10-16 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * @todo currently the association is only based on the x, y position of the observation the signature S (Color) is not used. Has to be added in the future.
     * @param observation observation vector z.
     * @return std::vector<std::tuple<Eigen::Vector3d, Eigen::Matrix3d>> the assigned landmark k to the obserevation z, the corresponding H matrix, the corresponding S matrix and the likelihood.
     */
    std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d, double> data_association(Eigen::Vector3d observation);
        /**
     * @brief Converts the landmark from coordinate representation to a range bearing landmark.
     * 
     * Formula 12 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * @param landmark landmark vector in coordinate representation
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d calculate_z_hat(Eigen::Vector3d landmark);
    
  private:
    bool init_flag; ///< Initialization flag for when using the default constructor
    const float EPS = 1e-8; ///< Threshhold for approximation of straight forward movement in motion_model().
    const float MAPRANGE = 20; ///< Threshhold for the width the robot can see. Used in get_observable_landmarks().
    Eigen::Matrix3d Q_t; ///< Covariance of additional noise of the motion_update() through the motion_model()
    Eigen::Vector3d mu; ///< Vehicle state of the last ekf iteration. Resulting either from the motion_update() or the correction_step()
    Eigen::Matrix3d sigma; ///< Covariance of the state vector mu.
    Eigen::Vector4d alpha; ///< Noise of the control space used in calculate_Mt() to calculate the covariance matrix
    std::vector<Eigen::Vector3d> map; ///< Map represented as a vector of feature vectors
    /**
     * @brief Updates the state vector according to the control.  
     * Formula 6 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * The modtion model used is the velocity motion model explained in probabalistic robotics (thrun et al.) in Formula 5.13. 
     * 
     * @param control control vector at time t
     * @return Eigen::Vector3d Updated state vector
     */
    Eigen::Vector3d motion_model(Eigen::Vector3d control);
    /**
     * @brief Calculate the jacobian G_t.
     * 
     * Formula 3 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * @param control control vector at time t
     * @return Eigen::Matrix3d 
     */
    Eigen::Matrix3d calculate_G(Eigen::Vector3d control);
    /**
     * @brief Calculate the jacobian V_t.
     * 
     * Formula 4 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * @param control control vector at time t
     * @return Eigen::Eigen::Matrix<double, 3, 2>
     */
    Eigen::Matrix<double, 3, 2> calculate_V(Eigen::Vector3d control);
    /**
     * @brief Calculate the covariance matrix M_t of the control vector.
     * 
     * Formula 5 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * @param control control vector at time t
     * @return Eigen::Matrix2d 
     */
    Eigen::Matrix2d calculate_M(float v, float omega);
    /**
     * @brief Calculates the covariance matrix of the state vector
     * 
     * Formula 7 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * @param G jacobian matrix
     * @param V jacobian matrix 
     * @param M covariance matrix
     * @return Eigen::Matrix3d updated covariance of the state vector
     */
    Eigen::Matrix3d calculate_sigma(Eigen::Matrix3d G, Eigen::Matrix<double, 3, 2> V, Eigen::Matrix2d M);
    /**
     * @brief Calculates the squared euclidean distance between landmark vector and state vector.
     * 
     * Formula 11 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * @note untested because only matrix operations are used 
     * @param landmark landmark vector in coordinate representation
     * @return double squared euclidean distance
     */
    double calculate_q(Eigen::Vector3d landmark);
    /**
     * @brief Calculates the jacobian H for the landmark
     * 
     * Formula 13 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * @param landmark landmark vector in coordinate representation
     * @return Eigen::Matrix3d 
     */
    Eigen::Matrix3d calculate_H(Eigen::Vector3d landmark);
    /**
     * @brief Calculates S (no explanation in probabalistic robotics --> black magic)
     * 
     * Formula 14 in Table 7.3 probabalistic robotics (thrun et al.).
     * 
     * @param H jacobian for the landmark
     * @return Eigen::Matrix3d 
     */
    Eigen::Matrix3d calculate_S(Eigen::Matrix3d H);
    /**
     * @brief Calculates Kalman gain k.
     * 
     * @param H jacobian for the landmark k
     * @param S  
     * @return Eigen::Matrix3d 
     */
    Eigen::Matrix3d calculate_K(Eigen::Matrix3d H, Eigen::Matrix3d S);
    /**
     * @brief Corrects the state based on the observation and landmark
     * 
     * @param K Kalman gain
     * @param z observation in range bearing representation
     * @param z_hat landmark k in range bearing representation
     * @return Eigen::Vector3d updated state vector
     */
    Eigen::Vector3d correct_state(Eigen::Matrix3d K, Eigen::Vector3d z, Eigen::Vector3d z_hat);
    /**
     * @brief Calculates the updated state covariance matrix.
     * 
     * @param K Kalman gain
     * @param H jacobian for the landmark k
     * @return Eigen::Matrix3d 
     */
    Eigen::Matrix3d calculate_sigma(Eigen::Matrix3d K, Eigen::Matrix3d H);
    /**
     * @brief Outlier rejection based on likelihood threshhold test.
     * 
     * @param likelihood likelihood of the state correction
     * @return true the correction is implausible and should be rejected
     * @return false the correction is plausible and should be accepted
     */
    bool outlier_rejection(double likelihood);
};

#endif  // EKF_LOCALIZATION__EKF_HPP_
