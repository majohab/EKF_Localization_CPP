#include "ekf_localization/ekf.hpp"

Ekf::Ekf()
{
    // Q_t stays the init sigma 
    this->Q_t << std::pow(1, 2), 0, 0,
                 0, std::pow(1, 2), 0,
                 0, 0, std::pow(0.05, 2);
    // alpha also stays the same
    this->alpha << 0.0001, 0.0001, 0.0001, 0.0001;
    //set initial sigma
    this->sigma << std::pow(1, 2), 0, 0,
                   0, std::pow(1, 2), 0,
                   0, 0, std::pow(0.05, 2);
    // set init flag
    this->init_flag = false;
}

Ekf::Ekf(Eigen::Vector3d init_sigma, Eigen::Vector4d alpha, std::vector<Eigen::Vector3d> map)
{
    // Q_t stays the init sigma 
    this->Q_t << std::pow(init_sigma[0], 2), 0, 0,
                 0, std::pow(init_sigma[1], 2), 0,
                 0, 0, std::pow(init_sigma[2], 2);
    // alpha also stays the same
    this->alpha << alpha[0], alpha[1], alpha[2], alpha[3];
    //set initial sigma
    this->sigma << init_sigma[0], 0, 0,
                   0, init_sigma[1], 0,
                   0, 0, init_sigma[2];
    //set map
    this->map = map;
    // set init flag 
    this->init_flag = true;
}

Ekf::~Ekf()
{
}

Eigen::Vector3d Ekf::motion_model(Eigen::Vector3d control)
{
    Eigen::Vector3d updated_state;
    if (std::abs(control[1]) > this->EPS)
    {
        // extract control data from msg
        double updated_theta = this->mu[2] + control[1] * control[2];
        double quot_v_omega = control[0] / control[1];
        //motion update
        updated_state[0] = this->mu[0] + quot_v_omega * (-std::sin(this->mu[2]) + std::sin(updated_theta));
        updated_state[1] = this->mu[1] + quot_v_omega * (std::cos(this->mu[2]) - std::cos(updated_theta));
        updated_state[2] = this->mu[2] + control[1] * control[2];
    } else {
        //motion update
        updated_state[0] = this->mu[0] + control[0] * std::cos(this->mu[2]) * control[2];
        updated_state[1] = this->mu[1] + control[0] * std::sin(this->mu[2]) * control[2];
        updated_state[2] = this->mu[2] + 0;
    }
    return updated_state;
}

Eigen::Matrix3d Ekf::calculate_G(Eigen::Vector3d control)
{
    double quot_v_omega = control[0] / control[1];
    double updated_theta = this->mu[2] + control[1] * control[2];
    double G_0_2 = quot_v_omega * (-std::cos(this->mu[2]) + std::cos(updated_theta));
    double G_1_2 = quot_v_omega * (-std::sin(this->mu[2]) + std::sin(updated_theta));
    Eigen::Matrix3d G;
    G << 1, 0, G_0_2,
         0, 1, G_1_2,
         0, 0, 1;
    return G;
}

Eigen::Matrix<double, 3, 2> Ekf::calculate_V(Eigen::Vector3d control)
{
    double updated_theta = this->mu[2] + control[1] * control[2];
    double V_0_0 = (-std::sin(this->mu[2])+std::sin(updated_theta)) / control[1];
    double V_1_0 = (std::cos(this->mu[2])-std::cos(updated_theta)) / control[1];
    double V_0_1 = control[0]*(std::sin(this->mu[2])-std::sin(updated_theta)) / std::pow(control[1], 2) + control[0] * std::cos(updated_theta) * control[2] / control[1];
    double V_1_1 = -control[0]*(std::cos(this->mu[2])-std::cos(updated_theta)) / std::pow(control[1], 2) + control[0] * std::sin(updated_theta) * control[2] / control[1];
    Eigen::Matrix<double, 3, 2> V;
    V << V_0_0, V_0_1,
         V_1_0, V_1_1,
         0, control[2];
    return V; 
}

Eigen::Matrix2d Ekf::calculate_M(float v, float omega)
{
    double M_0_0 = this->alpha[0]*std::pow(v, 2) + this->alpha[1]*std::pow(omega, 2);
    double M_1_1 = this->alpha[2]*std::pow(v, 2) + this->alpha[3]*std::pow(omega, 2);
    Eigen::Matrix2d M;
    M << M_0_0, 0,
         0,     M_1_1;
    return M;
}

Eigen::Matrix3d Ekf::calculate_sigma(Eigen::Matrix3d G, Eigen::Matrix<double, 3, 2> V, Eigen::Matrix2d M)
{
    Eigen::Matrix3d updated_sigma = G * this->sigma * G.transpose() + V * M * V.transpose();
    return updated_sigma;
}

void Ekf::motion_update(Eigen::Vector3d control)
{
    if(!this->init_flag) return; // Catch uninitialized case
    // update the state based on the motion model
    Eigen::Vector3d updated_state = this->motion_model(control);
    // calculate the matrices for the covariance update
    Eigen::Matrix3d G = this->calculate_G(control);
    Eigen::Matrix<double, 3, 2> V = this->calculate_V(control);
    Eigen::Matrix2d M = this->calculate_M(control[0], control[2]);
    // calculate the updated sigma
    Eigen::Matrix3d updated_sigma = this->calculate_sigma(G, V, M);
    // update the internal values
    this->mu = updated_state;
    this->sigma = updated_sigma;
}

double Ekf::calculate_q(Eigen::Vector3d landmark)
{
    return std::pow(landmark[0]-this->mu[0], 2)+std::pow(landmark[1]-this->mu[1], 2);
}

Eigen::Vector3d Ekf::calculate_z_hat(Eigen::Vector3d landmark)
{
    double r = sqrt(calculate_q(landmark));
    double phi = atan2(landmark[1]-this->mu[1], landmark[0]-this->mu[0]) - this->mu[2];
    Eigen::Vector3d z_hat;
    z_hat << r, phi, landmark[2];
    return z_hat;
}

Eigen::Matrix3d Ekf::calculate_H(Eigen::Vector3d landmark)
{
    double diff_mx_mux = landmark[0] - this->mu[0];
    double diff_my_muy = landmark[1] - this->mu[1];
    double q = this->calculate_q(landmark);
    double H_0_0 = - diff_mx_mux / sqrt(q);
    double H_0_1 = - diff_my_muy / sqrt(q);
    double H_1_0 = diff_my_muy / q;
    double H_1_1 = - diff_mx_mux / q;
    Eigen::Matrix3d H;
    H << H_0_0, H_0_1,  0,
         H_1_0, H_1_1, -1,
         0,     0,      0;
    return H;
}

Eigen::Matrix3d Ekf::calculate_S(Eigen::Matrix3d H)
{
    Eigen::Matrix3d S = H * this->sigma * H.transpose() + this->Q_t;
    return S;
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d, double> Ekf::data_association(Eigen::Vector3d observation)
{
    std::vector<Eigen::Vector3d>::iterator iter = this->map.begin();   
    // run for loop from 0 to vecSize
    std::vector<double> j;
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d, double>> c;
    Eigen::Matrix3d S, H;
    Eigen::Vector3d z_hat;
    double argument;
    for(iter; iter < this->map.end(); iter++)
    {
        z_hat = calculate_z_hat(*iter);
        H = calculate_H(*iter);
        S = calculate_S(H);
        argument = std::pow((2 * M_PI * S).determinant(), -0.5) * exp(-0.5 * (observation-z_hat).transpose() * S.inverse() * (observation - z_hat));
        j.push_back(argument);
        c.push_back(std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d, double> {z_hat, S, H, argument});
    }
    std::vector<double>::iterator result = std::max_element(j.begin(), j.end());
    int argmax = std::distance(j.begin(), result);
    return c[argmax];
}

Eigen::Matrix3d Ekf::calculate_K(Eigen::Matrix3d H, Eigen::Matrix3d S)
{
    Eigen::Matrix3d K = this->sigma * H.transpose() * S.inverse();
    return K;
}

Eigen::Vector3d Ekf::correct_state(Eigen::Matrix3d K, Eigen::Vector3d z, Eigen::Vector3d z_hat)
{
    Eigen::Vector3d updated_state = this->mu + K * (z - z_hat);
    return updated_state;
}

Eigen::Matrix3d Ekf::calculate_sigma(Eigen::Matrix3d K, Eigen::Matrix3d H)
{
    Eigen::Matrix3d sigma = (Eigen::Matrix3d::Identity() - K * H) * this->sigma;
    return sigma;
}

bool Ekf::outlier_rejection(double likelihood)
{
    return true;
}

void Ekf::correction_step(std::vector<Eigen::Vector3d> observations)
{
    if(!this->init_flag) return; // Catch uninitialized case
    std::vector<Eigen::Vector3d>::iterator iter = observations.begin();
    //save state and cov in case of outlier rejection
    Eigen::Vector3d pre_correction_state = this->mu;
    Eigen::Matrix3d pre_correction_covariance = this->sigma;
    // correction step
    std::vector<double> p_zit;
    Eigen::Matrix3d K;
    for(iter; iter < this->map.end(); iter++)
    {
        auto [z_hat, S, H, p_i] = data_association(*iter);
        K = calculate_K(H, S);
        this->mu = correct_state(K, *iter, z_hat);
        this->sigma = calculate_sigma(K, H);
        p_zit.push_back(p_i);
    }
    double p_zt = std::accumulate(p_zit.begin(), p_zit.end(), 1, std::multiplies<double>());
    if(outlier_rejection(p_zt))
    {
        //reject the updated states and revert to old state
        this->mu = pre_correction_state;
        this->sigma = pre_correction_covariance;
    }
}

void Ekf::set_sigma(Eigen::Vector3d sigma)
{
    this->Q_t << std::pow(sigma[0], 2), 0, 0,
                 0, std::pow(sigma[1], 2), 0,
                 0, 0, std::pow(sigma[2], 2);
    this->sigma << std::pow(sigma[0], 2), 0, 0,
                   0, std::pow(sigma[1], 2), 0,
                   0, 0, std::pow(sigma[2], 2);
}

void Ekf::set_alpha(Eigen::Vector4d alpha)
{
    this->alpha = alpha;
}

void Ekf::set_map(std::vector<Eigen::Vector3d> map)
{
    this->map = map;
}

std::vector<Eigen::Vector3d> Ekf::get_map()
{
    return this->map;
}

Eigen::Vector3d Ekf::get_state()
{
    return this->mu;
}

void Ekf::set_state(Eigen::Vector3d state_vector)
{
    this->mu = state_vector;
}

std::vector<Eigen::Vector3d> Ekf::get_observable_landmarks()
{
    double theta[2] = {std::cos(this->mu[2]), std::sin(this->mu[2])};
    double perpendicular[2] = {-theta[1], theta[0]};
    // select landmarks in front of the robot in the MAPRANGE
    std::vector<Eigen::Vector3d> observable_landmarks;
    for (const Eigen::Vector3d &landmark : this->map) {
        double det = perpendicular[0] * (landmark[1] - this->mu[1]) - perpendicular[1] * (landmark[0] - this->mu[0]);
        if (det < 0) {
            double dist = std::sqrt(this->calculate_q(landmark));
            if (dist < this->MAPRANGE) {
                observable_landmarks.push_back(landmark);
            }
        }
    }
    return observable_landmarks;
}
