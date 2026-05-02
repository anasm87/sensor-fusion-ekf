#include "ekf/kalman_filter.hpp"

namespace ekf {

void KalmanFilter::Init(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance) {
    x = initial_state;
    P = initial_covariance;
}

void KalmanFilter::Predict(const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& f,
                           const Eigen::VectorXd& u,
                           const Eigen::MatrixXd& F_j) {
    F = F_j;
    x = f(x, u);
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::Update(const Eigen::VectorXd& z,
                          const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& h,
                          const Eigen::MatrixXd& H_j,
                          const Eigen::MatrixXd& R) {
    Eigen::VectorXd z_pred = h(x);
    Eigen::VectorXd y = z - z_pred;
    
    Eigen::MatrixXd Ht = H_j.transpose();
    Eigen::MatrixXd S = H_j * P * Ht + R;
    
    // Use robust solve instead of inverse for numerical stability
    // K = P * Ht * S^{-1}
    Eigen::MatrixXd K = P * Ht * S.llt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));

    // Update state
    x = x + (K * y);

    // Update covariance using Joseph form for better numerical stability
    // P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
    Eigen::Index x_size = x.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    Eigen::MatrixXd I_KH = I - K * H_j;
    P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
}

} // namespace ekf