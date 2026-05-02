#ifndef EKF_KALMAN_FILTER_HPP_
#define EKF_KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace ekf {

class KalmanFilter {
public:
    // State vector
    Eigen::VectorXd x;

    // State covariance matrix
    Eigen::MatrixXd P;

    // State transition matrix (Jacobian)
    Eigen::MatrixXd F;

    // Process noise covariance matrix
    Eigen::MatrixXd Q;

    /**
     * @brief Constructor
     */
    KalmanFilter() = default;

    /**
     * @brief Initialize the filter
     * @param initial_state Initial state vector
     * @param initial_covariance Initial state covariance matrix
     */
    void Init(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance);

    /**
     * @brief Predict the state and the state covariance
     * @param f Non-linear state transition function: x_k = f(x_{k-1}, u_k)
     * @param u Control input
     * @param F_j Jacobian matrix of the state transition function
     */
    void Predict(const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& f,
                 const Eigen::VectorXd& u,
                 const Eigen::MatrixXd& F_j);

    /**
     * @brief Update the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     * @param h Non-linear measurement function: z_pred = h(x_pred)
     * @param H_j Jacobian matrix of the measurement function
     * @param R Measurement noise covariance
     */
    void Update(const Eigen::VectorXd& z,
                const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& h,
                const Eigen::MatrixXd& H_j,
                const Eigen::MatrixXd& R);
};

} // namespace ekf

#endif // EKF_KALMAN_FILTER_HPP_