#include <gtest/gtest.h>
#include "ekf/kalman_filter.hpp"

using namespace ekf;

TEST(KalmanFilterTest, FilteringHighNoiseReducesCovariance) {
    KalmanFilter ekf;
    
    // 1D state: position
    Eigen::VectorXd x(1);
    x << 0.0;
    
    // Initial high uncertainty
    Eigen::MatrixXd P(1, 1);
    P << 1000.0;
    
    ekf.Init(x, P);
    
    // Process noise
    ekf.Q = Eigen::MatrixXd(1, 1);
    ekf.Q << 0.1;
    
    // Measurement noise (high)
    Eigen::MatrixXd R(1, 1);
    R << 10.0;
    
    // Non-linear state transition function (linear here for simplicity)
    auto f = [](const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
        Eigen::VectorXd next_state(1);
        next_state << state(0) + control(0);
        return next_state;
    };
    
    // Non-linear measurement function
    auto h = [](const Eigen::VectorXd& state) {
        Eigen::VectorXd z(1);
        z << state(0);
        return z;
    };
    
    // Jacobians (constant for linear model)
    Eigen::MatrixXd F_j(1, 1);
    F_j << 1.0;
    
    Eigen::MatrixXd H_j(1, 1);
    H_j << 1.0;
    
    // Control input
    Eigen::VectorXd u(1);
    u << 1.0; // move forward by 1 unit
    
    double initial_cov = ekf.P(0, 0);
    
    for (int i = 0; i < 50; ++i) {
        // Predict step
        ekf.Predict(f, u, F_j);
        
        // Measurement: true position is (i + 1), add some measurement noise
        Eigen::VectorXd z(1);
        z << static_cast<double>(i + 1) + 0.5; // noisy measurement
        
        // Update step
        ekf.Update(z, h, H_j, R);
    }
    
    double final_cov = ekf.P(0, 0);
    
    // The covariance should decrease over time
    EXPECT_LT(final_cov, initial_cov);
    
    // With Q=0.1, R=10, the steady state P should be around 0.95
    EXPECT_NEAR(final_cov, 0.95, 0.1);
}
