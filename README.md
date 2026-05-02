# Extended Kalman Filter (EKF) for Sensor Fusion

A high-performance, numerically stable implementation of an Extended Kalman Filter (EKF) written in modern C++17. This repository serves as the core state-estimation module for a broader autonomous driving and sensor fusion architecture.

## Overview

Unlike standard textbook implementations, this EKF is engineered for production-grade numerical stability, specifically designed to prevent covariance divergence in high-dimensional state spaces and floating-point edge cases.

### Key Engineering Features
* **Joseph Form Covariance Update:** Utilizes the robust Joseph form for the measurement update step. This guarantees that the state covariance matrix $P$ remains positive semi-definite and symmetric, even in the presence of severe floating-point rounding errors.
* **Cholesky Decomposition:** Replaces direct matrix inversion with Eigen's LLT (Cholesky) decomposition `S.llt().solve()` when computing the Kalman Gain $K$. This provides superior numerical precision and computational efficiency.
* **Modern C++ Architecture:** Clean API utilizing `std::function` for injecting non-linear state transition and measurement functions.
* **Hermetic Build System:** Employs CMake `FetchContent` to automatically manage dependencies (Eigen3, Google Test), ensuring deterministic, zero-friction builds across Linux, macOS, and Windows.

## Mathematical Formulation

The filter operates iteratively via standard Prediction and Update phases, generalized for non-linear systems via Jacobians.

### 1. Predict
Projects the state and covariance forward in time using the non-linear transition function $f$ and its Jacobian $F$:

$$ x_{k|k-1} = f(x_{k-1|k-1}, u_k) $$
$$ P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k $$

### 2. Update
Fuses the incoming measurement $z_k$ using the non-linear measurement function $h$ and its Jacobian $H$. The Kalman Gain $K_k$ is solved via LLT decomposition:

$$ S_k = H_k P_{k|k-1} H_k^T + R_k $$
$$ K_k = P_{k|k-1} H_k^T S_k^{-1} $$
$$ x_{k|k} = x_{k|k-1} + K_k (z_k - h(x_{k|k-1})) $$

**Joseph Form Covariance Update:**
$$ P_{k|k} = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R_k K_k^T $$

## API Reference

The core functionality is encapsulated within the `ekf::KalmanFilter` class.

### Public Attributes

* `Eigen::VectorXd x`: The state vector $x$.
* `Eigen::MatrixXd P`: The state covariance matrix $P$.
* `Eigen::MatrixXd F`: The state transition matrix (Jacobian) $F$.
* `Eigen::MatrixXd Q`: The process noise covariance matrix $Q$.

### Methods

* `void Init(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance)`
  Initializes the state vector `x` and covariance `P`.

* `void Predict(const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& f, const Eigen::VectorXd& u, const Eigen::MatrixXd& F_j)`
  Performs the prediction step using the non-linear state transition function `f`, the control input `u`, and the computed Jacobian `F_j`.

* `void Update(const Eigen::VectorXd& z, const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& h, const Eigen::MatrixXd& H_j, const Eigen::MatrixXd& R)`
  Performs the measurement update step using the actual measurement `z`, non-linear measurement function `h`, Jacobian `H_j`, and the measurement noise covariance `R`.

## Usage Example

Below is a simplified 1D example demonstrating how to initialize and use the `KalmanFilter` class.

```cpp
#include "ekf/kalman_filter.hpp"
#include <iostream>

using namespace ekf;

int main() {
    KalmanFilter ekf;
    
    // 1. Initialize state and covariance
    Eigen::VectorXd x(1); x << 0.0;
    Eigen::MatrixXd P(1, 1); P << 1000.0; // High initial uncertainty
    ekf.Init(x, P);
    
    // 2. Set process noise covariance Q
    ekf.Q = Eigen::MatrixXd(1, 1);
    ekf.Q << 0.1;
    
    // Measurement noise covariance R
    Eigen::MatrixXd R(1, 1);
    R << 10.0;
    
    // 3. Define non-linear transition and measurement functions
    auto f = [](const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
        Eigen::VectorXd next_state(1);
        next_state << state(0) + control(0);
        return next_state;
    };
    
    auto h = [](const Eigen::VectorXd& state) {
        Eigen::VectorXd z(1);
        z << state(0);
        return z;
    };
    
    // Control input and Jacobians
    Eigen::VectorXd u(1); u << 1.0;
    Eigen::MatrixXd F_j(1, 1); F_j << 1.0;
    Eigen::MatrixXd H_j(1, 1); H_j << 1.0;
    
    // 4. Run Predict and Update Loop
    for (int i = 0; i < 50; ++i) {
        // Predict
        ekf.Predict(f, u, F_j);
        
        // Noisy measurement
        Eigen::VectorXd z(1);
        z << static_cast<double>(i + 1) + 0.5;
        
        // Update
        ekf.Update(z, h, H_j, R);
        
        std::cout << "Step " << i << " State: " << ekf.x(0) << "\n";
    }
    
    return 0;
}
```

## Directory Structure

```text
sensor-fusion-ekf/
├── CMakeLists.txt              # Build configuration
├── README.md                   # Project documentation
├── include/
│   └── ekf/
│       └── kalman_filter.hpp   # EKF class definition and headers
├── src/
│   └── kalman_filter.cpp       # Core EKF implementation
└── tests/
    └── test_kalman_filter.cpp  # GTest suite
```

## Build Instructions

This project requires **CMake (3.14+)** and a **C++17 compatible compiler** (MSVC, GCC, or Clang). Dependencies (Eigen3 and GTest) are downloaded automatically during configuration.

### Command Line Build
Open a terminal (e.g., Developer PowerShell on Windows) and run:

```bash
# 1. Generate build files
cmake -B build -S .

# 2. Compile the project
cmake --build build --config Release

# 3. Run the tests
cd build && ctest -C Release
```
