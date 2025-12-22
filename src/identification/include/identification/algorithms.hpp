#ifndef IDENTIFICATION_ALGORITHMS_HPP_
#define IDENTIFICATION_ALGORITHMS_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

// Forward declarations
namespace robot {
class RobotDynamics;
}

namespace identification {

/**
 * @brief Base class for identification algorithms
 */
class IdentificationAlgorithm {
public:
  virtual ~IdentificationAlgorithm() = default;

  /**
   * @brief Solve the identification problem: W * beta = Tau
   *
   * @param W Observation matrix (N*K x M)
   * @param Tau_meas Measured torque vector (N*K x 1)
   * @return Eigen::VectorXd Identified parameters beta (M x 1)
   */
  virtual Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                                const Eigen::VectorXd &Tau_meas) = 0;
};

/**
 * @brief Ordinary Least Squares
 * solution = (W^T W)^-1 W^T Tau
 */
class OLS : public IdentificationAlgorithm {
public:
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;
};

/**
 * @brief Weighted Least Squares
 * solution = (W^T Sigma^-1 W)^-1 W^T Sigma^-1 Tau
 */
class WLS : public IdentificationAlgorithm {
public:
  explicit WLS(int dof);
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;

private:
  int dof_;
};

/**
 * @brief Iteratively Reweighted Least Squares
 * Robust regression using Huber weights
 */
class IRLS : public IdentificationAlgorithm {
public:
  explicit IRLS(int max_iter = 50, double tol = 1e-4);
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;

private:
  int max_iter_;
  double tol_;
};

/**
 * @brief Total Least Squares
 * Minimizes error in both W and Tau using SVD
 */
class TLS : public IdentificationAlgorithm {
public:
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;
};

/**
 * @brief Recursive Least Squares / Extended Kalman Filter
 * Ideal for online identification
 */
class EKF : public IdentificationAlgorithm {
public:
  explicit EKF(int n_params);
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;

private:
  int n_params_;
};

/**
 * @brief Closed-Loop Output Error
 * Minimizes position error by simulating forward dynamics.
 */
class CLOE : public IdentificationAlgorithm {
public:
  // We need the robot dynamics module to simulate forward dynamics
  CLOE(const robot::RobotDynamics &dynamics, int dof, double dt);
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;

  // Set the measured Q, QD, and Tau (needed for simulation, not just W)
  void setExperimentData(const Eigen::MatrixXd &q_meas,
                         const Eigen::MatrixXd &qd_meas,
                         const Eigen::MatrixXd &tau_meas);

private:
  const robot::RobotDynamics &dynamics_;
  int dof_;
  double dt_;
  Eigen::MatrixXd q_meas_;
  Eigen::MatrixXd qd_meas_;
  Eigen::MatrixXd tau_meas_;
};

/**
 * @brief Maximum Likelihood (simplified)
 * Uses Levenberg-Marquardt to minimize force error (Non-linear LS)
 */
class ML : public IdentificationAlgorithm {
public:
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;
};

// Factory
std::unique_ptr<IdentificationAlgorithm>
createAlgorithm(const std::string &type, int dof = 7);

} // namespace identification

#endif // IDENTIFICATION_ALGORITHMS_HPP_
