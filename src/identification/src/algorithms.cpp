#include "identification/algorithms.hpp"
#include "identification/optimizer.hpp"
#include "robot/robot_dynamics.hpp" // Need full definition for CLOE
#include <cmath>
#include <iostream>

namespace identification {

// =============================================================================
// Factory
// =============================================================================
// Note: CLOE requires extra init, so if created via factory, it needs casting
// later to set data. This is a slight design smell but acceptable for
// migration.
std::unique_ptr<IdentificationAlgorithm>
createAlgorithm(const std::string &type, int dof) {
  if (type == "OLS")
    return std::make_unique<OLS>();
  if (type == "WLS")
    return std::make_unique<WLS>(dof);
  if (type == "IRLS")
    return std::make_unique<IRLS>();
  if (type == "TLS")
    return std::make_unique<TLS>();
  if (type == "EKF") {
    // Basic params size estimate: 10 params per link * dof?
    // Actually standard inertial params is usually 10*N.
    // But 'W' columns tells us M. We don't know M yet.
    // EKF constructor will resize in solve.
    return std::make_unique<EKF>(0);
  }
  if (type == "ML")
    return std::make_unique<ML>();

  // CLOE cannot be fully created here without dynamics reference.
  // We will handle CLOE specially in Identification class or assume factory
  // is only for simple ones.
  // For now, return nullptr for CLOE to indicate special handling needed.
  if (type == "CLOE")
    return nullptr;

  std::cerr << "Unknown algorithm type: " << type << ". Defaulting to OLS."
            << std::endl;
  return std::make_unique<OLS>();
}

// =============================================================================
// OLS
// =============================================================================
Eigen::VectorXd OLS::solve(const Eigen::MatrixXd &W,
                           const Eigen::VectorXd &Tau_meas) {
  return W.colPivHouseholderQr().solve(Tau_meas);
}

// =============================================================================
// WLS
// =============================================================================
WLS::WLS(int dof) : dof_(dof) {}

Eigen::VectorXd WLS::solve(const Eigen::MatrixXd &W,
                           const Eigen::VectorXd &Tau_meas) {
  Eigen::VectorXd beta_ols = W.colPivHouseholderQr().solve(Tau_meas);
  Eigen::VectorXd residual = Tau_meas - W * beta_ols;

  int n_samples = Tau_meas.size() / dof_;
  Eigen::VectorXd sigma_sq = Eigen::VectorXd::Zero(dof_);

  for (int j = 0; j < dof_; ++j) {
    double sum_sq_err = 0.0;
    for (int i = 0; i < n_samples; ++i) {
      int idx = i * dof_ + j;
      sum_sq_err += residual(idx) * residual(idx);
    }
    sigma_sq(j) = sum_sq_err / n_samples;
    if (sigma_sq(j) < 1e-9)
      sigma_sq(j) = 1e-9;
  }

  Eigen::MatrixXd W_weighted = W;
  Eigen::VectorXd Tau_weighted = Tau_meas;

  for (int i = 0; i < n_samples; ++i) {
    for (int j = 0; j < dof_; ++j) {
      int idx = i * dof_ + j;
      double weight = 1.0 / std::sqrt(sigma_sq(j));
      W_weighted.row(idx) *= weight;
      Tau_weighted(idx) *= weight;
    }
  }

  return W_weighted.colPivHouseholderQr().solve(Tau_weighted);
}

// =============================================================================
// IRLS
// =============================================================================
IRLS::IRLS(int max_iter, double tol) : max_iter_(max_iter), tol_(tol) {}

Eigen::VectorXd IRLS::solve(const Eigen::MatrixXd &W,
                            const Eigen::VectorXd &Tau_meas) {
  Eigen::VectorXd beta = W.colPivHouseholderQr().solve(Tau_meas);
  for (int k = 0; k < max_iter_; ++k) {
    Eigen::VectorXd residual = Tau_meas - W * beta;
    Eigen::VectorXd weights(residual.size());

    std::vector<double> abs_res(residual.size());
    for (int i = 0; i < residual.size(); ++i)
      abs_res[i] = std::abs(residual(i));
    std::sort(abs_res.begin(), abs_res.end());
    double median_res = abs_res[abs_res.size() / 2];
    double sigma = median_res / 0.6745;
    double delta = 1.345 * sigma;
    if (delta < 1e-6)
      delta = 1e-6;

    for (int i = 0; i < residual.size(); ++i) {
      double r = std::abs(residual(i));
      weights(i) = (r <= delta) ? 1.0 : (delta / r);
    }

    Eigen::MatrixXd W_w = W;
    Eigen::VectorXd Tau_w = Tau_meas;
    for (int i = 0; i < W.rows(); ++i) {
      double w_sqrt = std::sqrt(weights(i));
      W_w.row(i) *= w_sqrt;
      Tau_w(i) *= w_sqrt;
    }
    Eigen::VectorXd beta_new = W_w.colPivHouseholderQr().solve(Tau_w);
    if ((beta_new - beta).norm() < tol_) {
      beta = beta_new;
      break;
    }
    beta = beta_new;
  }
  return beta;
}

// =============================================================================
// TLS
// =============================================================================
Eigen::VectorXd TLS::solve(const Eigen::MatrixXd &W,
                           const Eigen::VectorXd &Tau_meas) {
  long n_cols = W.cols();
  Eigen::MatrixXd Z(W.rows(), n_cols + 1);
  Z << W, Tau_meas;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Z, Eigen::ComputeThinV);
  Eigen::VectorXd v_last = svd.matrixV().col(n_cols);
  if (std::abs(v_last(n_cols)) < 1e-9)
    return W.colPivHouseholderQr().solve(Tau_meas);
  return -v_last.head(n_cols) / v_last(n_cols);
}

// =============================================================================
// EKF
// =============================================================================
EKF::EKF(int n_params) : n_params_(n_params) {}

Eigen::VectorXd EKF::solve(const Eigen::MatrixXd &W,
                           const Eigen::VectorXd &Tau_meas) {
  // EKF as Recursive Least Squares for static parameters
  // x_k = beta (state)
  // y_k = W_k * x_k + v_k

  int n_params = W.cols(); // Determine params from matrix
  int total_rows = W.rows();
  // Assuming W rows are correlated to time steps.
  // Tau is stacked [tau(t1); tau(t2)...]
  // The "measurement dimension" at each step is DOF? Or can we process
  // row-by-row? Processing row-by-row (scalar update) is efficient for RLS.

  Eigen::VectorXd beta = Eigen::VectorXd::Zero(n_params);
  Eigen::MatrixXd P =
      100.0 *
      Eigen::MatrixXd::Identity(n_params, n_params); // Large initial covariance
  double R = 0.1; // Measurement noise covariance (scalar)

  // We process each row as a scalar measurement
  for (int i = 0; i < total_rows; ++i) {
    // H = row of W
    Eigen::RowVectorXd H = W.row(i);
    double y = Tau_meas(i);

    // Prediction (static model)
    // x_pred = x_prev
    // P_pred = P_prev + Q (Q=0 for static)

    // Update
    // K = P * H' / (H * P * H' + R)
    // Since H is row vector, H*P*H' is scalar
    double S = (H * P * H.transpose())(0, 0) + R;
    Eigen::VectorXd K = P * H.transpose() / S;

    double error = y - H * beta;
    beta = beta + K * error;
    P = (Eigen::MatrixXd::Identity(n_params, n_params) - K * H) * P;
  }

  return beta;
}

// =============================================================================
// ML (using LevenbergMarquardt)
// =============================================================================
Eigen::VectorXd ML::solve(const Eigen::MatrixXd &W,
                          const Eigen::VectorXd &Tau_meas) {
  // Basic ML/Non-linear LS: Minimize || W*beta - Tau ||
  // Ideally this is same as OLS for linear.
  // But for demo, we use the non-linear solver.

  int n_params = W.cols();

  LevenbergMarquardt::ResidualFunction residuals =
      [&](const Eigen::VectorXd &beta) { return W * beta - Tau_meas; };

  LevenbergMarquardt::JacobianFunction jacobian =
      [&](const Eigen::VectorXd & /*beta*/) {
        return W; // Jacobian is constant W for linear model
      };

  LevenbergMarquardt solver(residuals, jacobian);
  // Initial guess
  Eigen::VectorXd beta_init = Eigen::VectorXd::Zero(n_params);
  return solver.minimize(beta_init);
}

// =============================================================================
// CLOE
// =============================================================================
CLOE::CLOE(const robot::RobotDynamics &dynamics, int dof, double dt)
    : dynamics_(dynamics), dof_(dof), dt_(dt) {}

void CLOE::setExperimentData(const Eigen::MatrixXd &q_meas,
                             const Eigen::MatrixXd &qd_meas,
                             const Eigen::MatrixXd &tau_meas) {
  q_meas_ = q_meas;
  qd_meas_ = qd_meas;
  tau_meas_ = tau_meas;
}

Eigen::VectorXd CLOE::solve(const Eigen::MatrixXd &W,
                            const Eigen::VectorXd & /*Tau_meas*/) {
  if (q_meas_.rows() == 0) {
    std::cerr << "CLOE Error: Experiment data not set!" << std::endl;
    // Fallback to OLS using W
    return W.colPivHouseholderQr().solve(tau_meas_.reshaped().eval());
  }

  // Initial guess using OLS
  Eigen::VectorXd beta =
      W.colPivHouseholderQr().solve(tau_meas_.reshaped().eval());

  // NOTE: Implementing full CLOE with forward dynamics inside LM is
  // computationally heavy and requires mapping 'beta' back to robot model
  // parameters to update matrices M, C, G. Currently, our RobotDynamics takes
  // 'RobotModel' which is const. To support this, we would need to update
  // RobotModel from beta vector. This is significant work: mapping standard
  // parameters [m, mc, I...] to Model. Given scope, we will implement a stub
  // that warns user or does a simplified version.

  // Simplification: Output CLOE currently not supported due to Model constness.
  // We return OLS result with a warning.
  std::cerr
      << "WARNING: CLOE requires updating RobotModel from parameters, "
      << "which is not yet supported in C++ structure. Returning OLS result."
      << std::endl;

  return beta;
}

} // namespace identification
