#include "identification/algorithms.hpp"
#include "identification/optimizer.hpp"
#include "robot/robot_dynamics.hpp" // Need full definition for CLOE
#include <array>
#include <cmath>
#include <iostream>
#include <limits>

namespace identification {

// =============================================================================
// 正则化求解辅助函数 (Tikhonov/Ridge Regression)
// 正则化求解辅助函数 (Tikhonov/Ridge Regression)
// =============================================================================
namespace {
const double kRegularizationLambda = 1e-6;

inline Eigen::VectorXd solveRegularized(const Eigen::MatrixXd &W,
                                        const Eigen::VectorXd &tau) {
  // 使用正规方程 (Normal Equations) 方法。
  // 在剔除离群点后，矩阵不再极度病态，LDLT 速度快且有效。
  Eigen::MatrixXd WtW = W.transpose() * W;
  WtW +=
      kRegularizationLambda * Eigen::MatrixXd::Identity(WtW.rows(), WtW.cols());
  return WtW.ldlt().solve(W.transpose() * tau);
}

inline double clampPositive(double value, double min_value = 1e-6) {
  return std::max(value, min_value);
}

inline Eigen::VectorXd decodePositiveParameters(const Eigen::VectorXd &z) {
  return z.array().exp().matrix();
}

inline double safeSechSquared(double x) {
  const double c = std::cosh(x);
  const double inv = 1.0 / c;
  return inv * inv;
}
} // namespace

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
  if (type == "NLS_FRICTION")
    return std::make_unique<NonlinearFrictionLM>(dof);

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
// OLS (with Tikhonov Regularization / Ridge Regression)
// =============================================================================
Eigen::VectorXd OLS::solve(const Eigen::MatrixXd &W,
                           const Eigen::VectorXd &Tau_meas) {
  // 使用 SVD 直接求解 OLS
  // 这比正规方程 (Normal Equations) 方法更慢但数值上更稳定，
  // 使用正则化求解以防止参数数值爆炸
  // (即使数据已过滤，某些参数仍可能不可辨识导致秩亏)
  return solveRegularized(W, Tau_meas);
}

// =============================================================================
// WLS
// =============================================================================
WLS::WLS(int dof) : dof_(dof) {}

Eigen::VectorXd WLS::solve(const Eigen::MatrixXd &W,
                           const Eigen::VectorXd &Tau_meas) {
  // 使用正则化 OLS 计算初始估计
  Eigen::VectorXd beta_ols = solveRegularized(W, Tau_meas);
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

  // 使用正则化求解加权最小二乘
  return solveRegularized(W_weighted, Tau_weighted);
}

// =============================================================================
// IRLS
// =============================================================================
IRLS::IRLS(int max_iter, double tol) : max_iter_(max_iter), tol_(tol) {}

Eigen::VectorXd IRLS::solve(const Eigen::MatrixXd &W,
                            const Eigen::VectorXd &Tau_meas) {
  // 使用正则化 OLS 初始化
  Eigen::VectorXd beta = solveRegularized(W, Tau_meas);
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
    // 使用正则化求解
    Eigen::VectorXd beta_new = solveRegularized(W_w, Tau_w);
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
    return solveRegularized(W, Tau_meas);
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
// NonlinearFrictionLM
// =============================================================================
NonlinearFrictionLM::NonlinearFrictionLM(int dof, int multi_start)
    : dof_(dof), multi_start_(multi_start) {}

void NonlinearFrictionLM::setVelocityData(const Eigen::MatrixXd &qd_meas) {
  qd_meas_ = qd_meas;
}

Eigen::VectorXd
NonlinearFrictionLM::predictTorques(const Eigen::MatrixXd &W_base,
                                    const Eigen::MatrixXd &qd_meas,
                                    const Eigen::VectorXd &params, int dof) {
  const Eigen::Index linear_count = W_base.cols();
  const Eigen::Index friction_count =
      static_cast<Eigen::Index>(frictionParameterCount(dof));

  if (params.size() != linear_count + friction_count) {
    throw std::runtime_error("NLS_FRICTION 参数维度与观测矩阵不匹配");
  }
  if (qd_meas.cols() != dof ||
      W_base.rows() != qd_meas.rows() * static_cast<Eigen::Index>(dof)) {
    throw std::runtime_error("NLS_FRICTION 速度矩阵维度不匹配");
  }

  Eigen::VectorXd tau_pred = W_base * params.head(linear_count);
  const Eigen::VectorXd friction_params = params.tail(friction_count);

  for (Eigen::Index sample = 0; sample < qd_meas.rows(); ++sample) {
    for (int joint = 0; joint < dof; ++joint) {
      const Eigen::Index base = static_cast<Eigen::Index>(joint * 6);
      const double g1 = friction_params(base + 0);
      const double g2 = friction_params(base + 1);
      const double g3 = friction_params(base + 2);
      const double g4 = friction_params(base + 3);
      const double g5 = friction_params(base + 4);
      const double g6 = friction_params(base + 5);
      const double v = qd_meas(sample, joint);

      const double friction =
          g1 * (std::tanh(g2 * v) - std::tanh(g3 * v)) +
          g4 * std::tanh(g5 * v) + g6 * v;

      tau_pred(sample * dof + joint) -= friction;
    }
  }

  return tau_pred;
}

Eigen::VectorXd NonlinearFrictionLM::solve(const Eigen::MatrixXd &W_base,
                                           const Eigen::VectorXd &Tau_meas) {
  if (qd_meas_.rows() == 0) {
    throw std::runtime_error("NLS_FRICTION 缺少速度数据");
  }
  if (qd_meas_.cols() != dof_) {
    throw std::runtime_error("NLS_FRICTION 速度自由度不匹配");
  }
  if (W_base.rows() != Tau_meas.size() ||
      W_base.rows() != qd_meas_.rows() * dof_) {
    throw std::runtime_error("NLS_FRICTION 训练数据维度不一致");
  }

  const Eigen::Index linear_count = W_base.cols();
  const Eigen::Index friction_count =
      static_cast<Eigen::Index>(frictionParameterCount(dof_));

  const Eigen::VectorXd beta_linear = solveRegularized(W_base, Tau_meas);
  const Eigen::VectorXd residual_linear = Tau_meas - W_base * beta_linear;

  Eigen::VectorXd friction_init = Eigen::VectorXd::Zero(friction_count);
  for (int joint = 0; joint < dof_; ++joint) {
    const Eigen::Index base = static_cast<Eigen::Index>(joint * 6);
    double residual_abs_mean = 0.0;
    double residual_abs_low_speed = 0.0;
    double low_speed_count = 0.0;
    double viscous_numerator = 0.0;
    double viscous_denominator = 0.0;

    for (Eigen::Index sample = 0; sample < qd_meas_.rows(); ++sample) {
      const Eigen::Index idx = sample * dof_ + joint;
      const double v = qd_meas_(sample, joint);
      const double r = residual_linear(idx);
      residual_abs_mean += std::abs(r);
      if (std::abs(v) < 0.2) {
        residual_abs_low_speed += std::abs(r);
        low_speed_count += 1.0;
      }
      viscous_numerator += -v * r;
      viscous_denominator += v * v;
    }

    residual_abs_mean /= std::max<Eigen::Index>(qd_meas_.rows(), 1);
    const double plateau =
        low_speed_count > 0.0 ? residual_abs_low_speed / low_speed_count
                              : residual_abs_mean;
    const double viscous_guess =
        viscous_denominator > 1e-8 ? viscous_numerator / viscous_denominator
                                   : 0.05;

    friction_init(base + 0) = clampPositive(0.3 * plateau + 1e-3);
    friction_init(base + 1) = 12.0;
    friction_init(base + 2) = 3.0;
    friction_init(base + 3) = clampPositive(0.7 * plateau + 1e-3);
    friction_init(base + 4) = 12.0;
    friction_init(base + 5) = clampPositive(std::abs(viscous_guess), 1e-3);
  }

  Eigen::VectorXd best_params(linear_count + friction_count);
  double best_error = std::numeric_limits<double>::infinity();

  const std::array<double, 4> start_scales = {1.0, 0.7, 1.4, 2.0};
  const int start_count =
      std::min<int>(multi_start_, static_cast<int>(start_scales.size()));
  std::vector<Eigen::VectorXd> candidate_params(
      static_cast<std::size_t>(start_count),
      Eigen::VectorXd::Zero(linear_count + friction_count));
  std::vector<double> candidate_errors(static_cast<std::size_t>(start_count),
                                       std::numeric_limits<double>::infinity());

#ifdef IDENTIFICATION_USE_OPENMP
#pragma omp parallel for schedule(static) if (start_count > 1)
#endif
  for (int start = 0; start < start_count; ++start) {
    Eigen::VectorXd x_init(linear_count + friction_count);
    x_init.head(linear_count) = beta_linear;
    for (Eigen::Index i = 0; i < friction_count; ++i) {
      const double scaled =
          clampPositive(friction_init(i) * start_scales[static_cast<std::size_t>(start)]);
      x_init(linear_count + i) = std::log(scaled);
    }

    LevenbergMarquardt::ResidualFunction residuals =
        [this, &W_base, &Tau_meas, linear_count,
         friction_count](const Eigen::VectorXd &x) {
          Eigen::VectorXd params(linear_count + friction_count);
          params.head(linear_count) = x.head(linear_count);
          params.tail(friction_count) =
              decodePositiveParameters(x.tail(friction_count));
          Eigen::VectorXd residual =
              predictTorques(W_base, qd_meas_, params, dof_);
          residual -= Tau_meas;
          return residual;
        };

    LevenbergMarquardt::JacobianFunction jacobian =
        [this, &W_base, linear_count,
         friction_count](const Eigen::VectorXd &x) {
          Eigen::MatrixXd J = Eigen::MatrixXd::Zero(
              W_base.rows(), linear_count + friction_count);
          J.leftCols(linear_count) = W_base;

          const Eigen::VectorXd friction_params =
              decodePositiveParameters(x.tail(friction_count));

          for (Eigen::Index sample = 0; sample < qd_meas_.rows(); ++sample) {
            for (int joint = 0; joint < dof_; ++joint) {
              const Eigen::Index row = sample * dof_ + joint;
              const Eigen::Index param_base =
                  linear_count + static_cast<Eigen::Index>(joint * 6);
              const Eigen::Index friction_base =
                  static_cast<Eigen::Index>(joint * 6);
              const double g1 = friction_params(friction_base + 0);
              const double g2 = friction_params(friction_base + 1);
              const double g3 = friction_params(friction_base + 2);
              const double g4 = friction_params(friction_base + 3);
              const double g5 = friction_params(friction_base + 4);
              const double g6 = friction_params(friction_base + 5);
              const double v = qd_meas_(sample, joint);

              const double t2 = std::tanh(g2 * v);
              const double t3 = std::tanh(g3 * v);
              const double t5 = std::tanh(g5 * v);

              J(row, param_base + 0) = -g1 * (t2 - t3);
              J(row, param_base + 1) = -g1 * safeSechSquared(g2 * v) * g2 * v;
              J(row, param_base + 2) = g1 * safeSechSquared(g3 * v) * g3 * v;
              J(row, param_base + 3) = -g4 * t5;
              J(row, param_base + 4) = -g4 * safeSechSquared(g5 * v) * g5 * v;
              J(row, param_base + 5) = -g6 * v;
            }
          }

          return J;
        };

    LevenbergMarquardt::Options options;
    options.max_iter = 80;
    options.lambda_init = 1e-2;
    options.tol_delta = 1e-7;
    options.tol_error = 1e-7;

    LevenbergMarquardt solver(residuals, jacobian, options);
    const Eigen::VectorXd x_opt = solver.minimize(x_init);

    Eigen::VectorXd params(linear_count + friction_count);
    params.head(linear_count) = x_opt.head(linear_count);
    params.tail(friction_count) =
        decodePositiveParameters(x_opt.tail(friction_count));

    Eigen::VectorXd residual = predictTorques(W_base, qd_meas_, params, dof_);
    residual -= Tau_meas;
    const double error = residual.squaredNorm();
    candidate_errors[static_cast<std::size_t>(start)] = error;
    candidate_params[static_cast<std::size_t>(start)] = std::move(params);
  }

  for (int start = 0; start < start_count; ++start) {
    const double error = candidate_errors[static_cast<std::size_t>(start)];
    if (error < best_error) {
      best_error = error;
      best_params = candidate_params[static_cast<std::size_t>(start)];
    }
  }

  return best_params;
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
                            const Eigen::VectorXd &Tau_meas) {
  if (q_meas_.rows() == 0) {
    std::cerr << "CLOE Error: Experiment data not set!" << std::endl;
    // Fallback to OLS using W and passed Tau_meas (which matches W dimensions)
    return W.colPivHouseholderQr().solve(Tau_meas);
  }

  // Initial guess using OLS with the passed Tau_meas (matches W dimensions)
  Eigen::VectorXd beta = W.colPivHouseholderQr().solve(Tau_meas);

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
