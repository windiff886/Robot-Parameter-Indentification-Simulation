#ifndef IDENTIFICATION_OPTIMIZER_HPP_
#define IDENTIFICATION_OPTIMIZER_HPP_

#include <Eigen/Dense>
#include <functional>
#include <iostream>

namespace identification {

/**
 * @brief Simple Levenberg-Marquardt optimizer for Non-Linear Least Squares
 *
 * Minimizes F(x) = 0.5 * || r(x) ||^2
 */
class LevenbergMarquardt {
public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;

  // Functor: (params) -> residuals
  using ResidualFunction = std::function<VectorXd(const VectorXd &)>;
  // Functor: (params) -> Jacobian (J_ij = d r_i / d x_j)
  using JacobianFunction = std::function<MatrixXd(const VectorXd &)>;

  struct Options {
    int max_iter;
    double tol_error;
    double tol_delta;
    double lambda_init;
    bool verbose;

    Options()
        : max_iter(100), tol_error(1e-6), tol_delta(1e-6), lambda_init(1e-3),
          verbose(false) {}
  };

  LevenbergMarquardt(ResidualFunction res_func, JacobianFunction jac_func,
                     Options options = Options())
      : res_func_(res_func), jac_func_(jac_func), options_(options) {}

  VectorXd minimize(VectorXd x_init) {
    VectorXd x = x_init;
    double lambda = options_.lambda_init;
    double nu = 2.0;

    VectorXd r = res_func_(x);
    double error = 0.5 * r.squaredNorm();

    for (int iter = 0; iter < options_.max_iter; ++iter) {
      if (options_.verbose) {
        std::cout << "Iter: " << iter << ", Error: " << error
                  << ", Lambda: " << lambda << std::endl;
      }

      MatrixXd J = jac_func_(x);
      MatrixXd JtJ = J.transpose() * J;
      VectorXd Jtr = J.transpose() * r;

      // Solve (J^T J + lambda I) delta = -J^T r
      MatrixXd A = JtJ + lambda * MatrixXd::Identity(JtJ.rows(), JtJ.cols());
      VectorXd delta = A.colPivHouseholderQr().solve(-Jtr);

      if (delta.norm() < options_.tol_delta) {
        if (options_.verbose)
          std::cout << "Converged (delta)." << std::endl;
        break;
      }

      // Check improvements
      VectorXd x_new = x + delta;
      VectorXd r_new = res_func_(x_new);
      double error_new = 0.5 * r_new.squaredNorm();

      // Gain ratio rho = (error - error_new) / (L(0) - L(delta))
      // L(0) - L(delta) approx 0.5 * delta^T (lambda*delta - Jtr)
      double dL = 0.5 * delta.dot(lambda * delta - Jtr);
      double rho = (error - error_new) / dL;

      if (rho > 0) {
        // Accept step
        x = x_new;
        r = r_new;
        error = error_new;
        lambda *= std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * rho - 1.0, 3));
        nu = 2.0;

        if (error < options_.tol_error) {
          if (options_.verbose)
            std::cout << "Converged (error)." << std::endl;
          break;
        }
      } else {
        // Reject step
        lambda *= nu;
        nu *= 2.0;
      }
    }

    return x;
  }

private:
  ResidualFunction res_func_;
  JacobianFunction jac_func_;
  Options options_;
};

} // namespace identification

#endif // IDENTIFICATION_OPTIMIZER_HPP_
