#ifndef IDENTIFICATION_DATA_LOADER_HPP_
#define IDENTIFICATION_DATA_LOADER_HPP_

#include <Eigen/Dense>
#include <string>
#include <vector>

struct ExperimentData {
  std::vector<double> time;
  Eigen::MatrixXd q;   // N x DOF
  Eigen::MatrixXd qd;  // N x DOF
  Eigen::MatrixXd qdd; // N x DOF (Computed during preprocessing)
  Eigen::MatrixXd tau; // N x DOF
  std::size_t n_samples;
  std::size_t n_dof;
};

class DataLoader {
public:
  static ExperimentData loadCSV(const std::string &filename,
                                std::size_t n_dof = 7);
};

#endif // IDENTIFICATION_DATA_LOADER_HPP_
