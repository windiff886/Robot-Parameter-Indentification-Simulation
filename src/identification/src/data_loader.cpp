#include "identification/data_loader.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

ExperimentData DataLoader::loadCSV(const std::string &filename,
                                   std::size_t n_dof) {
  ExperimentData data;
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + filename);
  }

  std::string line;
  // Read header to detect format
  std::getline(file, line);

  // Detect if qdd columns exist by checking header or column count
  // Format with qdd: time, q0..qN, qd0..qdN, qdd0..qddN, tau0..tauN
  // Format without qdd: time, q0..qN, qd0..qdN, tau0..tauN
  const std::size_t cols_with_qdd = 1 + 4 * n_dof;
  const std::size_t cols_without_qdd = 1 + 3 * n_dof;

  std::vector<double> time_vec;
  std::vector<std::vector<double>> q_vec, qd_vec, qdd_vec, tau_vec;
  bool has_qdd = false;
  bool format_detected = false;

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;
    std::vector<double> row;

    while (std::getline(ss, token, ',')) {
      row.push_back(std::stod(token));
    }

    // Detect format on first data row
    if (!format_detected) {
      if (row.size() >= cols_with_qdd) {
        has_qdd = true;
      } else if (row.size() >= cols_without_qdd) {
        has_qdd = false;
      } else {
        continue; // Skip invalid lines
      }
      format_detected = true;
    }

    // Validate row size
    const std::size_t expected_cols = has_qdd ? cols_with_qdd : cols_without_qdd;
    if (row.size() < expected_cols) {
      continue; // Skip invalid lines
    }

    time_vec.push_back(row[0]);

    std::vector<double> q_row, qd_row, qdd_row, tau_row;
    for (std::size_t i = 0; i < n_dof; ++i)
      q_row.push_back(row[1 + i]);
    for (std::size_t i = 0; i < n_dof; ++i)
      qd_row.push_back(row[1 + n_dof + i]);

    if (has_qdd) {
      for (std::size_t i = 0; i < n_dof; ++i)
        qdd_row.push_back(row[1 + 2 * n_dof + i]);
      for (std::size_t i = 0; i < n_dof; ++i)
        tau_row.push_back(row[1 + 3 * n_dof + i]);
    } else {
      for (std::size_t i = 0; i < n_dof; ++i)
        tau_row.push_back(row[1 + 2 * n_dof + i]);
    }

    q_vec.push_back(q_row);
    qd_vec.push_back(qd_row);
    if (has_qdd) {
      qdd_vec.push_back(qdd_row);
    }
    tau_vec.push_back(tau_row);
  }

  data.n_samples = time_vec.size();
  data.n_dof = n_dof;
  data.time = time_vec;

  data.q.resize(data.n_samples, n_dof);
  data.qd.resize(data.n_samples, n_dof);
  data.tau.resize(data.n_samples, n_dof);

  for (std::size_t i = 0; i < data.n_samples; ++i) {
    for (std::size_t j = 0; j < n_dof; ++j) {
      data.q(i, j) = q_vec[i][j];
      data.qd(i, j) = qd_vec[i][j];
      data.tau(i, j) = tau_vec[i][j];
    }
  }

  // Load qdd if present, otherwise leave empty for preprocess to compute
  if (has_qdd) {
    data.qdd.resize(data.n_samples, n_dof);
    for (std::size_t i = 0; i < data.n_samples; ++i) {
      for (std::size_t j = 0; j < n_dof; ++j) {
        data.qdd(i, j) = qdd_vec[i][j];
      }
    }
    std::cout << "Loaded " << data.n_samples << " samples from " << filename
              << " (with qdd from physics engine)" << std::endl;
  } else {
    std::cout << "Loaded " << data.n_samples << " samples from " << filename
              << " (qdd will be computed via numerical differentiation)"
              << std::endl;
  }

  return data;
}
