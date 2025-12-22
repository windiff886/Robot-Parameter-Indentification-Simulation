/**
 * @file trajectory_generator.cpp
 * @brief Base trajectory generator implementation
 */

#include "trajectory/trajectory_generator.hpp"

namespace trajectory {

TrajectoryData TrajectoryGenerator::generate(double t_start, double t_end,
                                             std::size_t num_samples) const {
  TrajectoryData data;
  const std::size_t n = numDOF();

  data.time.resize(num_samples);
  data.Q.resize(n, num_samples);
  data.Qd.resize(n, num_samples);
  data.Qdd.resize(n, num_samples);

  const double dt = (t_end - t_start) / static_cast<double>(num_samples - 1);

  for (std::size_t i = 0; i < num_samples; ++i) {
    const double t = t_start + i * dt;
    data.time[i] = t;

    TrajectoryPoint pt = evaluate(t);
    data.Q.col(i) = pt.q;
    data.Qd.col(i) = pt.qd;
    data.Qdd.col(i) = pt.qdd;
  }

  return data;
}

} // namespace trajectory
