#ifndef MUJOCO_COLLISION_CHECKER_HPP_
#define MUJOCO_COLLISION_CHECKER_HPP_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <mujoco/mujoco.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot {

class MujocoCollisionChecker {
public:
  MujocoCollisionChecker() : m_(nullptr), d_(nullptr) {}

  ~MujocoCollisionChecker() {
    if (d_)
      mj_deleteData(d_);
    if (m_)
      mj_deleteModel(m_);
  }

  void initialize(const std::string &xml_path) {
    char error[1000] = "Could not load binary model";
    m_ = mj_loadXML(xml_path.c_str(), 0, error, 1000);
    if (!m_) {
      throw std::runtime_error("Mujoco load error: " + std::string(error));
    }
    d_ = mj_makeData(m_);
    if (!d_) {
      throw std::runtime_error("Mujoco make data error");
    }
    cacheModelMetadata();
    std::cout << "MujocoCollisionChecker initialized with: " << xml_path
              << std::endl;
  }

  bool checkCollision(const std::vector<double> &q) {
    if (!setState(q)) {
      return false;
    }
    if (d_->ncon > 0) {
      return true;
    }
    return false;
  }

  const std::vector<double> &jointLowerLimits() const { return joint_lower_limits_; }
  const std::vector<double> &jointUpperLimits() const { return joint_upper_limits_; }
  const std::vector<double> &actuatorLimits() const { return actuator_limits_; }
  const std::vector<double> &homeConfiguration() const { return home_configuration_; }
  std::size_t jointCount() const { return joint_qpos_indices_.size(); }
  std::size_t actuatorCount() const { return actuator_limits_.size(); }

  void printCollisions() {
    if (!m_ || !d_)
      return;
    if (d_->ncon > 0) {
      std::cout << "DEBUG: Collisions detected: " << d_->ncon << std::endl;
      for (int c = 0; c < d_->ncon; ++c) {
        int g1 = d_->contact[c].geom1;
        int g2 = d_->contact[c].geom2;
        const char *name1 = mj_id2name(m_, mjOBJ_GEOM, g1);
        const char *name2 = mj_id2name(m_, mjOBJ_GEOM, g2);
        std::cout << "  " << (name1 ? name1 : "?") << " <-> "
                  << (name2 ? name2 : "?") << std::endl;
      }
    }
  }

private:
  void cacheModelMetadata() {
    joint_qpos_indices_.clear();
    joint_lower_limits_.clear();
    joint_upper_limits_.clear();
    actuator_limits_.clear();
    home_configuration_.assign(m_->nq, 0.0);

    const int home_id = mj_name2id(m_, mjOBJ_KEY, "home");
    if (home_id >= 0 && m_->key_qpos) {
      const mjtNum *home_ptr = m_->key_qpos + static_cast<std::size_t>(home_id) * m_->nq;
      for (int i = 0; i < m_->nq; ++i) {
        home_configuration_[i] = home_ptr[i];
      }
    }

    for (int j = 0; j < m_->njnt; ++j) {
      const int type = m_->jnt_type[j];
      if (type == mjJNT_FREE || type == mjJNT_BALL) {
        continue;
      }

      joint_qpos_indices_.push_back(m_->jnt_qposadr[j]);

      if (m_->jnt_limited[j]) {
        joint_lower_limits_.push_back(m_->jnt_range[2 * j]);
        joint_upper_limits_.push_back(m_->jnt_range[2 * j + 1]);
      } else {
        joint_lower_limits_.push_back(-M_PI);
        joint_upper_limits_.push_back(M_PI);
      }
    }

    actuator_limits_.reserve(m_->nu);
    for (int a = 0; a < m_->nu; ++a) {
      double limit = 100.0;
      if (m_->actuator_ctrllimited[a]) {
        const double low = m_->actuator_ctrlrange[2 * a];
        const double high = m_->actuator_ctrlrange[2 * a + 1];
        limit = std::max(std::abs(low), std::abs(high));
      }
      actuator_limits_.push_back(limit);
    }
  }

  bool setState(const std::vector<double> &q) {
    if (!m_ || !d_) {
      return false;
    }

    for (int i = 0; i < m_->nq; ++i) {
      d_->qpos[i] = i < static_cast<int>(home_configuration_.size())
                        ? home_configuration_[i]
                        : 0.0;
    }
    for (int i = 0; i < m_->nv; ++i) {
      d_->qvel[i] = 0.0;
    }
    for (int i = 0; i < m_->na; ++i) {
      d_->act[i] = 0.0;
    }
    for (int i = 0; i < m_->nu; ++i) {
      d_->ctrl[i] = 0.0;
    }

    for (std::size_t i = 0; i < q.size() && i < joint_qpos_indices_.size(); ++i) {
      d_->qpos[joint_qpos_indices_[i]] = q[i];
    }

    mj_fwdPosition(m_, d_);
    return true;
  }

  mjModel *m_;
  mjData *d_;
  std::vector<int> joint_qpos_indices_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> actuator_limits_;
  std::vector<double> home_configuration_;
};

} // namespace robot

#endif // MUJOCO_COLLISION_CHECKER_HPP_
