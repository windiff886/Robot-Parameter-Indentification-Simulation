#ifndef MUJOCO_COLLISION_CHECKER_HPP_
#define MUJOCO_COLLISION_CHECKER_HPP_

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
    std::cout << "MujocoCollisionChecker initialized with: " << xml_path
              << std::endl;
  }

  bool checkCollision(const std::vector<double> &q) {
    if (!m_ || !d_)
      return false;

    // Set joint positions
    // Note: MuJoCo qpos size (nq) might be different from q size if there are
    // free joints or specific robot structure. For Franka (fixed base), nq
    // should match number of joints (plus gripper). Safely copy what we have.

    int nq = m_->nq;
    // Default reset
    // mj_resetData(m_, d_); // Do we need full reset? No, just setting state is
    // faster.

    for (int i = 0; i < nq && i < (int)q.size(); ++i) {
      d_->qpos[i] = q[i];
    }

    // Forward Kinematics & collision detection
    // mj_fwdPosition computes: cinert, com, tendon, actuator, joints, geoms...
    // and importantly: contacts!
    mj_fwdPosition(m_, d_);

    // Check contacts
    if (d_->ncon > 0) {
      // Print first few contacts for debugging
      // prevent spamming, only print occasionally or if needed
      // For now, let's print the first contact to see what's hitting what
      /*
      std::cout << "Collision detected! ncon=" << d_->ncon << std::endl;
      for (int c = 0; c < d_->ncon && c < 5; ++c) {
          int g1 = d_->contact[c].geom1;
          int g2 = d_->contact[c].geom2;
          const char* name1 = mj_id2name(m_, mjOBJ_GEOM, g1);
          const char* name2 = mj_id2name(m_, mjOBJ_GEOM, g2);
          std::cout << "  Contact " << c << ": "
                    << (name1 ? name1 : "null") << " (" << g1 << ") <-> "
                    << (name2 ? name2 : "null") << " (" << g2 << ")" <<
      std::endl;
      }
      */
      return true;
    }
    return false;
  }

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
  mjModel *m_;
  mjData *d_;
};

} // namespace robot

#endif // MUJOCO_COLLISION_CHECKER_HPP_
