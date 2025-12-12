#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <mujoco/mujoco.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

class PandaSimNode : public rclcpp::Node
{
public:
  PandaSimNode()
  : rclcpp::Node("panda_sim_node")
  {
    const auto default_model = fs::path("franka_emika_panda") / "scene.xml";
    const std::string model_param =
      this->declare_parameter<std::string>("model_path", default_model.string());
    const double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 100.0);

    const fs::path resolved_model = fs::absolute(model_param);
    if (!fs::exists(resolved_model)) {
      throw std::runtime_error("Model file not found: " + resolved_model.string());
    }

    char error[512];
    model_.reset(mj_loadXML(resolved_model.c_str(), nullptr, error, sizeof(error)));
    if (!model_) {
      throw std::runtime_error("Failed to load MuJoCo model: " + std::string(error));
    }

    data_.reset(mj_makeData(model_.get()));
    if (!data_) {
      throw std::runtime_error("Failed to allocate MuJoCo data");
    }

    // If the model provides a keyframe named "home", reset to it.
    const int home_id = mj_name2id(model_.get(), mjOBJ_KEY, "home");
    if (home_id >= 0) {
      mj_resetDataKeyframe(model_.get(), data_.get(), home_id);
    } else {
      mj_resetData(model_.get(), data_.get());
    }

    joint_indices_.reserve(model_->njnt);
    joint_names_.reserve(model_->njnt);
    for (int j = 0; j < model_->njnt; ++j) {
      const int type = model_->jnt_type[j];
      if (type == mjJNT_FREE || type == mjJNT_BALL) {
        // Skipping free/ball joints to keep the message simple (only hinge/slide are published).
        continue;
      }
      joint_indices_.push_back(j);
      joint_names_.emplace_back(model_->names + model_->name_jntadr[j]);
    }

    if (joint_indices_.empty()) {
      throw std::runtime_error("No hinge/slide joints found in the model to publish");
    }

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("panda/joint_states", 10);
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = this->create_wall_timer(period, std::bind(&PandaSimNode::step, this));

    RCLCPP_INFO(get_logger(), "Loaded model from %s, publishing %zu joints at %.1f Hz",
                resolved_model.string().c_str(), joint_indices_.size(), publish_rate_hz);
  }

private:
  void step()
  {
    // Advance simulation by one step using MuJoCo's internal timestep.
    mj_step(model_.get(), data_.get());

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position.resize(joint_indices_.size());
    msg.velocity.resize(joint_indices_.size());

    for (std::size_t i = 0; i < joint_indices_.size(); ++i) {
      const int j = joint_indices_[i];
      const int qpos_adr = model_->jnt_qposadr[j];
      const int qvel_adr = model_->jnt_dofadr[j];
      msg.position[i] = data_->qpos[qpos_adr];
      msg.velocity[i] = data_->qvel[qvel_adr];
    }

    joint_pub_->publish(std::move(msg));
  }

  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
  std::vector<int> joint_indices_;
  std::vector<std::string> joint_names_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PandaSimNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("panda_sim_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
