#include "gazebo_simulator/can_to_gazebo.hpp"
#include <rclcpp/rclcpp.hpp>
#include <filesystem>

CanToGazeboNode::CanToGazeboNode() : rclcpp::Node("can_to_gazebo") {
  const std::string robot_name = declare_parameter<std::string>("robot_name", "arm_2");

  // Load motor configuration from YAML
  load_motor_config_from_yaml(robot_name);

  // Create publisher for velocity commands
  gazebo_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 10);

  // Create subscription for CAN messages
  can_sub_ = create_subscription<uec_msgs::msg::CANArray>(
      "/can/tx/port2", 10,
      std::bind(&CanToGazeboNode::can_callback, this,
                std::placeholders::_1));

  // Log loaded motors
  RCLCPP_INFO(get_logger(),
              "CANArray to Gazebo converter initialized with %zu motors. "
              "Listening on /can/tx, publishing to "
              "/forward_velocity_controller/commands",
              motor_configs_.size());

  for (size_t i = 0; i < motor_configs_.size(); ++i) {
    const auto &cfg = motor_configs_[i];
    RCLCPP_INFO(get_logger(),
                "  Motor %d: CAN ID=%d, Joint=%s, Type=%s, Invert=%s",
                static_cast<int>(i),
                cfg.can_id,
                cfg.joint_name.c_str(),
                cfg.control_type.c_str(),
                cfg.invert ? "true" : "false");
  }
}

void CanToGazeboNode::load_motor_config_from_yaml(const std::string &robot_name) {
  // Get ROS2_WS environment variable
  const char *ros2_ws = std::getenv("ROS2_WS");
  if (!ros2_ws) {
    RCLCPP_WARN(get_logger(),
                "ROS2_WS environment variable not set. "
                "Attempting to load from relative path.");
    ros2_ws = ".";
  }

  // Prefer dedicated motors.yaml to avoid breaking processes given controller_config
  std::string motors_path =
      std::string(ros2_ws) +
      "/src/gazebo_simulator/config/motors_" +
      robot_name +
      ".yaml";

  std::string controller_path =
      std::string(ros2_ws) +
      "/src/gazebo_simulator/config/controller_" +
      robot_name +
      ".yaml";

  YAML::Node config;
  YAML::Node motors_node;

  // Try motors.yaml first
  try {
    if (std::filesystem::exists(motors_path)) {
      RCLCPP_INFO(get_logger(), "Loading motor config from: %s", motors_path.c_str());
      config = YAML::LoadFile(motors_path);
      if (config["motors"]) motors_node = config["motors"];
    }
  } catch (const YAML::Exception &e) {
    RCLCPP_WARN(get_logger(), "Failed to load %s: %s", motors_path.c_str(), e.what());
  }

  // Fallback to controller_config.yaml (backwards compatibility)
  if (!motors_node) {
    try {
      RCLCPP_INFO(get_logger(), "Loading motor config from: %s", controller_path.c_str());
      config = YAML::LoadFile(controller_path);
      if (config["can_to_gazebo"] && config["can_to_gazebo"]["ros__parameters"] && config["can_to_gazebo"]["ros__parameters"]["motors"]) {
        motors_node = config["can_to_gazebo"]["ros__parameters"]["motors"];
      } else if (config["motors"]) {
        motors_node = config["motors"];
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_WARN(get_logger(), "Failed to load %s: %s", controller_path.c_str(), e.what());
    }
  }

  if (!motors_node) {
    RCLCPP_WARN(get_logger(), "No motor configuration found. Using empty motor list.");
    return;
  }

    // Initialize velocity vector
    joint_velocities_.clear();
    motor_configs_.clear();
    can_id_to_joint_.clear();
    can_id_to_motor_idx_.clear();

    if (!motors_node.IsSequence()) {
      RCLCPP_ERROR(get_logger(),
                   "motors configuration is not a list in YAML");
      return;
    }

    for (size_t i = 0; i < motors_node.size(); ++i) {
      const YAML::Node &motor = motors_node[i];

      MotorConfig cfg;
      cfg.can_id = motor["can_id"].as<uint8_t>();
      cfg.joint_name = motor["joint_name"].as<std::string>();
      cfg.invert = motor["invert"].as<bool>(false);
      cfg.control_type = motor["control_type"].as<std::string>("velocity");

      motor_configs_.push_back(cfg);
      can_id_to_joint_[cfg.can_id] = cfg.joint_name;
      can_id_to_motor_idx_[cfg.can_id] = i;
      joint_velocities_.push_back(0.0);
    }

    RCLCPP_INFO(get_logger(), "Successfully loaded %zu motor configs from YAML",
                motor_configs_.size());
}

void CanToGazeboNode::can_callback(
    const uec_msgs::msg::CANArray::SharedPtr msg) {
  if (motor_configs_.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "No motor configs loaded. Ignoring CAN message.");
    return;
  }

  bool updated = false;

  for (const auto &can_msg : msg->array) {
    uint8_t motor_id = can_msg.bulk_id;

    // Find motor config for this CAN ID
    auto it = can_id_to_motor_idx_.find(motor_id);
    if (it == can_id_to_motor_idx_.end()) {
      // Unknown motor ID, skip
      continue;
    }

    size_t motor_idx = it->second;
    const auto &cfg = motor_configs_[motor_idx];

    // Extract velocity from CAN data
    double velocity = 0.0;
    if (!can_msg.data.empty()) {
      velocity = static_cast<double>(can_msg.data[0]);
    }

    // Apply invert flag
    if (cfg.invert) {
      velocity = -velocity;
    }

    joint_velocities_[motor_idx] = velocity;
    updated = true;

    RCLCPP_DEBUG(get_logger(), "Motor %d (%s) -> velocity: %.2f", motor_id,
                 cfg.joint_name.c_str(), velocity);
  }

  if (updated) {
    publish_velocity_command();
  }
}

void CanToGazeboNode::publish_velocity_command() {
  auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();

  // Publish velocities in order of motor config
  for (double velocity : joint_velocities_) {
    msg->data.push_back(velocity);
  }

  gazebo_pub_->publish(std::move(msg));

  RCLCPP_DEBUG(get_logger(), "Published %zu joint velocities",
               joint_velocities_.size());
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanToGazeboNode>());
  rclcpp::shutdown();
  return 0;
}
