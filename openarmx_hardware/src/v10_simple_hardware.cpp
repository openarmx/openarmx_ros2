// Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
//
// Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
// https://www.openarmx.com
//
// This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
// 4.0 International License (CC BY-NC-SA 4.0).
//
// To view a copy of this license, visit:
// http://creativecommons.org/licenses/by-nc-sa/4.0/
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

#include "openarmx_hardware/v10_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <openarmx/robstride_motor/rs_motor_control.hpp>

namespace openarmx_hardware {

// Helper function to create MIT mode packet (equivalent to csp_set_mode_packet)
// MIT mode is MOTION_CONTROL mode (value 0) in RUN_MODE parameter
static inline openarmx::robstride_motor::CANPacket mit_set_mode_packet(
    const openarmx::robstride_motor::Motor& motor) {
  // RUN_MODE = 0x7005, 0 = MOTION_CONTROL (MIT mode)
  return openarmx::robstride_motor::CanPacketEncoder::create_write_param_command(
      motor, 0x7005, 0.0f, openarmx::robstride_motor::ParamValueType::UINT8);
}

OpenArmX_v10HW::OpenArmX_v10HW() = default;

OpenArmX_v10HW::~OpenArmX_v10HW() {
  // Stop parameter node executor thread
  if (param_spin_thread_active_) {
    param_spin_thread_active_ = false;
    if (param_executor_) {
      param_executor_->cancel();
    }
    if (param_spin_thread_.joinable()) {
      param_spin_thread_.join();
    }
  }
}

bool OpenArmX_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse node_namespace (launch-level prefix, e.g. "robot1" for multi-robot)
  it = info.hardware_parameters.find("node_namespace");
  node_namespace_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: false per user request)
  it = info.hardware_parameters.find("can_fd");
  std::string raw_can_fd = (it != info.hardware_parameters.end()) ? it->second : std::string("<unset>");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = false;  // Default to false now
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  // Parse control_mode (default: mit)
  it = info.hardware_parameters.find("control_mode");
  if (it == info.hardware_parameters.end()) {
    control_mode_ = ControlMode::MIT;
  } else {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    if (value == "csp") {
      control_mode_ = ControlMode::CSP;
    } else {
      control_mode_ = ControlMode::MIT;  // fallback
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Raw can_fd param: %s", raw_can_fd.c_str());

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s, control_mode=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled",
              (control_mode_ == ControlMode::MIT ? "mit" : "csp"));
  return true;
}

void OpenArmX_v10HW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarmx_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarmx_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarmx_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArmX_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmX_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize ROS2 node for dynamic parameters
  std::string node_name = "openarmx_" + arm_prefix_ + "hardware_params";
  std::string node_ns = node_namespace_.empty() ? "" : ("/" + node_namespace_);
  param_node_ = std::make_shared<rclcpp::Node>(node_name, node_ns);

  // Initialize KP and KD values with defaults
  // 注意：第8个值是夹爪，增大KP/KD可提高响应速度和阻尼
  kp_values_ = {50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0, 50.0};  // 夹爪KP从10.0改为50.0
  kd_values_ = {2.5, 2.5, 2.5, 2.5, 0.5, 0.5, 0.5, 2.5};          // 夹爪KD从0.5改为2.5（与大关节相同）

  // Declare ROS2 parameters for KP and KD
  for (size_t i = 0; i < kp_values_.size(); ++i) {
    std::string kp_param_name = "kp_joint" + std::to_string(i + 1);
    std::string kd_param_name = "kd_joint" + std::to_string(i + 1);

    param_node_->declare_parameter(kp_param_name, kp_values_[i]);
    param_node_->declare_parameter(kd_param_name, kd_values_[i]);

    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Declared parameter %s with default value %.2f",
                kp_param_name.c_str(), kp_values_[i]);
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Declared parameter %s with default value %.2f",
                kd_param_name.c_str(), kd_values_[i]);
  }

  // Register parameter callback for dynamic reconfiguration
  param_callback_handle_ = param_node_->add_on_set_parameters_callback(
      std::bind(&OpenArmX_v10HW::parameters_callback, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Dynamic parameter reconfiguration enabled for KP and KD values");

  // Create executor and start spinning param_node_ in a separate thread
  param_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  param_executor_->add_node(param_node_);
  param_spin_thread_active_ = true;
  param_spin_thread_ = std::thread([this]() {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Parameter node executor thread started for node: %s",
                param_node_->get_name());
    while (param_spin_thread_active_ && rclcpp::ok()) {
      param_executor_->spin_some(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Parameter node executor thread stopped");
  });

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Parameter node %s is now spinning in background thread",
              param_node_->get_name());

  // Initialize OpenArmX with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Initializing OpenArmX on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  openarmx =
      std::make_unique<openarmx::can::socket::OpenArmX>(can_interface_, can_fd_);

  // Initialize arm motors with V10 defaults
  openarmx->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"), "Initializing gripper...");
    openarmx->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "OpenArmX V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmX_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarmx->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarmx->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArmX_v10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArmX_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArmX_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"), "Activating OpenArmX V10...");
  openarmx->set_callback_mode_all(openarmx::robstride_motor::CallbackMode::STATE);

  // Enable a short debug window to verify raw motor readings vs. published joint states
  debug_cycles_remaining_ = 50;  // ~first 50 read() calls

  if (control_mode_ == ControlMode::MIT) {
    using namespace openarmx::robstride_motor;
    auto& master = openarmx->get_master_can_device_collection();
    auto& sock = master.get_can_socket();

    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Configuring motors for MIT mode...");

    // 1) Disable all motors
    if (!openarmx->disable_all()) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArmX_v10HW"),
                  "Some motors failed to disable, but continuing with configuration...");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 2) Collect all motors and devices (arm + gripper)
    std::vector<Motor*> all_motors;
    std::vector<RSCANDevice*> all_devices;

    // Add arm motors
    auto& arm = openarmx->get_arm();
    auto arm_motors = arm.get_all_motors();
    auto arm_devices = arm.get_all_devices();
    all_motors.insert(all_motors.end(), arm_motors.begin(), arm_motors.end());
    all_devices.insert(all_devices.end(), arm_devices.begin(), arm_devices.end());

    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Configuring %zu arm motors for MIT mode", arm_motors.size());

    // Add gripper motor if enabled
    if (hand_) {
      auto g_motors = openarmx->get_gripper().get_all_motors();
      auto g_devices = openarmx->get_gripper().get_all_devices();
      if (!g_motors.empty()) {
        all_motors.insert(all_motors.end(), g_motors.begin(), g_motors.end());
        all_devices.insert(all_devices.end(), g_devices.begin(), g_devices.end());
        RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                    "Configuring gripper motor for MIT mode");
      }
    }

    // 3) Configure all motors to MIT mode using unified helper function
    for (size_t i = 0; i < all_motors.size(); ++i) {
      configure_motor_mit_mode(all_motors[i], all_devices[i], sock);
    }

    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Successfully configured %zu motors for MIT mode", all_motors.size());

    // 4) Enable all motors
    if (!openarmx->enable_all()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmX_v10HW"),
                   "Failed to enable all motors in MIT mode");
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    openarmx->refresh_all();
    openarmx->recv_all();

    ////////////////////////////////////  初始化命令为当前位置，避免自动回零 ////////////////////////////////
    const auto direction_multipliers = get_motor_direction_multipliers();
    for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
      pos_commands_[i] = arm_motors[i]->get_position() * direction_multipliers[i];
    }
    if (hand_) {
      auto gripper_motors = openarmx->get_gripper().get_motors();
      if (!gripper_motors.empty()) {
        pos_commands_[ARM_DOF] = motor_radians_to_joint(gripper_motors[0]->get_position());
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////

    // Return to zero position
    // return_to_zero();
  } else {
    // CSP: follow documented flow
    using namespace openarmx::robstride_motor;
    auto& master = openarmx->get_master_can_device_collection();
    auto& sock = master.get_can_socket();

    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Configuring motors for CSP mode...");

    // 1) Disable all motors
    if (!openarmx->disable_all()) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArmX_v10HW"),
                  "Some motors failed to disable, but continuing with configuration...");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 2) Collect all motors and devices (arm + gripper)
    std::vector<Motor*> all_motors;
    std::vector<RSCANDevice*> all_devices;

    // Add arm motors
    auto& arm = openarmx->get_arm();
    auto arm_motors = arm.get_all_motors();
    auto arm_devices = arm.get_all_devices();
    all_motors.insert(all_motors.end(), arm_motors.begin(), arm_motors.end());
    all_devices.insert(all_devices.end(), arm_devices.begin(), arm_devices.end());

    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Configuring %zu arm motors for CSP mode", arm_motors.size());

    // Add gripper motor if enabled
    if (hand_) {
      auto g_motors = openarmx->get_gripper().get_all_motors();
      auto g_devices = openarmx->get_gripper().get_all_devices();
      if (!g_motors.empty()) {
        all_motors.insert(all_motors.end(), g_motors.begin(), g_motors.end());
        all_devices.insert(all_devices.end(), g_devices.begin(), g_devices.end());
        RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                    "Configuring gripper motor for CSP mode");
      }
    }

    // 3) Configure all motors using unified helper function
    for (size_t i = 0; i < all_motors.size(); ++i) {
      configure_motor_csp_mode(all_motors[i], all_devices[i], sock);
    }

    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Successfully configured %zu motors for CSP mode", all_motors.size());

    // 4) Enable all motors
    if (!openarmx->enable_all()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmX_v10HW"),
                   "Failed to enable all motors in CSP mode");
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    openarmx->recv_all();

    ////////////////////////////////////  初始化命令为当前位置，避免自动回零 ////////////////////////////////
    const auto direction_multipliers = get_motor_direction_multipliers();
    for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
      pos_commands_[i] = arm_motors[i]->get_position() * direction_multipliers[i];
    }
    if (hand_) {
      auto g_motors = openarmx->get_gripper().get_all_motors();
      if (!g_motors.empty()) {
        pos_commands_[ARM_DOF] = motor_radians_to_joint(g_motors[0]->get_position());
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////

    // 4) Go to zero
    // return_to_zero();
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"), "OpenArmX V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmX_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Deactivating OpenArmX V10...");

  // Disable all motors (like full_arm.cpp exit)
  if (!openarmx->disable_all()) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArmX_v10HW"),
                "Some motors failed to disable during deactivation");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarmx->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"), "OpenArmX V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmX_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  openarmx->refresh_all();
  openarmx->recv_all();

  // Read arm joint states - FIXED: Now using vector order with direction correction
  // joint1 -> motor[0] (CAN ID 1), joint7 -> motor[6] (CAN ID 7)
  auto arm_motors = openarmx->get_arm().get_motors();
  const auto direction_multipliers = get_motor_direction_multipliers();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    const double raw_pos = arm_motors[i]->get_position();
    const double raw_vel = arm_motors[i]->get_velocity();
    const double raw_tau = arm_motors[i]->get_torque();
    const double sign = direction_multipliers[i];
    pos_states_[i] = raw_pos * sign;
    vel_states_[i] = raw_vel * sign;
    tau_states_[i] = raw_tau * sign;

    // Transient debug logging to diagnose unexpected offsets/signs
    if (debug_cycles_remaining_ > 0 && i == 0) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                  "[%s] joint%zu raw=%.6f rad, sign=%.1f, published=%.6f (can_id=%u)",
                  arm_prefix_.c_str(), i + 1, raw_pos, sign, pos_states_[i],
                  static_cast<unsigned>(arm_motors[i]->get_send_can_id()));
    }
  }

  if (debug_cycles_remaining_ > 0) {
    --debug_cycles_remaining_;
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    auto gripper_motors = openarmx->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      double motor_pos = gripper_motors[0]->get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      // Unimplemented: Velocity and torque mapping
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0]->get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0]->get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

void OpenArmX_v10HW::configure_motor_csp_mode(
    openarmx::robstride_motor::Motor* motor,
    openarmx::robstride_motor::RSCANDevice* device,
    openarmx::canbus::CANSocket& socket) {
  using namespace openarmx::robstride_motor;

  // 1) Switch to CSP mode
  auto mode_pkt = csp_set_mode_packet(*motor);
  auto mode_frame = device->create_can_frame(mode_pkt.send_can_id, mode_pkt.data);
  socket.write_can_frame(mode_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  // 2) Get motor limits from motor type
  auto motor_type = motor->get_motor_type();
  size_t idx = static_cast<size_t>(motor_type);
  const auto& limits = MOTOR_LIMIT_PARAMS[idx];

  // 3) Set speed limit
  auto spd_pkt = csp_set_speed_limit_packet(*motor, static_cast<float>(0.5));
  auto spd_frame = device->create_can_frame(spd_pkt.send_can_id, spd_pkt.data);
  socket.write_can_frame(spd_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  // 4) Set current limit
  auto cur_pkt = csp_set_current_limit_packet(*motor, static_cast<float>(limits.tMax));
  auto cur_frame = device->create_can_frame(cur_pkt.send_can_id, cur_pkt.data);
  socket.write_can_frame(cur_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void OpenArmX_v10HW::configure_motor_mit_mode(
    openarmx::robstride_motor::Motor* motor,
    openarmx::robstride_motor::RSCANDevice* device,
    openarmx::canbus::CANSocket& socket) {
  using namespace openarmx::robstride_motor;

  // Switch to MIT mode (motion control mode)
  auto mode_pkt = mit_set_mode_packet(*motor);
  auto mode_frame = device->create_can_frame(mode_pkt.send_can_id, mode_pkt.data);
  socket.write_can_frame(mode_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

hardware_interface::return_type OpenArmX_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  const auto direction_multipliers = get_motor_direction_multipliers();
  if (control_mode_ == ControlMode::MIT) {
    // Motion control path (MIT)
    std::vector<openarmx::robstride_motor::MotionControlParam> arm_params(ARM_DOF);

    // Lock to safely read KP/KD values
    std::lock_guard<std::mutex> lock(kp_kd_mutex_);

    for (size_t i = 0; i < ARM_DOF; ++i) {
      openarmx::robstride_motor::MotionControlParam param;
      param.kp = kp_values_[i];
      param.kd = kd_values_[i];
      param.position = pos_commands_[i] * direction_multipliers[i];
      param.velocity = vel_commands_[i] * direction_multipliers[i];
      param.torque = tau_commands_[i] * direction_multipliers[i];
      arm_params[i] = param;
    }
    openarmx->get_arm().send_motion_control_commands(arm_params);

    if (hand_ && joint_names_.size() > ARM_DOF) {
      // Stall detection: if position error is large but the gripper hasn't moved
      // for GRIPPER_STALL_CYCLES consecutive cycles, assume it's gripping a hard
      // object and clamp the command to the current state to relieve motor current.
      // Once stall is active, hold the locked position until the upper-layer sends
      // a meaningfully different command (i.e. release gesture), preventing oscillation.
      if (gripper_stall_active_) {
        // Stay locked unless upper-layer command moves far enough away (release intent)
        double cmd_delta = pos_commands_[ARM_DOF] - gripper_stall_locked_pos_;
        if (cmd_delta > GRIPPER_STALL_RELEASE_THRESHOLD) {
          gripper_stall_active_  = false;
          gripper_stall_counter_ = 0;
          RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                      "[%s] Gripper stall released, new cmd=%.4f",
                      arm_prefix_.c_str(), pos_commands_[ARM_DOF]);
        } else {
          pos_commands_[ARM_DOF] = gripper_stall_locked_pos_;
          RCLCPP_INFO_THROTTLE(rclcpp::get_logger("OpenArmX_v10HW"),
                               *param_node_->get_clock(), 2000,
                               "[%s] Gripper stall holding at pos=%.4f",
                               arm_prefix_.c_str(), gripper_stall_locked_pos_);
        }
      } else {
        double pos_error  = std::abs(pos_commands_[ARM_DOF] - pos_states_[ARM_DOF]);
        double pos_change = std::abs(pos_states_[ARM_DOF] - gripper_last_pos_state_);
        if (pos_error > GRIPPER_STALL_POS_ERROR_THRESHOLD &&
            pos_change < GRIPPER_STALL_POS_CHANGE_THRESHOLD) {
          ++gripper_stall_counter_;
          if (gripper_stall_counter_ >= GRIPPER_STALL_CYCLES) {
            gripper_stall_active_     = true;
            gripper_stall_locked_pos_ = pos_states_[ARM_DOF];
            pos_commands_[ARM_DOF]    = gripper_stall_locked_pos_;
            RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                        "[%s] Gripper stall detected, locking at pos=%.4f",
                        arm_prefix_.c_str(), gripper_stall_locked_pos_);
          }
        } else {
          gripper_stall_counter_ = 0;
        }
      }
      gripper_last_pos_state_ = pos_states_[ARM_DOF];

      double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
      openarmx::robstride_motor::MotionControlParam gripper_param;
      gripper_param.kp       = kp_values_[ARM_DOF];
      gripper_param.kd       = kd_values_[ARM_DOF];
      gripper_param.position = motor_command;
      gripper_param.velocity = 0.0;
      gripper_param.torque   = 0.0;
      openarmx->get_gripper().send_motion_control_commands({gripper_param});
    }
  } else {
    // CSP path: write LOC_REF for each motor
    using namespace openarmx::robstride_motor;
    auto& arm = openarmx->get_arm();
    auto motors = arm.get_all_motors();
    auto devices = arm.get_all_devices();
    auto& sock = openarmx->get_master_can_device_collection().get_can_socket();
    for (size_t i = 0; i < ARM_DOF && i < motors.size(); ++i) {
      float target = static_cast<float>(pos_commands_[i] * direction_multipliers[i]);
      auto pkt = csp_set_target_position_packet(*motors[i], target);
      auto frame = devices[i]->create_can_frame(pkt.send_can_id, pkt.data);
      sock.write_can_frame(frame);
    }

    if (hand_ && joint_names_.size() > ARM_DOF) {
      auto g_motors = openarmx->get_gripper().get_all_motors();
      auto g_devices = openarmx->get_gripper().get_all_devices();
      if (!g_motors.empty()) {
        float motor_command = static_cast<float>(joint_to_motor_radians(pos_commands_[ARM_DOF]));
        auto pkt = csp_set_target_position_packet(*g_motors[0], motor_command);
        auto frame = g_devices[0]->create_can_frame(pkt.send_can_id, pkt.data);
        sock.write_can_frame(frame);
      }
    }
  }

  openarmx->recv_all(1000);
  return hardware_interface::return_type::OK;
}

void OpenArmX_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Returning to zero position...");

  // Lock to safely read KP/KD values
  std::lock_guard<std::mutex> lock(kp_kd_mutex_);

  // Return arm to zero with motion control - use hardware order for zero position
  std::vector<openarmx::robstride_motor::MotionControlParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    openarmx::robstride_motor::MotionControlParam param;
    param.kp = kp_values_[i];
    param.kd = kd_values_[i];
    param.position = 0.0;
    param.velocity = 0.0;
    param.torque = 0.0;
    arm_params.push_back(param);
  }
  openarmx->get_arm().send_motion_control_commands(arm_params);

  // Return gripper to zero if enabled
  if (hand_) {
    openarmx::robstride_motor::MotionControlParam gripper_param;
    gripper_param.kp = kp_values_[ARM_DOF];
    gripper_param.kd = kd_values_[ARM_DOF];
    gripper_param.position = GRIPPER_JOINT_0_POSITION;
    gripper_param.velocity = 0.0;
    gripper_param.torque = 0.0;
    openarmx->get_gripper().send_motion_control_commands({gripper_param});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarmx->recv_all();
}

// Gripper mapping helper functions
double OpenArmX_v10HW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double OpenArmX_v10HW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

// Motor direction correction based on URDF axis definitions and Python reference
std::vector<double> OpenArmX_v10HW::get_motor_direction_multipliers() const {
  return std::vector<double>{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
}

// Dynamic parameter callback implementation
rcl_interfaces::msg::SetParametersResult OpenArmX_v10HW::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::lock_guard<std::mutex> lock(kp_kd_mutex_);

  for (const auto& param : parameters) {
    std::string param_name = param.get_name();

    // Check if it's a KP parameter
    if (param_name.find("kp_joint") == 0) {
      // Extract joint index from parameter name (kp_joint1 -> index 0)
      size_t joint_idx = std::stoi(param_name.substr(8)) - 1;

      if (joint_idx < kp_values_.size()) {
        double new_value = param.as_double();
        double old_value = kp_values_[joint_idx];
        kp_values_[joint_idx] = new_value;

        RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                    "Updated %s: %.2f -> %.2f",
                    param_name.c_str(), old_value, new_value);
      } else {
        result.successful = false;
        result.reason = "Joint index out of range for " + param_name;
        RCLCPP_ERROR(rclcpp::get_logger("OpenArmX_v10HW"),
                     "Failed to update %s: joint index %zu out of range",
                     param_name.c_str(), joint_idx);
      }
    }
    // Check if it's a KD parameter
    else if (param_name.find("kd_joint") == 0) {
      // Extract joint index from parameter name (kd_joint1 -> index 0)
      size_t joint_idx = std::stoi(param_name.substr(8)) - 1;

      if (joint_idx < kd_values_.size()) {
        double new_value = param.as_double();
        double old_value = kd_values_[joint_idx];
        kd_values_[joint_idx] = new_value;

        RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                    "Updated %s: %.2f -> %.2f",
                    param_name.c_str(), old_value, new_value);
      } else {
        result.successful = false;
        result.reason = "Joint index out of range for " + param_name;
        RCLCPP_ERROR(rclcpp::get_logger("OpenArmX_v10HW"),
                     "Failed to update %s: joint index %zu out of range",
                     param_name.c_str(), joint_idx);
      }
    }
  }

  // Print current KP/KD values for all joints after any parameter change
  // if (result.successful && !parameters.empty()) {
  //   RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //               "========== Current KP/KD Values ==========");

  //   // Print arm joints (1-7)
  //   RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //               "Arm Joints:");
  //   for (size_t i = 0; i < ARM_DOF && i < kp_values_.size(); ++i) {
  //     RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //                 "  Joint %zu: KP=%.2f, KD=%.2f",
  //                 i + 1, kp_values_[i], kd_values_[i]);
  //   }

  //   // Print gripper (joint 8) if available
  //   if (kp_values_.size() > ARM_DOF) {
  //     RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //                 "Gripper:");
  //     RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //                 "  Joint 8 (Gripper): KP=%.2f, KD=%.2f",
  //                 kp_values_[ARM_DOF], kd_values_[ARM_DOF]);
  //   }

  //   RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //               "==========================================");
  // }

  return result;
}

}  // namespace openarmx_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarmx_hardware::OpenArmX_v10HW,
                       hardware_interface::SystemInterface)
