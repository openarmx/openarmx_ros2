// Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
// Licensed under CC-BY-NC-SA 4.0

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "dynamics.hpp"

using namespace std::chrono_literals;

// Joint order as exposed by forward_effort_controller (must match YAML)
static const std::vector<std::string> LEFT_JOINT_NAMES = {
    "openarmx_left_joint1", "openarmx_left_joint2", "openarmx_left_joint3",
    "openarmx_left_joint4", "openarmx_left_joint5", "openarmx_left_joint6",
    "openarmx_left_joint7"};

static const std::vector<std::string> RIGHT_JOINT_NAMES = {
    "openarmx_right_joint1", "openarmx_right_joint2", "openarmx_right_joint3",
    "openarmx_right_joint4", "openarmx_right_joint5", "openarmx_right_joint6",
    "openarmx_right_joint7"};

// Per-joint torque safety limits [Nm]
static const std::vector<double> TAU_LIMITS = {20.0, 20.0, 7.0, 7.0, 2.0, 2.0, 2.0};

// Gravity vectors in each arm's link0 frame.
// The arm base is mounted with rpy=(±1.5708, 0, 0) relative to world.
// World gravity is -Z. After rotation:
//   right arm (rpy=+1.5708 0 0): gravity in link0 = (0, +9.81, 0)
//   left  arm (rpy=-1.5708 0 0): gravity in link0 = (0, -9.81, 0)
// Direction correction (dir=-1) is handled by hardware write(), not here.
static constexpr double RIGHT_ARM_GY = -9.81;
static constexpr double LEFT_ARM_GY  = +9.81;

class GravityCompNode : public rclcpp::Node
{
public:
    GravityCompNode() : Node("gravity_comp_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("urdf_path", "");
        this->declare_parameter<double>("g_scale", 1.05);
        this->declare_parameter<bool>("enable_left", true);
        this->declare_parameter<bool>("enable_right", true);
        this->declare_parameter<bool>("verbose", false);
        this->declare_parameter<bool>("enable_compensation", true);

        g_scale_             = this->get_parameter("g_scale").as_double();
        verbose_             = this->get_parameter("verbose").as_bool();
        enable_left_         = this->get_parameter("enable_left").as_bool();
        enable_right_        = this->get_parameter("enable_right").as_bool();
        enable_compensation_ = this->get_parameter("enable_compensation").as_bool();

        std::string urdf_path = this->get_parameter("urdf_path").as_string();
        if (urdf_path.empty()) {
            RCLCPP_FATAL(get_logger(), "Parameter 'urdf_path' is required but not set.");
            throw std::runtime_error("urdf_path not set");
        }

        // Init dynamics for both arms
        if (enable_left_) {
            left_dyn_ = std::make_unique<Dynamics>(
                urdf_path, "openarmx_left_link0", "openarmx_left_link7");
            if (!left_dyn_->Init()) {
                RCLCPP_FATAL(get_logger(), "Left arm KDL dynamics init failed");
                throw std::runtime_error("Left dynamics init failed");
            }
            left_dyn_->SetGravityVector(0.0, LEFT_ARM_GY, 0.0);
            RCLCPP_INFO(get_logger(), "Left arm dynamics: %zu joints, gy=%.2f",
                        left_dyn_->NumJoints(), LEFT_ARM_GY);
        }

        if (enable_right_) {
            right_dyn_ = std::make_unique<Dynamics>(
                urdf_path, "openarmx_right_link0", "openarmx_right_link7");
            if (!right_dyn_->Init()) {
                RCLCPP_FATAL(get_logger(), "Right arm KDL dynamics init failed");
                throw std::runtime_error("Right dynamics init failed");
            }
            right_dyn_->SetGravityVector(0.0, RIGHT_ARM_GY, 0.0);
            RCLCPP_INFO(get_logger(), "Right arm dynamics: %zu joints, gy=%.2f",
                        right_dyn_->NumJoints(), RIGHT_ARM_GY);
        }

        // Publishers to effort controllers
        if (enable_left_) {
            left_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
                "/left_forward_effort_controller/commands", 10);
        }
        if (enable_right_) {
            right_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
                "/right_forward_effort_controller/commands", 10);
        }

        // Dynamic parameter callback for runtime g_scale tuning
        param_callback_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto & p : params) {
                    if (p.get_name() == "g_scale") {
                        g_scale_ = p.as_double();
                        RCLCPP_INFO(get_logger(), "g_scale updated to %.3f", g_scale_);
                    } else if (p.get_name() == "enable_compensation") {
                        enable_compensation_ = p.as_bool();
                        RCLCPP_INFO(get_logger(), "enable_compensation set to %s",
                                    enable_compensation_ ? "true" : "false");
                    }
                }
                return result;
            });

        // Subscriber
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&GravityCompNode::joint_state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "gravity_comp_node started. g_scale=%.3f enable_compensation=%s",
                    g_scale_, enable_compensation_ ? "true" : "false");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (enable_left_ && left_dyn_) {
            publish_gravity_torques(msg, LEFT_JOINT_NAMES, *left_dyn_, left_pub_);
        }
        if (enable_right_ && right_dyn_) {
            publish_gravity_torques(msg, RIGHT_JOINT_NAMES, *right_dyn_, right_pub_);
        }
    }

    void publish_gravity_torques(
        const sensor_msgs::msg::JointState::SharedPtr & msg,
        const std::vector<std::string> & joint_names,
        Dynamics & dyn,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr & pub)
    {
        const size_t ndof = joint_names.size();  // 7

        // When compensation is disabled, publish zero torques to clear any residual feedforward
        if (!enable_compensation_) {
            auto out = std_msgs::msg::Float64MultiArray();
            out.data.assign(ndof, 0.0);
            pub->publish(out);
            return;
        }

        std::vector<double> q(ndof, 0.0);

        // Map joint_states (unordered) into q[] by name lookup
        for (size_t j = 0; j < ndof; ++j) {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[j]);
            if (it == msg->name.end()) {
                // Not all joints present yet — skip this cycle
                return;
            }
            size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
            if (idx >= msg->position.size()) return;
            q[j] = msg->position[idx];
        }

        // Compute gravity torques in URDF frame
        std::vector<double> tau_g(ndof, 0.0);
        dyn.GetGravity(q.data(), tau_g.data());

        // Scale and clamp — direction correction is done by hardware write()
        auto out = std_msgs::msg::Float64MultiArray();
        out.data.resize(ndof);
        for (size_t j = 0; j < ndof; ++j) {
            double tau_motor = g_scale_ * tau_g[j];
            double limit = (j < TAU_LIMITS.size()) ? TAU_LIMITS[j]
                                                   : std::numeric_limits<double>::infinity();
            tau_motor = std::clamp(tau_motor, -limit, limit);
            out.data[j] = tau_motor;

            if (verbose_) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "j%zu q=%.3f tau_g=%.3f tau_out=%.3f", j, q[j], tau_g[j], tau_motor);
            }
        }

        pub->publish(out);
    }

    // Parameters
    double g_scale_            = 1.05;
    bool verbose_              = false;
    bool enable_left_          = true;
    bool enable_right_         = true;
    bool enable_compensation_  = true;

    // Dynamics
    std::unique_ptr<Dynamics> left_dyn_;
    std::unique_ptr<Dynamics> right_dyn_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<GravityCompNode>());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("gravity_comp_node"), "Fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
