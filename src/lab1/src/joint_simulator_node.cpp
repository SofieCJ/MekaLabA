#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "lab1/joint_simulator.hpp"
#include <chrono>
#include <memory>
#include <functional>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

class JointSimulatorNode : public rclcpp::Node {
public:
    JointSimulatorNode()
        : Node("joint_simulator_node"), joint_simulator_() {
        // Use rclcpp::QoS(10) and matching SharedPtr callback type

        this->declare_parameter<double>("K", 230.0);
        this->declare_parameter<double>("T", 0.15);
        this->declare_parameter<double>("noise", 0.0);

        K_ = this->get_parameter("K").as_double();
        T_ = this->get_parameter("T").as_double();
        noise_ = this->get_parameter("noise").as_double();

        joint_simulator_.setParameters(K_, T_);
        joint_simulator_.setNoise(noise_);

        param_cb_handle_ =
            this->add_on_set_parameters_callback(
            std::bind(&JointSimulatorNode::parameterCallback,
                    this,
                     std::placeholders::_1)
   );

        voltage_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "voltage",
            rclcpp::QoS(10),
            [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
                this->voltageCallback(msg);
            }
        );

        angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "measured_angle",
            rclcpp::QoS(10)
        );

        timer_ = this->create_wall_timer(
            20ms,
            [this]() { this->updateAndPublish(); }
        );
    }

private:

    double K_{230.0};
    double T_{0.15};
    double noise_{0.0};

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;


    rcl_interfaces::msg::SetParametersResult
parameterCallback(const std::vector<rclcpp::Parameter> & params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";

        // Midlertidige kandidater (så vi kan avvise uten å endre state)
        double newK = K_;
        double newT = T_;
        double newNoise = noise_;

        for (const auto & p : params) {
            const auto & name = p.get_name();

            if (name == "K") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    result.successful = false;
                    result.reason = "K must be a double";
                    return result;
                }
                newK = p.as_double();
            }
            else if (name == "T") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    result.successful = false;
                    result.reason = "T must be a double";
                    return result;
                }
                newT = p.as_double();
                if (newT <= 0.0) {
                    result.successful = false;
                    result.reason = "T must be > 0";
                    return result;
                }
            }
            else if (name == "noise") {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    result.successful = false;
                    result.reason = "noise must be a double";
                    return result;
                }
                newNoise = p.as_double();
                if (newNoise < 0.0) {   // valgfritt krav
                    result.successful = false;
                    result.reason = "noise must be >= 0";
                    return result;
                }
            }
        }

        // Hvis alt er OK: commit
        K_ = newK;
        T_ = newT;
        noise_ = newNoise;

        joint_simulator_.setParameters(K_, T_);
        joint_simulator_.setNoise(noise_);

        RCLCPP_INFO(this->get_logger(), "Updated params: K=%.3f, T=%.3f, noise=%.3f", K_, T_, noise_);
        return result;
    }


    void voltageCallback(std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        joint_simulator_.setVoltage(msg->data);
    }

    void updateAndPublish() {
        joint_simulator_.update(0.02);

        auto angle_msg = std::make_shared<std_msgs::msg::Float64>();
        angle_msg->data = joint_simulator_.getAngle();
        angle_publisher_->publish(*angle_msg);
    }


    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr voltage_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
    jointSimulator joint_simulator_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
