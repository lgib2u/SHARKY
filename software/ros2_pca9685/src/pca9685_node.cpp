#include "ros2_pca9685/pca9685_node.h"

#include <stdexcept>

using std::placeholders::_1;
using std::placeholders::_2;

Pca9685Node::Pca9685Node(const std::string& name)
    : Node(name)
    , pca9685_dev_{std::make_unique<Pca9685>()}
{
    /* Declare parameters */
    this->declare_parameter<double>("servo_freq", 50.0);
    this->declare_parameter<double>("pca9685_osc", 25000000.0);
    this->declare_parameter<double>("max_offset_ms", 0.0);
    this->declare_parameter<double>("min_offset_ms", 0.0);

    max_offset_ms_ = this->get_parameter("max_offset_ms").as_double();
    min_offset_ms_ = this->get_parameter("min_offset_ms").as_double();

    min_ms_ = 1 + min_offset_ms_;
    max_ms_ = 2 + max_offset_ms_;

    servo_freq_ = this->get_parameter("servo_freq").as_double();
    pca9685_osc_ = this->get_parameter("pca9685_osc").as_double();

    if (pca9685_dev_->Pca9685_SetPrescale(servo_freq_, pca9685_osc_) !=
            Pca9685Hal::PCA9685_OK ||
        pca9685_dev_->Pca9685_PwrMode(Pca9685::PCA9685_MODE_NORMAL) !=
            Pca9685Hal::PCA9685_OK ||
        pca9685_dev_->Pca9685_AllLedSet(Pca9685::PCA9685_LED_OFF) !=
            Pca9685Hal::PCA9685_OK)
    {
        throw std::runtime_error("Failed to initialise PCA9685 in disabled state");
    }

    server_ = this->create_service<ros2_pca9685::srv::SetPwm>("/pca9685/set_pwm",
                                                              std::bind(&Pca9685Node::ServiceCallback,
                                                                        this,
                                                                        _1,
                                                                        _2));

    enable_server_ = this->create_service<std_srvs::srv::SetBool>(
        "/pca9685/set_enabled",
        std::bind(&Pca9685Node::EnableCallback, this, _1, _2));

    RCLCPP_WARN(get_logger(),
                "PCA9685 ready with all outputs disabled; explicit enable required.");

}

Pca9685Node::~Pca9685Node()
{
    outputs_enabled_ = false;
    if (pca9685_dev_)
    {
        pca9685_dev_->Pca9685_AllLedSet(Pca9685::PCA9685_LED_OFF);
    }
}

void Pca9685Node::EnableCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
{
    const auto result = request->data
        ? pca9685_dev_->Pca9685_AllLedClear()
        : pca9685_dev_->Pca9685_AllLedSet(Pca9685::PCA9685_LED_OFF);

    if (result != Pca9685Hal::PCA9685_OK)
    {
        outputs_enabled_ = false;
        response->success = false;
        response->message = "PCA9685 I2C operation failed; outputs remain disabled";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }

    outputs_enabled_ = request->data;
    response->success = true;
    response->message = outputs_enabled_
        ? "PCA9685 enabled; channels remain off until individually commanded"
        : "PCA9685 outputs disabled";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());

}

void Pca9685Node::ServiceCallback(const ros2_pca9685::srv::SetPwm::Request::SharedPtr request,
                                  const ros2_pca9685::srv::SetPwm::Response::SharedPtr response)
{
    if (!outputs_enabled_)
    {
        response->is_success = false;
        response->response = "PCA9685 outputs are disabled";
        RCLCPP_WARN(get_logger(), "Rejected PWM request while outputs are disabled");
        return;
    }

    if(request->channel_num > MAX_CHANNEL_NUM)
    {
        response->is_success = false;
        response->response = "Invalid channel number";
        RCLCPP_INFO_STREAM(get_logger(), "Invalid channel number" << std::endl);
        return;
    }
    if(request->target_position < 0.0 || request->target_position > MAX_SERVO_ANGLE)
    {
        response->is_success = false;
        response->response = "Invalid angle range";
        RCLCPP_INFO_STREAM(get_logger(), "Invalid angle range" << std::endl);
        return;
    }

    /* Log details */
    RCLCPP_INFO_STREAM(get_logger(), "Actuate servo at channel number " << static_cast<int>(request->channel_num) << " to " << static_cast<double>(request->target_position) << " degrees angle position" << std::endl);

    /* Calculate duty cycle based on degree value */
    double duty_cycle {0.0};

    duty_cycle = ((request->target_position / 180.0) * (max_ms_ - min_ms_)) + min_ms_;  /* Converting 0 to 180 deg to 1ms to 2 ms */
    duty_cycle = (duty_cycle/(1000.0 / servo_freq_)) * 100.0; /* Converting to duty cycle */

    if (pca9685_dev_->Pca9685_LedPwmSet(request->channel_num, duty_cycle) ==
        Pca9685Hal::PCA9685_OK)
    {
        response->is_success = true;
        response->response = "Servo actuation completed";
    }
    else
    {
        response->is_success = false;
        response->response = "PCA9685 I2C write failed";
        RCLCPP_ERROR(get_logger(), "%s", response->response.c_str());
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pca9685Node>("pca9685_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
