#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pigpio.h>  // Incluye la biblioteca pigpio

/*DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino")),
      gpio_initialized_(false) {}*/


DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino")),
      gpio_initialized_(false),
      arduino_(4, 18) // Example pins for servo_pin_1 and servo_pin_2
{
}
hardware_interface::CallbackReturn DiffDriveArduino::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Configuring...");

    time_ = std::chrono::system_clock::now();

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.left_motor_pin = std::stoi(info_.hardware_parameters["left_motor_pin"]);
    cfg_.right_motor_pin = std::stoi(info_.hardware_parameters["right_motor_pin"]);

    // Set up the wheels
    l_wheel_.setup(cfg_.left_wheel_name, cfg_.left_motor_pin);
    r_wheel_.setup(cfg_.right_wheel_name, cfg_.right_motor_pin);

    RCLCPP_INFO(logger_, "Finished Configuration");

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
    // We need to set up a position and a velocity interface for each wheel

    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
    // We need to set up a velocity command interface for each wheel

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

    return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Starting Controller...");

    // Initialize GPIO
    if (gpio_initialized_ < 0) {
        RCLCPP_ERROR(logger_, "Failed to initialize GPIO");
        return CallbackReturn::ERROR;
    }
    gpio_initialized_ = true;

    // Set the initial PWM settings if needed
    // ...

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Stopping Controller...");

    if (gpio_initialized_) {
        gpioTerminate(); // Terminate GPIO library
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduino::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Calculate time delta
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    double deltaSeconds = diff.count();
    time_ = new_time;

    // Read encoder values if applicable
    // Update wheel positions and velocities
    // ...

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!gpio_initialized_) {
        return hardware_interface::return_type::ERROR;
    }

    // Convert the command to the appropriate PWM signal
    gpioSetPWMfrequency(l_wheel_.pin, 50);
    gpioSetPWMrange(l_wheel_.pin, 50);
    gpioPWM(l_wheel_.pin, 50);

    gpioSetPWMfrequency(r_wheel_.pin, 50);
    gpioSetPWMrange(r_wheel_.pin, 50);
    gpioPWM(r_wheel_.pin, 50);

    return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    DiffDriveArduino,
    hardware_interface::SystemInterface
)
