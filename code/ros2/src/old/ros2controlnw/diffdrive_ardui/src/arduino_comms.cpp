#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
//#include <rclcpp/rclcpp.hpp>
//#include <sstream>
//#include <cstdlib>


/*void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}


void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    std::string response = sendMsg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output)
    {
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;
}*/

#include <stdexcept>
#include <iostream>
#include <pigpio.h>  // Incluye la biblioteca pigpio

void ArduinoComms::setup()
{
    //if (< 0)
    //{
    //    throw std::runtime_error("Pigpio initialization failed.");
    //}
    
    int pin_pos;
    pin_pos = 0;

    initialized_ = true;
    //gpioSetMode(servo_pin_1_, PI_OUTPUT);
    //gpioSetMode(servo_pin_2_, PI_OUTPUT);

    //int servo_pins[] = {4, 18};
    GPIO.setmode(GPIO.BCM);

    GPIO.setup(4, GPIO.OUT) // motor izquierdo
    servo_izquierdo = GPIO.PWM(4, 50) 
    servo_izquierdo.start(0)

    //int servos = []
    //for(pin_pos; pin_pos < 2; pin_pos++){
    //    GPIO.setup(servo_pins[pin_pos], GPIO.OUT)


    //}
        //GPIO.setup(pin, GPIO.OUT)  # Configura el pin como salida
        //servo = GPIO.PWM(pin, 50)  # Inicializa PWM en el pin con frecuencia de 50Hz
        //servo.start(0)  Comienza el PWM con un ciclo de trabajo de 0%
        //self.servos.append(servo)
}

void ArduinoComms::setServo1(int value)
{
    if (initialized_)
    {
        gpioServo(servo_pin_1_, value);
    }
    else
    {
        throw std::runtime_error("Pigpio is not initialized.");
    }
}

void ArduinoComms::setServo2(int value)
{
    if (initialized_)
    {
        gpioServo(servo_pin_2_, value);
    }
    else
    {
        throw std::runtime_error("Pigpio is not initialized.");
    }
}

void ArduinoComms::setServoValues(int value_1, int value_2)
{
    setServo1(value_1);
    setServo2(value_2);
}

bool ArduinoComms::connected() const
{
    return initialized_;
}

void ArduinoComms::cleanup()
{
    if (initialized_)
    {
        gpioTerminate();
        initialized_ = false;
    }
}
