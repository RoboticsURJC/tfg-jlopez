/*#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

//#include <serial/serial.h>
#include <cstring>
#include <pigpio.h>

class ArduinoComms
{


public:

  ArduinoComms()
  {  }

  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
      : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
  {  }

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendEmptyMsg();
  void readEncoderValues(int &val_1, int &val_2);
  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return serial_conn_.isOpen(); }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


private:
  serial::Serial serial_conn_;  ///< Underlying serial connection 
};*/
#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <pigpio.h>
#include <stdexcept>

class ArduinoComms
{
public:
  ArduinoComms(int servo_pin_1, int servo_pin_2)
      : servo_pin_1_(servo_pin_1), servo_pin_2_(servo_pin_2), initialized_(false)
  {
    setup();
  }

  ~ArduinoComms()
  {
    cleanup();
  }

  void setup();
  void setServo1(int value);
  void setServo2(int value);
  void setServoValues(int value_1, int value_2);
  bool connected() const;

private:
  void cleanup();

  int servo_pin_1_;  ///< Pin del servo 1
  int servo_pin_2_;  ///< Pin del servo 2
  bool initialized_; ///< Estado de la inicialización de pigpio
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
