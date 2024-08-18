#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>


struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 30;
  //std::string device = "/dev/ttyUSB0";
  //int baud_rate = 57600;
  //int timeout = 1000;
  int enc_counts_per_rev = 1920;

  // Pines GPIO para los motores
  int left_motor_pin = 4;  // Pin GPIO para el motor izquierdo
  int right_motor_pin = 18; // Pin GPIO para el motor derecho
};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H