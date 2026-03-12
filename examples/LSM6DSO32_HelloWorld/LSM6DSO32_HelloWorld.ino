/**
 ******************************************************************************
 * @file    LSM6DSO32_HelloWorld.ino
 * @author  Nathan Garnier, based on LSM6DSO lib.
 * @version V1.0.0
 * @date    February 2026
 * @brief   Arduino test application for the STMicrolectronics LSM6DSO32
 *          MEMS IMU 6-axis sensor.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 */


// Includes
#include <LSM6DSO32Sensor.h>

#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
  #define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
  #define DEV_I2C Wire
#else
  #define DEV_I2C Wire
#endif
#define SerialPort Serial

// Components
LSM6DSO32Sensor AccGyr(&DEV_I2C);

void setup()
{
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();

  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();
}

void loop()
{
  // Led blinking.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  // Read accelerometer and gyroscope.
  int32_t accelerometer[3];
  int32_t gyroscope[3];
  AccGyr.Get_X_Axes(accelerometer);
  AccGyr.Get_G_Axes(gyroscope);

  // Output data.
  SerialPort.print("| Acc[mg]: ");
  SerialPort.print(accelerometer[0]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[1]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[2]);
  SerialPort.print(" | Gyr[mdps]: ");
  SerialPort.print(gyroscope[0]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope[1]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope[2]);
  SerialPort.println(" |");
}
