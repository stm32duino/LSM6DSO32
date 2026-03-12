/**
 ******************************************************************************
 * @file    LSM6DSO32_6DOrientation.ino
 * @author  Nathan Garnier, based on LSM6DSO lib.
 * @version V1.0.0
 * @date    March 2026
 * @brief   Arduino test application for the STMicrolectronics
 *          LSM6DSO32 MEMS IMU 6-axis sensor.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 */

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

#define IMU_INT_1 4

LSM6DSO32Sensor accGyr(&DEV_I2C);

//Interrupts.
volatile int mems_event = 0;

char report[256];

void INT1Event_cb();
void sendOrientation();

void setup()
{
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();

  //Int1 pin input
  pinMode(IMU_INT_1, INPUT);

  accGyr.begin();
  accGyr.Enable_X();
  accGyr.Enable_6D_Orientation(LSM6DSO32_INT1_PIN);
}

void loop()
{

  int int_val = digitalRead(IMU_INT_1);
  if (int_val && !mems_event) {
    mems_event = 1;
  }

  if (mems_event) {
    mems_event = 0;
    LSM6DSO32_Event_Status_t status;
    accGyr.Get_X_Event_Status(&status);
    if (status.D6DOrientationStatus) {
      sendOrientation();
      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void sendOrientation()
{
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;

  accGyr.Get_6D_Orientation_XL(&xl);
  accGyr.Get_6D_Orientation_XH(&xh);
  accGyr.Get_6D_Orientation_YL(&yl);
  accGyr.Get_6D_Orientation_YH(&yh);
  accGyr.Get_6D_Orientation_ZL(&zl);
  accGyr.Get_6D_Orientation_ZH(&zh);

  if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1) {
    sprintf(report, "\r\n  __*_____________  " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |________________| " \
            "\r\n    *               \r\n");
  }

  else {
    sprintf(report, "None of the 6D orientation axes is set in LSM6DSO - accelerometer.\r\n");
  }

  SerialPort.print(report);
}
