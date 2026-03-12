/**
 ******************************************************************************
 * @file    LSM6DSO32_Pedometer.ino
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

uint16_t step_count = 0;
uint32_t previous_tick;

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
  accGyr.Enable_Pedometer();

  previous_tick = millis();
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
    if (status.StepStatus) {
      // New step detected, so print the step counter
      accGyr.Get_Step_Count(&step_count);
      snprintf(report, sizeof(report), "Step counter: %d", step_count);
      SerialPort.println(report);

      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  // Print the step counter in any case every 3000 ms
  uint32_t current_tick = millis();
  if ((current_tick - previous_tick) >= 3000) {
    accGyr.Get_Step_Count(&step_count);
    snprintf(report, sizeof(report), "Step counter: %d", step_count);
    SerialPort.println(report);
    previous_tick = millis();
  }

}