#include <Arduino.h>
#include "ESPMotor.hpp"
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

typedef struct test_struct
{
  float distance; // x
  float angle;    // y
  float state;    // z
} test_struct;

test_struct myData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // Serial.print("x: ");
  Serial.print(myData.distance);
  Serial.print(",");
  Serial.print(myData.angle);
  Serial.print(",");
  Serial.print(myData.state);
  Serial.println();
}

// PWM availible on : 2, 4, 5, 12 - 19, 21 - 23, 25 - 27, 32 - 33
//----------------------------------------------------------------
// Motor Pins
#define M1IN1 18
#define M1IN2 19
#define M2IN1 23
#define M2IN2 5
#define M3IN1 16
#define M3IN2 17
#define M4IN1 25
#define M4IN2 32
//----------------------------------------------------------------
// Electromagnet Pins
#define magnet1pin 2
#define magnet2pin 4
#define magnet3pin 27
#define magnet4pin 26

//----------------------------------------------------------------
// Magnet On
//
#define OpenMagnet(magnetpin)      \
  do                               \
  {                                \
    digitalWrite(magnetpin, HIGH); \
  } while (0);

// Magnet Off
#define CloseMagnet(magnetpin)    \
  do                              \
  {                               \
    digitalWrite(magnetpin, LOW); \
  } while (0);

ESPMotor motor1;
ESPMotor motor2;
ESPMotor motor3;
ESPMotor motor4;

void move(int direct, double speed);
void turret(int direct, double speed);

_PID run_pid;
_PID turn_pid;
_PID turret_pid;

_PID final_pid;

float range;

bool Final_flag = 0;
bool PID_flag = 1;

bool Out_flag = false;

unsigned long start_final = 0;
unsigned long end_final = 0;
long time_passed;

MPU6050 mpu6050(Wire); // Creates instance of gyroscope
void TaskBlink(void *pvParameters); // Creates 2nd thread

void setup()
{
  Serial.begin(115200);


  WiFi.mode(WIFI_STA);

  // Initializes espnow
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  pinMode(magnet1pin, OUTPUT); // Set electromagnet pins to output
  pinMode(magnet2pin, OUTPUT); 
  pinMode(magnet3pin, OUTPUT); 
  pinMode(magnet4pin, OUTPUT); 

  digitalWrite(magnet1pin, LOW); // Default off
  digitalWrite(magnet2pin, LOW); 
  digitalWrite(magnet3pin, LOW); 
  digitalWrite(magnet4pin, LOW); 

  motor1.init(M1IN1, M1IN2);
  motor2.init(M2IN1, M2IN2);
  motor3.init(M3IN1, M3IN2);
  motor4.init(M4IN1, M4IN2);

  turn_pid.Kp = -0.02;
  turn_pid.Kd = 0.04;

  run_pid.Kp = -0.005;
  run_pid.Kd = 0.01;

  final_pid.Kp = -0.015;
  final_pid.Kd = 0.04;

  turret_pid.Kp = 0.028;

  turret_pid.set_pid_target(2.50); // Starting position of Turret

  Wire.begin(21, 22, 400000U);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  xTaskCreatePinnedToCore(
      TaskBlink, "TaskBlink" 
      ,
      2048 
      ,
      NULL, 1 
      ,
      NULL, ARDUINO_RUNNING_CORE);

}

void loop()
{
  float Keter;

  float turn_speed;
  float middle_speed;
  float final_speed;

  if (myData.state == 3.0 || myData.state == 4.0)
  {
    OpenMagnet(magnet2pin);

    if (myData.state == 4.0)
    {
      turret_pid.set_pid_target(10.0); // lifts turret: 10deg above horizontal
    }
  }
  else if (myData.state == 5.0)
  {
    CloseMagnet(magnet2pin);
    delay(1000);
    if (Out_flag == false)
    {
      move(1, 0.5);
      delay(1000);
      move(1, 0.0);
      turret_pid.set_pid_target(2.50); 
      Out_flag = true;                 
    }
  }

  else
  {
    CloseMagnet(magnet2pin);
  }

  if (myData.distance > 30)
  {
    PID_flag = 1;
  }
  else if (myData.distance < 30)
  {
    PID_flag = 0;
    motor4.run(0);
    motor3.run(0);
  }

  if (PID_flag) // PID_flag
  {
    if (!Final_flag)
    {
      turn_speed = turn_pid.PID_realize(myData.angle);
      middle_speed = run_pid.PID_realize(myData.distance);

      range = abs(myData.angle);

      if (range >= 90)
      {
        motor4.run(middle_speed * 0.05 + turn_speed * 1.5);
        motor3.run(-(middle_speed)*0.05 + turn_speed * 1.5);
      }

      if (range >= 50 && range < 90)
      {
        motor4.run(middle_speed * 0.1 + turn_speed * 1.2);
        motor3.run(-(middle_speed)*0.1 + turn_speed * 1.2);
      }
      if (range >= 24 && range < 50)
      {
        motor4.run(middle_speed * 0.2 + turn_speed);
        motor3.run(-(middle_speed)*0.2 + turn_speed);
      }
      if (range >= 8 && range < 24)
      {
        motor4.run(middle_speed * 0.4 + turn_speed);
        motor3.run(-(middle_speed)*0.4 + turn_speed);
      }
      if (range >= 2 && range < 8)
      {
        motor4.run(middle_speed * 0.8 + turn_speed);
        motor3.run(-(middle_speed)*0.8 + turn_speed);
      }
      if (range >= 0 && range < 2)
      {
        motor4.run(middle_speed * 1 + turn_speed);
        motor3.run(-(middle_speed)*1 + turn_speed);
      }
    }
  }
  else
  {
  }
}

void TaskBlink(void *pvParameters) 
{
  (void)pvParameters;

  for (;;) 
  {
    delay(20);
    mpu6050.update();
    float turret_speed = turret_pid.PID_realize(mpu6050.getAngleY());
    motor1.run(turret_speed);
    motor2.run(turret_speed);
    unsigned int stackLeft = uxTaskGetStackHighWaterMark(NULL);
  }
}

// 1 forward, 2 right, 3 backward, 4 left//
void move(int direct, double speed)
{
  switch (direct)
  {
  case 1: 
    motor3.run(speed);
    motor4.run(-(speed));
    break;
  case 2:
    motor3.run(speed);
    motor4.run(speed);
    break;
  case 3:
    motor3.run(-(speed));
    motor4.run(speed);
    break;
  case 4:
    motor3.run(-(speed));
    motor4.run(-(speed));
    break;
  default:
    break;
  }
}
