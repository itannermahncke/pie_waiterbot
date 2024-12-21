#include "CytronMotorDriver.h"
#include "AccelStepper.h"
#define dirPin 8
#define stepPin 9
#define motorInterfaceType 1

CytronMD left(PWM_DIR, 3, 4);
CytronMD right(PWM_DIR, 11, 12);
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

float WHEEL_DIST = 0.576501;
float lin_vel;
float ang_vel;
float max_vel = 0.272;

float steps_per_rev = 1600;
float steps_per_deg = steps_per_rev / 360;
float gear_ratio = 4;

char buffer[30] = {0};
int code = 0;
float angle = 0;

void setup() {
  Serial.begin(115200);

  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
}

void loop() {
  while (Serial.available())
  {
    byte m = Serial.readBytesUntil('\n', buffer, 30);
    buffer[m] = '\0';

    int code = atoi(strtok(buffer, ","));

    //Serial.println(code);
    if (code == 0)
    {
      lin_vel = atof(strtok(NULL,","));
      ang_vel = atof(strtok(NULL, ","));

      float right_speed = (ang_vel * WHEEL_DIST) / 2 + lin_vel;
      float left_speed = lin_vel*2 - right_speed;

      if (right_speed > max_vel)
      {
        right_speed = max_vel;
      }

      if (left_speed > max_vel)
      {
        left_speed = max_vel;
      }

      if (left_speed < 0)
      {
        left_speed = map(left_speed, -max_vel, 0, -255, 0);
      }
      else
      {
        left_speed = map(left_speed, 0, max_vel, 0 , 255);
      }
    
      if (right_speed < 0)
      {
        right_speed = map(right_speed, -max_vel, 0, -255, 0);
      }
      else
      {
        right_speed = map(right_speed, 0, max_vel, 0, 255);
      }
      
      //if (left_speed < 0.01 && right_speed < 0.01)
      //{
        //left_speed = 0;
        //right_speed = 0;
      //}

      left.setSpeed((int) left_speed);
      right.setSpeed((int) right_speed);

    }
    
    else if (code == 1)
    {
      angle = atof(strtok(NULL, ","));
      //Serial.println(angle);
      int pos = floor(angle * steps_per_deg * gear_ratio * 0.5);
      stepper.moveTo(pos);
      //Serial.println(pos);
      stepper.runToPosition();
    }
  }
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}