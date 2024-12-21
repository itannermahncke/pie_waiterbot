#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int red_led = 11;
const int red_button = 10;

const int green_led = 9;
const int green_button = 8;

const int blue_led = 7;
const int blue_button = 6;

bool red_state = 0;
bool green_state = 0;
bool blue_state = 0;

int start_time;
int imu_timer;

Adafruit_ICM20948 imu;
float lin_accel = 0;
float ang_accel = 0;

const int left_leading_pin = 2;
const int left_following_pin = 3;
const int right_leading_pin = 12;
const int right_following_pin = 13;

float pulses_per_rev = 1425.1;
float belt_reduction = 0.5;
float wheel_diameter = 3.5;
float wheel_dist = 0.576501;

int left_pulse_count = 0;
int right_pulse_count = 0;

int left_motor_dir = 1;
int right_motor_dir = 1;

float lin_vel = 0;
float ang_vel = 0;

volatile int left_lastEncoded = 0;
volatile int right_lastEncoded = 0;

float sampling_time = 100;
unsigned long encoder_timer = millis();

void setup() {
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led, OUTPUT);

  pinMode(red_button, INPUT);
  pinMode(green_button, INPUT);
  pinMode(blue_button, INPUT);

  Serial.begin(115200);

  imu.begin_I2C();

  uint16_t accel_divisor = imu.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  uint16_t gyro_divisor = imu.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  pinMode(left_leading_pin, INPUT);
  pinMode(left_following_pin, INPUT);
  pinMode(right_leading_pin, INPUT);
  pinMode(right_following_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(left_leading_pin), updateLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_following_pin),updateLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_leading_pin), updateRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_following_pin), updateRight, CHANGE);

}

void loop() {
  start_time = millis();

  while (digitalRead(red_button) == HIGH)
  {
    if ((millis() - start_time > 500))
    {
      if (red_state == 0)
      {
        digitalWrite(red_led, HIGH);
        red_state = 1;
        Serial.print("01");
        Serial.print("rb");
        Serial.println(red_state);
        //Serial.println(button_state);
      }
      else if (red_state == 1)
      {
        digitalWrite(red_led, LOW);
        red_state = 0;
        Serial.print("01");
        Serial.println("rb1");
        //Serial.println(button_state);
      }

      break;
    }
  }

  while (digitalRead(green_button) == HIGH)
  {
    if (millis() - start_time > 500)
    {
      if (green_state == 0)
      {
        digitalWrite(green_led, HIGH);
        green_state = 1;
        Serial.print("01");
        Serial.print("gb");
        Serial.println(green_state);
        //Serial.println(button_state);
      }
      else if (green_state == 1)
      {
        digitalWrite(green_led, LOW);
        green_state = 0;
        Serial.print("01");
        Serial.println("gb1");
        //Serial.println(button_state);
      }
      break;
    }
  }

  while (digitalRead(blue_button) == HIGH)
  {
    if (millis() - start_time > 500)
    {
      if (blue_state == 0)
      {
        digitalWrite(blue_led, HIGH);
        blue_state = 1;
        Serial.print("01");
        Serial.print("bb");
        Serial.println(blue_state);
        //Serial.println(button_state);
      }
      else if (blue_state == 1)
      {
        digitalWrite(blue_led, LOW);
        blue_state = 0;
        Serial.print("01");
        Serial.println("bb1");
        //Serial.println(button_state);
      }
      break;
    }
  }

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag;
  imu.getEvent(&accel, &gyro, &temp, &mag);
  lin_accel = accel.acceleration.x;
  ang_accel = gyro.gyro.z;

  if (millis() - imu_timer > 100)
  {
    Serial.print("01");
    Serial.print("mu");
    Serial.print(lin_accel);
    Serial.print(",");
    Serial.println(ang_accel);
    
    imu_timer = millis();
  }
  
  unsigned long current_time = millis();
  if ((current_time - encoder_timer) > sampling_time)
  {
    encoder_timer = millis();
    float left_speed = speed(left_pulse_count);
    float left_velocity = left_speed * left_motor_dir;

    float right_speed = speed(right_pulse_count);
    float right_velocity = right_speed * right_motor_dir;

    left_pulse_count = 0;
    right_pulse_count = 0;

    lin_vel = (left_velocity + right_velocity) / 2;
    ang_vel = ((right_velocity - left_velocity)) / wheel_dist;
    
    Serial.print("01");
    Serial.print("en");
    Serial.print(lin_vel,4);
    Serial.print(",");
    Serial.println(ang_vel,4);

  }
}


float speed(int pulses)
{
  float motor_speed = pulses / (pulses_per_rev * (sampling_time/1000));
  float wheel_speed = motor_speed * belt_reduction * 0.25;

  return wheel_speed;
}

void updateLeft() {

  left_pulse_count = left_pulse_count + 1;

  int left_MSB = digitalRead(left_leading_pin); // Most Significant Bit
  int left_LSB = digitalRead(left_following_pin); // Least Significant Bit

  int left_encoded = (left_MSB << 1) | left_LSB; // Combine A and B signals
  int left_sum = (left_lastEncoded << 2) | left_encoded; // Track transitions

  // Determine rotation direction and increment/decrement position
  if (left_sum == 0b1101 || left_sum == 0b0100 || left_sum == 0b0010 || left_sum == 0b1011) {
    left_motor_dir = -1;
  } 
  else if (left_sum == 0b1110 || left_sum == 0b0111 || left_sum == 0b0001 || left_sum == 0b1000) {
    left_motor_dir = 1;
  }

  left_lastEncoded = left_encoded; // Update last encoded value
}

void updateRight() {

  right_pulse_count = right_pulse_count + 1;

  int right_MSB = digitalRead(right_leading_pin); // Most Significant Bit
  int right_LSB = digitalRead(right_following_pin); // Least Significant Bit

  int right_encoded = (right_MSB << 1) | right_LSB; // Combine A and B signals
  int right_sum = (right_lastEncoded << 2) | right_encoded; // Track transitions

  // Determine rotation direction and increment/decrement position
  if (right_sum == 0b1101 || right_sum == 0b0100 || right_sum == 0b0010 || right_sum == 0b1011) {
    right_motor_dir = -1;
  } 
  else if (right_sum == 0b1110 || right_sum == 0b0111 || right_sum == 0b0001 || right_sum == 0b1000) {
    right_motor_dir = 1;
  }

  right_lastEncoded = right_encoded; // Update last encoded value
}