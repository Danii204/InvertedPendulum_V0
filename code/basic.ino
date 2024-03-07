#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 sensor;

// Motor pinout
const int pinPWMA = 13;
const int pinAIN2 = 14;
const int pinAIN1 = 27;
const int pinSTBY = 26;
const int pinBIN1 = 33;
const int pinBIN2 = 25;
const int pinPWMB = 32;


// MPU6050 variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

// PID
float Kp = 8;
float Ki = 0.1;
float Kd = 0.05;
float integral = 0.0;
float derivative = 0.0;
float previous_error = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(22, 23);
  sensor.initialize();
  delay(1000);

  // Motor pinout setup
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(pinSTBY, OUTPUT);
}

void loop() {
  // MPU6050 data reading
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Calculating angles using accelerometer
  float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / PI);
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / PI);

  // Calculating angles using gyroscope
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
  ang_x = 0.98 * (ang_x_prev + (gx / 131.0) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131.0) * dt) + 0.02 * accel_ang_y;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;

  // Inverted pendulum control
  float desired_angle = 0.0;  // Desired angle
  float error = desired_angle - ang_x;  // Error calculation

  integral += error * dt;
  derivative = (error - previous_error) / dt;

  float motor_speed = Kp * error + Ki * integral + Kd * derivative;

  // Motor control
  enableMotors();
  moveMotor(pinPWMA, pinAIN1, pinAIN2, motor_speed);
  moveMotor(pinPWMB, pinBIN1, pinBIN2, motor_speed);

  // Print data
  Serial.print("Rotacion en X: ");
  Serial.print(ang_x);
  Serial.print(", Error: ");
  Serial.print(error);
  Serial.print(", Velocidad de los motores: ");
  Serial.println(motor_speed);

  // Error update
  previous_error = error;

  delay(0.5);
}

// Motor control functions
void moveMotor(int pinPWMA, int pinIN1, int pinIN2, float speed) {
  if (speed > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
    analogWrite(pinPWMA, speed);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    analogWrite(pinPWMA, -speed);
  }
}

void enableMotors() {
  digitalWrite(pinSTBY, HIGH);
}
