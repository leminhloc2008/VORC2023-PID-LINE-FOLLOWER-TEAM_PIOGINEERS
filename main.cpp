#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

#define mode 8

#define left_motor_channel_a 0
#define left_motor_channel_b 1
#define right_motor_channel_a 2
#define right_motor_channel_b 3

#define SENSOR_1_PIN 4
#define SENSOR_2_PIN 3
#define SENSOR_3_PIN 1
#define SENSOR_4_PIN 0

#define W_LED_ON 20
#define IR_LED_ON 21

// #define threshold 2000

/* ----------- PID -----------*/
double lastError = 0;
double lightValue = 0;

double Kp = 0.14;
double Ki = 0;
double Kd = 4.2;
double Kr = 1.1;

double P = 0;
double I = 0;
double D = 0;

double avg = 0;
double sum = 0;
double division = 0;

long long value = 0;
long total = 0;
/* --------------------------*/

/* ----------Speed--------------*/
int baseSpeedValue = 255;

uint8_t vspeed = 245;  // 50
uint8_t tspeed = 245;  // 70

uint8_t vspeed_l = vspeed;
uint8_t tspeed_l = tspeed;
uint8_t vspeed_r = vspeed * 2.5;
uint8_t tspeed_r = tspeed * 2.5;

int maxSpeed = 255;
/* ----------------------------*/

/* ----------- Sensor------------*/
int onLine = 1;
int button_state = 0;
int button_pin = 7;

int sensor[4], threshold[4];
int sensorMin[4] = {255, 255, 255, 255}, sensorMax[4] = {0, 0, 0, 0};
/* ------------------------------*/

void setup() {
  Serial.begin(115200);

  ledcSetup(left_motor_channel_a, 5000, mode);
  ledcSetup(left_motor_channel_b, 5000, mode);
  ledcSetup(right_motor_channel_a, 5000, mode);
  ledcSetup(right_motor_channel_b, 5000, mode);

  ledcAttachPin(PWM_PIN_L_A, left_motor_channel_a);
  ledcAttachPin(PWM_PIN_L_B, left_motor_channel_b);
  ledcAttachPin(PWM_PIN_R_A, right_motor_channel_a);
  ledcAttachPin(PWM_PIN_R_B, right_motor_channel_b);

  pinMode(W_LED_ON, OUTPUT);
  pinMode(IR_LED_ON, OUTPUT);
  digitalWrite(W_LED_ON, 0);
  digitalWrite(IR_LED_ON, 1);
}

void calibration() {
  sensor[0] = analogRead(SENSOR_1_PIN);
  sensor[1] = analogRead(SENSOR_2_PIN);
  sensor[2] = analogRead(SENSOR_3_PIN);
  sensor[3] = analogRead(SENSOR_4_PIN);

  for (int j = 0; j < 12000; j++) {
    ledcWrite(left_motor_channel_a, 0);
    ledcWrite(left_motor_channel_b, 150);
    ledcWrite(right_motor_channel_a, 150);
    ledcWrite(right_motor_channel_b, 0);
    for (int i = 0; i < 4; i++) {
      if (sensor[i] > sensorMax[i]) {
        sensorMax[i] = sensor[i];
      }
      if (sensor[i] < sensorMin[i]) {
        sensorMin[i] = sensor[i];
      }
    }
  }

  for (int i = 0; i < 4; i++) {
    threshold[i] = (sensorMax[i] + sensorMin[i]) / 2;
  }
}

void forwardMovement(int speedA, int speedB) {
  if (speedA < 0) {
    speedA = 0 - speedA;
    ledcWrite(left_motor_channel_a, 0);
    ledcWrite(left_motor_channel_b, speedA);
  } 
  else {
    ledcWrite(left_motor_channel_a, speedA);
    ledcWrite(left_motor_channel_b, 0);
  }
  if (speedB < 0) {
    speedB = 0 - speedB;
    ledcWrite(right_motor_channel_a, 0);
    ledcWrite(right_motor_channel_b, speedB);
  } 
  else {
    ledcWrite(right_motor_channel_a, speedB);
    ledcWrite(right_motor_channel_b, 0);
  }
  // Serial.println("forward");
}

int pid = 1;

void getLine() {
  avg = 0;
  sum = 0;
  onLine = 0;
  total = 0;

  for (int i = 0; i < 4; i++) {
    if (sensor[i] < sensorMin[i]) {
      sensor[i] = 0;
      value = 4095;
    }
    else if (sensor[i] > sensorMax[i]) {
      sensor[i] = 4095;
      value = 0;
    }
    else {
      division = ((4095) / (double) (sensorMax[i] - sensorMin[i]));
      sensor[i] = (long)((long) sensor[i] - sensorMin[i] * (division));
      value = (4095 - sensor[i]);
    }

    if (value > 1500) onLine = 1;
    avg = (avg + (double)(value) * ((double)(i) * 1000));
    sum = sum + (double)value;

    total = total + value;
  }

  if (onLine == 0) {
    if (lightValue < 1500) {
      lightValue = 0;
      total = 0;
      return;
    }
    else {
      lightValue = 3000;
      total = 0;
      return;
    }
  }
  lightValue = (double)((avg) / (sum));
  lightValue = (long)lightValue;
}

void Pid_control() {
  int error, motorSpeedA, motorSpeedB, motorSpeedChange;

  // error = (analogRead(SENSOR_1_PIN) - analogRead(SENSOR_4_PIN));
  error = (lightValue - 1500);

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error;

  motorSpeedChange = P * Kp + I * Ki + D * Kp;

  motorSpeedA = baseSpeedValue + motorSpeedChange;
  motorSpeedB = baseSpeedValue - motorSpeedChange;

  if (motorSpeedChange < 0) {
    motorSpeedA = baseSpeedValue + motorSpeedChange;
    motorSpeedB = baseSpeedValue - (Kr * motorSpeedChange);
  }
  else {
    motorSpeedA = baseSpeedValue + (Kr * motorSpeedChange);
    motorSpeedB = baseSpeedValue - motorSpeedChange;
  }
  if (motorSpeedA > maxSpeed) {
    motorSpeedA = maxSpeed;
  }
  if (motorSpeedB > maxSpeed) {
    motorSpeedB = maxSpeed;
  }
  if (motorSpeedA < -maxSpeed) {
    motorSpeedA = -maxSpeed;
  }
  if (motorSpeedB < -maxSpeed) {
    motorSpeedB = -maxSpeed;
  }
  forwardMovement(motorSpeedA, motorSpeedB);
}

void empty() {
  // empty func
}

void loop() {
  int i;
  delay(1000);

  for (int i = 0; i < 10; i++) {
    delay(100);
    digitalWrite(W_LED_ON, HIGH);
    delay(100);
    digitalWrite(W_LED_ON, LOW);
  }
  digitalWrite(W_LED_ON, HIGH);

  // Calibration Start
  delay(4000);
  calibration();
  delay(4000);
  while (button_state == LOW);
  while (button_state == HIGH) {
    button_state = digitalRead(button_pin);
    empty();
  }
  // Calibration Done

  //digitalWrite(W_LED_ON, LOW);
  // Start Line Following
  while (1) {
    sensor[0] = analogRead(SENSOR_1_PIN);
    sensor[1] = analogRead(SENSOR_2_PIN);
    sensor[2] = analogRead(SENSOR_3_PIN);
    sensor[3] = analogRead(SENSOR_4_PIN);

    if (sensor[1] <= threshold[1] && sensor[2]<= threshold[2] && sensor[0] <= threshold[0] && sensor[3] <= threshold[3]) {
      forwardMovement(-baseSpeedValue, baseSpeedValue);
      delay(170);
    }
    else {
      getLine();
      Pid_control();
    }
  }
}
