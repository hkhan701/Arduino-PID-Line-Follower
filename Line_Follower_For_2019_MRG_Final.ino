#include <QTRSensors.h>
#include <AFMotor.h>

AF_DCMotor m_left(2, MOTOR12_1KHZ);
AF_DCMotor m_right(1, MOTOR12_1KHZ);

float Kp = 2;  //1
float Kd = 5;  //.4

int base_speed = 60;  // vary base motor speed according to voltage
int bot_position = 0;
int error = 0;
unsigned int sensors[8];
int proportional = 0;
int derivative = 0;
int last_proportional = 0;

#define NUM_SENSORS 8
#define TIMEOUT 2500  //waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN 2
#define DEBUG 0

QTRSensorsRC qtrrc((unsigned char[]){ A0, A1, A2, A3, A4, A5, 9, 10 }, NUM_SENSORS, TIMEOUT,
                   EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup() {
  // Serial.begin(1000);
  // auto_calibration();
  bot_stop();
  delay(1000);
}

void loop() {
  m_left.setSpeed(255);
  m_right.setSpeed(255);
  m_left.run(FORWARD);
  m_right.run(FORWARD);
  pid_calc();
  motor_drive();
}

void auto_calibration() {
  for (int i = 0; i < 250; i++) {
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
    //if(i < 50) left();
    //else if (i > 50 && i < 120) right();
    //else if (i > 120 && i < 170) left();
  }

  if (DEBUG) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      qtrrc.calibratedMinimumOn[i];
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
      qtrrc.calibratedMaximumOn[i];
    }
  }
}

void pid_calc() {
  bot_position = qtrrc.readLine(sensors);
  Serial.println(bot_position);
  proportional = bot_position - 2500;
  derivative = proportional - last_proportional;
  last_proportional = proportional;

  error = (Kp * proportional) + (Kd * derivative);
  Serial.println(error);
}

void motor_drive() {
  int left_motor_speed;
  int right_motor_speed;

  left_motor_speed = base_speed - error;
  right_motor_speed = base_speed + error;

  if (left_motor_speed < 0) left_motor_speed = 0;
  else if (left_motor_speed > 150) left_motor_speed = 150;

  if (right_motor_speed < 0) right_motor_speed = 0;
  else if (right_motor_speed > 150) right_motor_speed = 150;

  Serial.print(left_motor_speed);
  Serial.print(" ");
  Serial.println(right_motor_speed);

  m_left.setSpeed(left_motor_speed);
  m_right.setSpeed(right_motor_speed);
  m_left.run(FORWARD);
  m_right.run(FORWARD);
}

void left() {
  m_left.setSpeed(255);
  m_right.setSpeed(255);
  m_left.run(BACKWARD);
  m_right.run(FORWARD);
}

void right() {
  m_left.setSpeed(255);
  m_right.setSpeed(255);
  m_left.run(FORWARD);
  m_right.run(BACKWARD);
}

void bot_stop() {
  m_left.setSpeed(0);
  m_right.setSpeed(0);
  m_left.run(FORWARD);
  m_right.run(FORWARD);
}
