#include <RPLidar.h>

RPLidar lidar;

#define RPLIDAR_MOTOR 3
#define obstacleD 20

#define ENA 9
#define IN1 4
#define IN2 5 

#define ENB 10
#define IN3 6
#define IN4 7

bool obstacleF = false;
bool obstacleR = false;
bool obstacleL = false;

void setup() {
  lidar.begin(Serial);

  pinMode(RPLIDAR_MOTOR, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;

    obstacleF = false;
    obstacleR = false;
    obstacleL = false;

    if (distance <= obstacleD) {
      if ((angle >= 350 && angle <= 360) || (angle <= 10 && angle >= 0)) {
        obstacleF = true;
      } else if (angle > 10 && angle <= 90) {
        obstacleR = true;
      } else if (angle >= 270 && angle < 350) {
        obstacleL = true;
      }
    }

    if (obstacleF) {
      if (obstacleL && obstacleR) {
        moveBack();
      } else if (obstacleL) {
        turnRight();
      } else {
        turnLeft();
      }
    } else {
      if (obstacleL) {
        turnRight();
      } else if (obstacleR) {
        turnLeft();
      } else {
        moveForward();
      }
    }
  } else {
    analogWrite(RPLIDAR_MOTOR, 0);

    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      lidar.startScan();
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(1000);
    } else {
      stopMotors();
    }
  }
  delay(10);
}

void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 60);
  analogWrite(ENB, 60);
}

void moveBack() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 60);
  analogWrite(ENB, 60);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
