/* 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 * RoboPeak.com
*/
 
#include <RPLidar.h>

RPLidar lidar;

#define RPLIDAR_MOTOR 3
#define obstacleD 155 //155 mm = 15.5 cm

//MOTOR PINS
#define ENA 9   // PWM motorA
#define IN1 4   //control pin1 motorA
#define IN2 5   //control pin2 motorA

#define ENB 10  // PWM motorB
#define IN3 6   //control pin1 motorB
#define IN4 7   //control pin2 motorB
             
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
    float distance = lidar.getCurrentPoint().distance; //distance in mm
    float angle = lidar.getCurrentPoint().angle; //angle in deg
    
    if (distance < obstacleD) {
      if (angle >= 0 && angle < 90) {
        //obstacle fr
        moveLeft();
      } else if (angle >= 90 && angle < 180) {
        //obstacle fl & f
        moveRight();
      }
    } else {
        moveForward();
    }
    
  } else {
    analogWrite(RPLIDAR_MOTOR, 0);
    
    //try to detect RPLIDAR
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    } else {
      stopMotors(); //if no lidar, stop
    }
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void moveLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
}

void moveRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
