#include <Arduino.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Wire.h>

MPU6050 mpu;
Servo servo1;

float roll = 0;
unsigned long lastTime = 0;
const float alpha = 0.96;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();
    servo1.attach(9);
    lastTime = millis();
    Serial.println("시작");
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Roll만 계산
    float accelRoll = atan2(ax, az) * 180.0 / PI;
    float gyroRoll  = gy / 131.0;
    roll = alpha * (roll + gyroRoll * dt) + (1 - alpha) * accelRoll;

    // 서보 각도: 90도 기준으로 roll 보정
    int servoAngle = constrain(90 - (int)roll, 0, 180);
    servo1.write(servoAngle);

    Serial.print("Roll: "); Serial.print(roll);
    Serial.print("  Servo: "); Serial.println(servoAngle);

    delay(500);
}