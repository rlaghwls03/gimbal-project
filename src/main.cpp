#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <AccelStepper.h>

// =============================================
// 스텝모터 설정 (3축)
// =============================================
AccelStepper rollMotor (AccelStepper::HALF4WIRE, 22, 24, 23, 25);
AccelStepper pitchMotor(AccelStepper::HALF4WIRE, 26, 28, 27, 29);
AccelStepper yawMotor  (AccelStepper::HALF4WIRE, 30, 32, 31, 33);

// =============================================
// MPU-6050
// =============================================
MPU6050 mpu;

// =============================================
// 상보 필터 (alpha ↑ = 가속도 진동 노이즈 ↓)
// =============================================
float rollAngle  = 0.0;
float pitchAngle = 0.0;
float yawAngle   = 0.0;   // 자이로 적분 기반
float alpha = 0.98;

// =============================================
// PID (진동 둔감화 + 오버슈트 방지)
// =============================================
float Kp = 1.2;   // 0.8 → 1.2 (반응성 ↑)
float Ki = 0.0;
float Kd = 0.05;

// 작은 오차 무시
const float DEADZONE_DEG = 2.0;
// 출력 제한 (모터 회전량 상한)
const float MAX_OUTPUT_DEG = 20.0;  // 15 → 20

// 모터 게인 (PID 출력 → 실제 모터 회전 비율)
const float MOTOR_GAIN = 0.6;       // 0.5 → 0.6

float rollError_prev  = 0.0;
float pitchError_prev = 0.0;
float yawError_prev   = 0.0;
float rollIntegral    = 0.0;
float pitchIntegral   = 0.0;
float yawIntegral     = 0.0;

const float STEPS_PER_DEG = 4096.0 / 360.0;

unsigned long lastTime = 0;

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();
    if (mpu.testConnection()) {
        Serial.println("MPU-6050 OK");
    } else {
        Serial.println("MPU-6050 FAIL");
        while (1);
    }

    rollMotor.setMaxSpeed(600);
    rollMotor.setAcceleration(400);
    pitchMotor.setMaxSpeed(600);
    pitchMotor.setAcceleration(400);
    yawMotor.setMaxSpeed(600);
    yawMotor.setAcceleration(400);

    lastTime = millis();
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt < 0.01) {
        rollMotor.run();
        pitchMotor.run();
        yawMotor.run();
        return;
    }
    lastTime = now;

    // --- MPU-6050 ---
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accelRoll  = atan2(ay, az) * 180.0 / PI;
    float accelPitch = atan2(-ax, sqrt((float)ay*ay + (float)az*az)) * 180.0 / PI;

    float gyroRoll  = gx / 131.0;
    float gyroPitch = gy / 131.0;
    float gyroYaw   = gz / 131.0;

    // 상보 필터 (Yaw는 가속도 기준 없음 → 자이로 적분만)
    rollAngle  = alpha * (rollAngle  + gyroRoll  * dt) + (1 - alpha) * accelRoll;
    pitchAngle = alpha * (pitchAngle + gyroPitch * dt) + (1 - alpha) * accelPitch;
    yawAngle  += gyroYaw * dt;

    // --- PID (Roll) ---
    float rollError = -rollAngle;
    if (fabs(rollError) < DEADZONE_DEG) rollError = 0;
    rollIntegral += rollError * dt;
    float rollD = (rollError - rollError_prev) / dt;
    float rollOutput = Kp * rollError + Ki * rollIntegral + Kd * rollD;
    rollOutput = clampf(rollOutput, -MAX_OUTPUT_DEG, MAX_OUTPUT_DEG);
    rollError_prev = rollError;

    // --- PID (Pitch) ---
    float pitchError = -pitchAngle;
    if (fabs(pitchError) < DEADZONE_DEG) pitchError = 0;
    pitchIntegral += pitchError * dt;
    float pitchD = (pitchError - pitchError_prev) / dt;
    float pitchOutput = Kp * pitchError + Ki * pitchIntegral + Kd * pitchD;
    pitchOutput = clampf(pitchOutput, -MAX_OUTPUT_DEG, MAX_OUTPUT_DEG);
    pitchError_prev = pitchError;

    // --- PID (Yaw) ---
    float yawError = -yawAngle;
    if (fabs(yawError) < DEADZONE_DEG) yawError = 0;
    yawIntegral += yawError * dt;
    float yawD = (yawError - yawError_prev) / dt;
    float yawOutput = Kp * yawError + Ki * yawIntegral + Kd * yawD;
    yawOutput = clampf(yawOutput, -MAX_OUTPUT_DEG, MAX_OUTPUT_DEG);
    yawError_prev = yawError;

    // --- 모터 명령 (MOTOR_GAIN으로 추가 감쇠) ---
    rollMotor.moveTo ((long)(-rollOutput  * MOTOR_GAIN * STEPS_PER_DEG));
    pitchMotor.moveTo((long)(-pitchOutput * MOTOR_GAIN * STEPS_PER_DEG));
    yawMotor.moveTo  ((long)(-yawOutput   * MOTOR_GAIN * STEPS_PER_DEG));

    rollMotor.run();
    pitchMotor.run();
    yawMotor.run();
}
