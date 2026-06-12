#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <AccelStepper.h>

// =============================================
// 스텝모터 설정 (3축)
// =============================================
AccelStepper rollMotor (AccelStepper::HALF4WIRE, 48, 50, 49, 51);
AccelStepper pitchMotor(AccelStepper::HALF4WIRE, 36, 37, 38, 39);
AccelStepper yawMotor  (AccelStepper::HALF4WIRE, 24, 26, 25, 27);

// =============================================
// MPU-6050
// =============================================
MPU6050 mpu;

// =============================================
// ★ 축 방향/매핑 설정 — 하드웨어에 맞춰 여기만 바꾸면 됨
// =============================================
// pitch를 기울였는데 시리얼의 R:값이 변하면 → SWAP_ROLL_PITCH 를 true 로.
// (roll/pitch 센서축이 모터축과 뒤바뀐 경우 한 줄로 교정)
const bool SWAP_ROLL_PITCH = false;

// 모터가 보정 방향과 반대로 돌면 해당 invert 를 뒤집으세요.
// 기존 코드가 세 축 모두 -output 이었으므로 기본 true.
const bool INVERT_ROLL  = true;
const bool INVERT_PITCH = true;
const bool INVERT_YAW   = true;

// =============================================
// 상보 필터 (alpha ↑ = 가속도 진동 노이즈 ↓)
// =============================================
float rollAngle  = 0.0;
float pitchAngle = 0.0;
float yawAngle   = 0.0;   // 자이로 적분 기반
// alpha↑ = 자이로를 더 신뢰 → 모터 진동이 가속도로 새어 다른 축이 흔들리는 것 ↓
const float alpha = 0.99;

// =============================================
// PID 게인 (축별)
// =============================================
// 데모용 — 적당히 반응하되 과하지 않게 톤다운.
float Kp_roll  = 6.0;
float Kp_pitch = 3.8;   // pitch 회전량 살짝 ↓ (4.5 → 3.8)
float Kp_yaw   = 5.6;

// 적분 게인 — 정상상태 오차 제거용(작게). 0이면 적분 미사용.
float Ki_roll  = 0.0;
float Ki_pitch = 0.0;
float Ki_yaw   = 0.0;

// 미분 게인 — ★ 오차 미분이 아니라 '자이로 각속도(측정값)'에 곱함.
//   → setpoint 변화/deadzone 스냅에서 생기는 미분 킥(떨림) 제거.
float Kd_roll  = 0.10;
float Kd_pitch = 0.08;   // 자이로 노이즈 주입 ↓ → pitch 떨림 완화
float Kd_yaw   = 0.04;   // yaw 떨림 더 완화 (0.10 → 0.06)

// 작은 오차 무시 (떨림 방지)
const float DEADZONE_DEG = 1.0;   // 미세 떨림 무시 (진동 억제)
// 출력 제한 (모터 회전량 상한, deg) — 큰 기울기도 충분히 보정하도록 ↑
const float MAX_OUTPUT_DEG = 60.0;
// 적분 와인드업 방지 클램프 (적분항 누적 상한)
const float INTEGRAL_MAX = 30.0;

// 모터 게인 (PID 출력 → 실제 모터 회전 비율)
// 0.9 = 살짝 덜 보정(과보정에 의한 진동 완화). 더 차분하게는 0.7~0.8.
const float MOTOR_GAIN = 0.9;

// 모터 목표 데드밴드(스텝) — 목표가 이만큼 이상 바뀔 때만 다시 명령.
// 매 루프 미세하게 목표가 흔들려 스텝모터가 출발/정지 반복하는 '떨림' 방지.
// STEPS_PER_DEG ≈ 11.4 이므로 6스텝 ≈ 0.5°. 더 차분하게는 10~12로.
const long MOTOR_DEADBAND_STEPS = 6;

const float STEPS_PER_DEG = 4096.0 / 360.0;

// =============================================
// 목표각(setpoint) — 전원 켤 때의 자세를 기준으로 저장
// =============================================
float rollSetpoint  = 0.0;
float pitchSetpoint = 0.0;
float yawSetpoint   = 0.0;

// 적분 누적값
float rollIntegral  = 0.0;
float pitchIntegral = 0.0;
float yawIntegral   = 0.0;

// 자이로 바이어스(영점 오차) — 시작 시 정지 상태에서 측정해 표류 제거
float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

// =============================================
// ★ YAW 드리프트 억제 (지자기 센서 없이 자이로만 쓸 때)
// =============================================
// (1) 데드밴드: 정지 상태에서 미세 노이즈를 적분하지 않도록 무시
const float YAW_GYRO_DEADBAND = 0.30;//미세 자이로 노이즈 무시 → yaw 떨림 ↓ (0.25 → 0.35)
// (2) 복원(leak): 추정 각도를 시작 방위(0)로 천천히 끌어당김
//   평형 yaw ≈ (잔류바이어스 × dt) / (1 - YAW_LEAK)
//   MPU6050은 잔류 바이어스가 커서(수 °/s) leak이 약하면 큰 각도에 박혀 드리프트함.
//   강할수록(값↓) 0 근처로 복원되지만 실제 회전 유지 시간(시정수)도 짧아짐.
//   0.99 ≈ 시정수 약 1~2초: 빠른 외란은 잡고 느린 드리프트는 0으로 복원.
//   yaw 절대 방위를 더 오래 유지하려면 0.995~0.998로(단, 드리프트 다시 증가).
const float YAW_LEAK = 0.99;

unsigned long lastTime = 0;

// 시작 시 기준 자세에 고정한 채 잠깐 대기 (보정 시작 전 안정화)
const unsigned long START_HOLD_MS = 1000;   // 약 1초
unsigned long startHoldUntil = 0;

// computePID에서 dt를 쓰기 위한 전역 (loop에서 매 주기 갱신)
float dt_global = 0.0;

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 가속도계로 roll/pitch 추정 (SWAP_ROLL_PITCH 반영)
void computeAccelAngles(int16_t ax, int16_t ay, int16_t az,
                        float &outRoll, float &outPitch) {
    // 분모를 둘 다 sqrt(다른 두 축)로 → roll↔pitch 상호 커플링 최소화(디커플링)
    float aRoll  = atan2(-ax, sqrt((float)ay*ay + (float)az*az)) * 180.0 / PI;
    float aPitch = atan2( ay, sqrt((float)ax*ax + (float)az*az)) * 180.0 / PI;
    if (SWAP_ROLL_PITCH) { float t = aRoll; aRoll = aPitch; aPitch = t; }
    outRoll  = aRoll;
    outPitch = aPitch;
}

// 한 축 PID 계산
//  angle   : 현재 추정각
//  setpoint: 목표각
//  rate    : 해당 축 자이로 각속도(deg/s) — 미분항(측정값 미분)에 사용
//  integral: 적분 누적값(참조로 갱신)
float computePID(float setpoint, float angle, float rate,
                 float Kp, float Ki, float Kd, float &integral) {
    float error = setpoint - angle;
    if (fabs(error) < DEADZONE_DEG) error = 0;

    // 적분 (anti-windup: 누적 상한 클램프)
    integral += error * dt_global;
    integral = clampf(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

    // ★ 미분은 오차가 아닌 각속도(측정값)에 -Kd → 미분 킥 제거 + 감쇠
    float output = Kp * error + Ki * integral - Kd * rate;
    return clampf(output, -MAX_OUTPUT_DEG, MAX_OUTPUT_DEG);
}

void driveMotor(AccelStepper &motor, float output, bool invert) {
    float dir = invert ? -1.0 : 1.0;
    long target = (long)(dir * output * MOTOR_GAIN * STEPS_PER_DEG);
    // 목표가 데드밴드 이상 바뀔 때만 갱신 → 미세 채터링(떨림) 방지
    long diff = target - motor.targetPosition();
    if (diff < 0) diff = -diff;
    if (diff < MOTOR_DEADBAND_STEPS) return;
    motor.moveTo(target);
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

    // 28BYJ-48은 기어드라 느림 — 안전 최고속 ~1000 step/s. 더 올리면 탈조(스킵).
    // 탈조하면 0점 기준이 어긋나 '가끔 반대로' 도는 원인이 됨 → 속도/가속 보수적으로.
    rollMotor.setMaxSpeed(900);
    rollMotor.setAcceleration(4000);
    pitchMotor.setMaxSpeed(900);
    pitchMotor.setAcceleration(4000);
    yawMotor.setMaxSpeed(900);
    yawMotor.setAcceleration(4000);

    // --- 자이로 바이어스 보정 (센서 정지 상태 유지!) ---
    Serial.println("Calibrating gyro... keep still");
    const int N = 1000;
    long sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < N; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumX += gx; sumY += gy; sumZ += gz;
        delay(1);
    }
    gyroBiasX = (sumX / (float)N) / 131.0;
    gyroBiasY = (sumY / (float)N) / 131.0;
    gyroBiasZ = (sumZ / (float)N) / 131.0;
    Serial.print("Gyro bias  X:"); Serial.print(gyroBiasX);
    Serial.print(" Y:"); Serial.print(gyroBiasY);
    Serial.print(" Z:"); Serial.println(gyroBiasZ);

    // --- 초기 정렬: 현재(전원 켤 때) 자세를 측정해 기준으로 저장 ---
    {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float accelRoll, accelPitch;
        computeAccelAngles(ax, ay, az, accelRoll, accelPitch);

        // 각도 추정값을 실제 기울기로 초기화 → 시작 시 덜컹임 제거
        rollAngle  = accelRoll;
        pitchAngle = accelPitch;
        yawAngle   = 0.0;

        // 이 자세를 목표로 저장 → 시작 자세를 유지
        rollSetpoint  = rollAngle;
        pitchSetpoint = pitchAngle;
        yawSetpoint   = yawAngle;   // 0 (시작 방위 유지)

        // 모터 위치도 0점으로 정렬 (기구를 중앙에 둔 상태로 켜세요)
        rollMotor.setCurrentPosition(0);
        pitchMotor.setCurrentPosition(0);
        yawMotor.setCurrentPosition(0);

        Serial.print("Init pose  R:"); Serial.print(rollSetpoint);
        Serial.print(" P:"); Serial.print(pitchSetpoint);
        Serial.print(" Y:"); Serial.println(yawSetpoint);
    }

    // 기준 자세 고정 후 약 1초간 대기 시작
    startHoldUntil = millis() + START_HOLD_MS;
    Serial.println("Holding start pose...");

    lastTime = millis();
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt < 0.01) {
        // 제어 주기 사이에도 모터 스텝은 계속 진행
        rollMotor.run();
        pitchMotor.run();
        yawMotor.run();
        return;
    }
    lastTime = now;
    dt_global = dt;

    // --- MPU-6050 ---
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accelRoll, accelPitch;
    computeAccelAngles(ax, ay, az, accelRoll, accelPitch);

    // 바이어스 빼서 표류 제거
    float gyroRoll  = gy / 131.0 - gyroBiasY;
    float gyroPitch = gx / 131.0 - gyroBiasX;
    float gyroYaw   = gz / 131.0 - gyroBiasZ;
    if (SWAP_ROLL_PITCH) { float t = gyroRoll; gyroRoll = gyroPitch; gyroPitch = t; }

    // YAW 데드밴드: 정지 시 미세 노이즈는 적분하지 않음 → 드리프트 억제
    if (fabs(gyroYaw) < YAW_GYRO_DEADBAND) gyroYaw = 0.0;

    // 상보 필터 (Roll/Pitch는 가속도로 보정, Yaw는 자이로 적분 + 약한 leak)
    rollAngle  = alpha * (rollAngle  + gyroRoll  * dt) + (1 - alpha) * accelRoll;
    pitchAngle = alpha * (pitchAngle + gyroPitch * dt) + (1 - alpha) * accelPitch;
    yawAngle   = (yawAngle + gyroYaw * dt) * YAW_LEAK;   // leak으로 0(시작방위)로 천천히 복원

    // 디버그 출력은 솎아서 (매 루프 출력하면 루프가 느려져 스텝모터가 끊기고 dt↑)
    static uint8_t printCnt = 0;
    if (++printCnt >= 10) {
        printCnt = 0;
        Serial.print("R:");  Serial.print(rollAngle);
        Serial.print(" P:"); Serial.print(pitchAngle);
        Serial.print(" Y:"); Serial.println(yawAngle);
    }

    // --- 시작 직후 고정 대기 (약 1초) : 보정 없이 기준 자세에 멈춰 있음 ---
    if (millis() < startHoldUntil) {
        // 모터는 0점 유지, 적분 누적 방지 (튐 없이 곧바로 제어 진입)
        rollIntegral = pitchIntegral = yawIntegral = 0.0;
        rollMotor.moveTo(0);
        pitchMotor.moveTo(0);
        yawMotor.moveTo(0);
        rollMotor.run();
        pitchMotor.run();
        yawMotor.run();
        return;
    }

    // --- PID (미분항은 자이로 각속도 기반) ---
    float rollOutput  = computePID(rollSetpoint,  rollAngle,  gyroRoll,
                                   Kp_roll,  Ki_roll,  Kd_roll,  rollIntegral);
    float pitchOutput = computePID(pitchSetpoint, pitchAngle, gyroPitch,
                                   Kp_pitch, Ki_pitch, Kd_pitch, pitchIntegral);
    float yawOutput   = computePID(yawSetpoint,   yawAngle,   gyroYaw,
                                   Kp_yaw,   Ki_yaw,   Kd_yaw,   yawIntegral);

    // --- 모터 명령 ---
    driveMotor(rollMotor,  rollOutput,  INVERT_ROLL);
    driveMotor(pitchMotor, pitchOutput, INVERT_PITCH);
    driveMotor(yawMotor,   yawOutput,   INVERT_YAW);

    rollMotor.run();
    pitchMotor.run();
    yawMotor.run();
}
