extern MPU6050 mpu;
extern int16_t gx, gy, gz;
extern float yaw, currentYaw;
extern unsigned long lastTime;

#define Motor_Left 1
#define Motor_Right 2

float gyroKp = 3.0;
float gyroKi = 0.03;
float gyroKd = 0.45;
float gyroMaxCorrection = 30;

float gyroIntegral = 0;
float gyroPrevError = 0;
unsigned long gyroPrevPidTime = 0;

void mManual(int lspeed = 100, int rspeed = 100, int timeout = 0, bool doBeep = false) {
  motor(Motor_Left, lspeed);
  motor(Motor_Right, rspeed);
  if (timeout > 0) {
    delay(timeout);
    ao();
  }
  if (doBeep) beep();
}

void FF(int speed = 100, int timeout = 250) {
  mManual(speed, speed, timeout);
}
void FL(int speed = 100, int timeout = 250) {
  mManual(speed, -speed, timeout);
}
void FR(int speed = 100, int timeout = 250) {
  mManual(-speed, speed, timeout);
}

int clampMotor(int value) {
  if (value > 100) return 100;
  if (value < -100) return -100;
  return value;
}

float clampFloat(float value, float minV, float maxV) {
  if (value > maxV) return maxV;
  if (value < minV) return minV;
  return value;
}

void updateMPU() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  mpu.getRotation(&gx, &gy, &gz);

  float gz_dps = gz / 131.0;
  yaw += gz_dps * dt;
  currentYaw = yaw;
}

void resetYaw() {
  yaw = 0;
  currentYaw = 0;
  lastTime = micros();
}

void calibrateGyro() {
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  mpu.setZGyroOffset(-sum / 1000);
}

void resetGyroPID() {
  gyroIntegral = 0;
  gyroPrevError = 0;
  gyroPrevPidTime = micros();
}

void gyroFF(int timeMs, int speed = 55) {
  resetGyroPID();
  updateMPU();

  float targetYaw = currentYaw;
  unsigned long start = millis();
  while (millis() - start < timeMs) {
    updateMPU();

    unsigned long now = micros();
    float dt = (now - gyroPrevPidTime) / 1000000.0;
    gyroPrevPidTime = now;

    if (dt <= 0 || dt > 0.1) dt = 0.01;

    // error > 0 = เบียงซ้าย
    // error < 0 = เบียงขวา
    float error = currentYaw - targetYaw;
    if (error > -0.8 && error < 0.8) {
      error = 0;
    }
    gyroIntegral += error * dt;
    gyroIntegral = clampFloat(gyroIntegral, -30, 30);

    float derivative = (error - gyroPrevError) / dt;
    gyroPrevError = error;

    float correction = (gyroKp * error) + (gyroKi * gyroIntegral) + (gyroKd * derivative);
    correction = clampFloat(correction, -gyroMaxCorrection, gyroMaxCorrection);

    int motorTrim = 0;

    int leftSpeed = speed;
    int rightSpeed = speed + motorTrim;

    if (correction > 0) {
      rightSpeed = speed - (int)correction;
    } else if (correction < 0) {
      leftSpeed = speed - (int)(-correction);
    }

    leftSpeed = clampMotor(leftSpeed);
    rightSpeed = clampMotor(rightSpeed);
    mManual(leftSpeed, rightSpeed);
  }

  ao();
  delay(60);
}

void gyroTurn(float angleDelta) {
  resetGyroPID();
  updateMPU();
  float targetYaw = currentYaw + angleDelta;

  float kp = 4.4;
  float ki = 0.01;
  float kd = 0.15;
  float minSpeed = 55;  // ความเร็วขั้นต่ำ กันหยุดกลางทาง
  float maxSpeed = 70;  // ความเร็วสูงสุดตอนเลี้ยว

  unsigned long start = millis();
  while (millis() - start < 3000) {
    updateMPU();

    unsigned long now = micros();
    float dt = (now - gyroPrevPidTime) / 1000000.0;
    gyroPrevPidTime = now;
    if (dt <= 0 || dt > 0.1) dt = 0.01;

    float error = targetYaw - currentYaw;

    if (abs(error) < 1.5) break;  // deadband หยุดเมื่อพอ

    gyroIntegral += error * dt;
    gyroIntegral = clampFloat(gyroIntegral, -20, 20);

    float derivative = (error - gyroPrevError) / dt;
    gyroPrevError = error;

    float pidOut = (kp * error) + (ki * gyroIntegral) + (kd * derivative);
    pidOut = clampFloat(pidOut, -maxSpeed, maxSpeed);

    // ใส่ minimum speed ป้องกันมอเตอร์หยุดกลางทาง
    int turnSpeed;
    if (pidOut > 0)
      turnSpeed = (int)max((float)minSpeed, pidOut);
    else
      turnSpeed = (int)min((float)-minSpeed, pidOut);

    mManual(-turnSpeed, turnSpeed);
  }

  ao();
  delay(80);
}

void gyroFL(float angle) {
  gyroTurn(+angle);
  resetYaw();
}
void gyroFR(float angle) {
  gyroTurn(-angle);
  resetYaw();
}