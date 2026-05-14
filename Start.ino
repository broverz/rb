void gyroFF(int timeMs, int speed = 55);
void gyroFL(float angle);
void gyroFR(float angle);

void Start() {
  resetYaw();
  sleep(500);

  // gyroFF(500,100);
  // sleep(500);
  // gyroFR(77.8);
  // sleep(500);
  // gyroFF(500,100);
  // sleep(1000);
  // gyroFL(32.8);
  // sleep(1000);
  gyroFR(55.8);
}