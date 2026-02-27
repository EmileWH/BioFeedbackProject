#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

float lastRoll = 0;
unsigned long lastTime = 0;

bool inRep = false;
float repEffort = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  delay(100);

  Serial.println("MPU Initializing");

  mpu.reset();
  delay(100);

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("No Connection");
    while (1);
  }

  uint8_t devStatus = mpu.dmpInitialize();

  // calibration offsets
  mpu.setXAccelOffset(-1600);
  mpu.setYAccelOffset(-1800);
  mpu.setZAccelOffset(1200);
  mpu.setXGyroOffset(25);
  mpu.setYGyroOffset(-7);
  mpu.setZGyroOffset(8);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("DMP ready!");
  } else {
    Serial.print("DMP init failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  lastTime = micros();
}

void loop() {
  if (!dmpReady) return;

  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
    return;
  }

  if (fifoCount < packetSize) return;

  while (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float roll = ypr[2] * 180 / M_PI; 

 // speed calculatioin
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0
  lastTime = now;

  // roll speed calculation based on accel data
  float rollSpeed = (roll - lastRoll) / dt;
  lastRoll = roll;

  // effort is 1/speed
  float speed = abs(rollSpeed);
  if (speed < 1.0) speed = 1.0; 
  float effortSample = 100.0 / speed;

  // rep detection based on roll speed
  float threshold = 20.0;
  if (!inRep && speed > threshold) {
    inRep = true;
    repEffort = 0;
  }
  if (inRep) {
    repEffort += effortSample * dt;
    if (speed < threshold) {
      inRep = false;
      Serial.print("Rep completed! Total effort: ");
      Serial.println(repEffort, 2);
    }
  }

  // debug output
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" deg, Roll speed: "); Serial.print(rollSpeed);
  Serial.print(" deg/s, Effort: "); Serial.println(effortSample, 2);

  delay(5);
}




        
