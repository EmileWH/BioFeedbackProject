#include <Wire.h>

#include <MPU6050_6Axis_MotionApps20.h>

#include <LiquidCrystal.h>


//loads in lcd pins based on breaboard connection
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

MPU6050 mpu;



bool dmpReady = false;

uint8_t mpuIntStatus;

uint16_t packetSize;

uint16_t fifoCount;

uint8_t fifoBuffer[64];



Quaternion q;

VectorFloat gravity;

float ypr[3];



float lastPitch = 0;

unsigned long lastTime = 0;



// ----- Rep Logic -----

bool liftingUp = false;

float repEffort = 0;

float lastRepEffort = 0;

int repCount = 0;

float upwardStartTime = 0;



// smoothing

float effortSmoothed = 0.0f;

const float SMOOTH_ALPHA = 0.2f;



unsigned long lastLcdUpdate = 0;

const unsigned long LCD_UPDATE_MS = 150;



float mapToPercent(float x, float inMin, float inMax) {

  if (x < inMin) x = inMin;

  if (x > inMax) x = inMax;

  return (x - inMin) * 100.0f / (inMax - inMin);

}



void setup() {

  Serial.begin(115200);



  lcd.begin(16, 2);

  lcd.print("Initializing");



  Wire.begin();

  Wire.setClock(400000);

  delay(100);



  mpu.initialize();



  if (!mpu.testConnection()) {

    lcd.clear();

    lcd.print("MPU FAIL");

    while (1);

  }



  uint8_t devStatus = mpu.dmpInitialize();


//initializing 
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

    lcd.clear();

    lcd.print("Ready");

    delay(600);

    lcd.clear();

  } else {

    lcd.clear();

    lcd.print("DMP FAIL");

    while (1);

  }



  lastTime = micros();

}



void loop() {

  if (!dmpReady) return;



  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();



  if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {

    mpu.resetFIFO();

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



  // Roll in degrees, pitch is whats used for the direction of the curl

  float pitch = ypr[1] * 180.0 / M_PI;



  unsigned long now = micros();

  float dt = (now - lastTime) / 1000000.0f; //dt calculation for speed

  lastTime = now;



  // Angular velocity (deg/sec)

  float pitchSpeed = (pitch - lastPitch) / dt; //calculates speed from accel

  lastPitch = pitch;



  

  float upwardSpeed = -pitchSpeed; 



  const float START_SPEED = 10.0f;  // deg/sec

  const float END_SPEED = 0.25f;





unsigned long nowMs = millis();

if (!liftingUp) {

  if (upwardSpeed > START_SPEED) {

    if (upwardStartTime == 0) {

      upwardStartTime = nowMs;

    }

    if (nowMs - upwardStartTime >= 300) {

      liftingUp = true;

      repEffort = 0;

      upwardStartTime = 0;

    }

  } else {

    upwardStartTime = 0;

  }

}





  if (liftingUp) {



    float speedMag = upwardSpeed;

    if (speedMag < 2.0f) speedMag = 2.0f;



    // Slower rotation = higher power, effort is inversly related to speed

    float effortSample = 100 / (speedMag);



    effortSmoothed =

      SMOOTH_ALPHA * effortSample +

      (1.0f - SMOOTH_ALPHA) * effortSmoothed;



    repEffort += effortSmoothed * dt;



    if (upwardSpeed < END_SPEED) {

      liftingUp = false;

      lastRepEffort = repEffort;

      repCount++;



      Serial.print("Rep ");

      Serial.print(repCount);

      Serial.print(" Effort: ");

      Serial.println(lastRepEffort, 2);

    }

  }



  float powerPct = mapToPercent(effortSmoothed, 0.0f, 1.0f);



  unsigned long msNow = millis();

  if (msNow - lastLcdUpdate >= LCD_UPDATE_MS) {

    lastLcdUpdate = msNow;



    lcd.setCursor(0, 0);

    lcd.print("Reps:");

    lcd.print(repCount);

   



    lcd.setCursor(0, 1);

    lcd.print("Effort: ");

    lcd.print(lastRepEffort, 1);

   

  }

  delay(5);

}

        
