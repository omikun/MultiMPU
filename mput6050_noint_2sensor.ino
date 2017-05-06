
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

const int numSensors = 2;

MPU6050 mpu;
//MPU6050 mpu(0x69); // 0x69 address, must set AD0 to high to use
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q[numSensors];
VectorFloat gravity;
long lastTime;
float ypr[numSensors][3*numSensors];
#define SENSOR0 2  //AD0 for sensor 0

void setup() {

    Wire.begin();
    TWBR = 24;

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    for (int i=0; i<2; i++)
    {
      pinMode(SENSOR0+i, OUTPUT);
      digitalWrite(i, HIGH);
    }
    for (int i=0; i<numSensors; i++)
    {
      digitalWrite(i+SENSOR0, LOW);
      if (true)
      {
        InitMPU();
      } else {
        //TODO delay(1);//maybe need to delay??
        mpu.initialize();
        mpu.dmpInitialize();
        mpu.setXAccelOffset(-1343);
        mpu.setYAccelOffset(-1155);
        mpu.setZAccelOffset(1033);
        mpu.setXGyroOffset(19);
        mpu.setYGyroOffset(-27);
        mpu.setZGyroOffset(16);
        mpu.setDMPEnabled(true);
      }
      
      digitalWrite(i+SENSOR0, HIGH);
    }
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();

    lastTime = micros();
    /*
// wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
    for (int i=0; i<numSensors; i++)
    {
      digitalWrite(i+SENSOR0, LOW);
      mpu.resetFIFO();
      digitalWrite(i, HIGH);
    }
}

void InitMPU()
{
  // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    uint8_t devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    

}
int it = 0;
long pt = 0;
void loop() {
    long timeNow;
    long dt = 0;
    int steps = 0;
    //wait for #ms: 30ms ~8000 loop iteration
    while(dt < 40)
    {
        timeNow = micros();
        dt = timeNow - lastTime;
        steps++;
    }
    
    lastTime = timeNow;

    uint16_t count[numSensors];
    for (int i=0; i<numSensors; i++)
    {
      digitalWrite(i+SENSOR0, LOW);
      
      fifoCount = mpu.getFIFOCount();
      count[i] = fifoCount;
      
      if (fifoCount < packetSize) {
        return;//continue;
      }

      if (fifoCount >= packetSize) {
          mpu.getFIFOBytes(fifoBuffer,14);//packetSize);
          fifoCount -= packetSize;
      }    
      
  
      mpu.dmpGetQuaternion(&q[i],fifoBuffer);
#if false
      mpu.dmpGetGravity(&gravity,&q[i]);
      mpu.dmpGetYawPitchRoll(ypr[i],&q[i],&gravity);          
#endif
      mpu.resetFIFO();
      
      digitalWrite(i+SENSOR0, HIGH);
    }
    timeNow = micros();
    long st = timeNow - lastTime;
    long lastTime2 = timeNow;
    
    Serial.print(timeNow);
    Serial.print(" wait steps: ");
    Serial.print(steps);
    
    Serial.print(" sample time: ");
    Serial.print(st);
    
    Serial.print(" last print time: ");
    Serial.print(pt);
    ///*
    for (int i=0; i<numSensors; i++)
    {
      Serial.print("\t fifoCount: ");
      Serial.print(count[i]);
#if true
      Serial.print("\t q\t");
      Serial.print(q[i].w);
      Serial.print(" ");
      Serial.print(q[i].x);
      Serial.print(" ");
      Serial.print(q[i].y);
      Serial.print(" ");
      Serial.print(q[i].z);
#else
      Serial.print("\t ypr\t");
      Serial.print(ypr[i][0]*180/PI);
      Serial.print("\t");
      Serial.print(ypr[i][1]*180/PI);
      Serial.print("\t");
      Serial.print(ypr[i][2]*180/PI);
#endif
    }
    //*/
    Serial.println();
    timeNow = micros();
    pt = timeNow - lastTime2;
}
