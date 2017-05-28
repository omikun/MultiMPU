
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

//using CD4515BE (select LOW, INHIBIT=1 means select=1)
#define INHIBIT "A5" //SCL=1, select=1" 
#define STROBE 8
#define DATA0 4 //datasheet: data 1
#define DATA1 5 //datasheet: data 2
#define DATA2 3 //datasheet: data 3
#define DATA3 2 //datasheet: data 4

#define MPU true
#define MICRO true //get time in micros instead of millis
const int numSensors = 2;

MPU6050 mpu(0x68);
MPU6050 mpu2(0x69); // 0x69 address, must set AD0 to high to use
uint16_t packetSize;
uint16_t fifoCount;
Quaternion q[numSensors];
VectorFloat gravity;
long lastTime;
float ypr[numSensors][3*numSensors];

int8_t numEnabledMPUs = 0; //for use when printing (not enabled MPU's not sampled)
uint16_t count[numSensors]; //size of FIFOqueue per sensor
#if MICRO
  const long sampleInterval = 30000;//45000;
#else
  const long sampleInterval = 40;
#endif

long time()
{
#if MICRO
  return micros();
#else
  return millis();
#endif
}
bool EnabledMPU[numSensors];

void EnableSensor(int i)
{
  digitalWrite(STROBE, HIGH);
  digitalWrite(DATA0, i & 0x1);
  digitalWrite(DATA1, i & 0x2);
  digitalWrite(DATA2, i & 0x4);
  digitalWrite(DATA3, i & 0x8);
  digitalWrite(STROBE, LOW);
}

void setup() {

    Wire.begin();
    TWBR = 12;

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    pinMode(STROBE, OUTPUT);
    pinMode(DATA0, OUTPUT);
    pinMode(DATA1, OUTPUT);
    pinMode(DATA2, OUTPUT);
    pinMode(DATA3, OUTPUT);
    digitalWrite(STROBE, LOW);

    for (int i=0; i<numSensors; i+=2)
    {
      EnableSensor(i/2);
#if MPU
      EnabledMPU[i] = InitMPU(i, &mpu);
      if (i+1 < numSensors)
      {
        EnabledMPU[i+1] = InitMPU(i+1, &mpu2);
      }
      numEnabledMPUs += EnabledMPU[i] + EnabledMPU[i+1];
#endif
    }
    
    //packetSize = mpu.dmpGetFIFOPacketSize();
    //fifoCount = mpu.getFIFOCount();

    /*
// wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
#if MPU
    for (int i=0; i<numSensors; i+=2)
    {
      EnableSensor(i/2);
      mpu.resetFIFO();
      mpu2.resetFIFO();
    }
#endif
    String msgSetupComplete = "Setup complete: " + String(numEnabledMPUs);
    Serial.println(msgSetupComplete);
    lastTime = time();
}

bool InitMPU(int i, MPU6050 * mpu)
{
  // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu->initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    String connectStatus = "MPU6050 " + String(i) + " connection ";
    connectStatus += mpu->testConnection() ? "successful" : "failed";
    Serial.println( connectStatus);
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    uint8_t devStatus = mpu->dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu->setXGyroOffset(220);
    mpu->setYGyroOffset(76);
    mpu->setZGyroOffset(-85);
    mpu->setZAccelOffset(1788); // 1688 factory default for my test chip
// supply your own gyro offsets here, scaled for min sensitivity
    return CheckMPUStatus(devStatus, mpu);
}

bool CheckMPUStatus(uint8_t devStatus, MPU6050 * mpu)
{
  // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu->setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        packetSize = mpu->dmpGetFIFOPacketSize();
        return true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        return false;
    }
}

void ReadMPU(bool checkFIFO, int i, MPU6050 * mpu)
{
    #if MPU
    uint8_t fifoBuffer[64];
    if (checkFIFO)
    {
        fifoCount = mpu->getFIFOCount();
        count[i] = fifoCount;

        if (false && fifoCount < packetSize) {
            return;
        }

        if (true || fifoCount >= packetSize) {
            mpu->getFIFOBytes(fifoBuffer,packetSize);
            fifoCount -= packetSize;
        }
        mpu->dmpGetQuaternion(&q[i],fifoBuffer);
        mpu->dmpGetGravity(&gravity,&q[i]);
        mpu->dmpGetYawPitchRoll(ypr[i],&q[i],&gravity);          

    } else {
        mpu->getFIFOBytes(fifoBuffer,14);//packetSize);
        mpu->dmpGetQuaternion(&q[i],fifoBuffer);
    }

    mpu->resetFIFO();
#endif

}

int it = 0;
long pt = 0;
long iteration = 0;
void loop() {
    long timeNow;
    long dt = 0;
    int steps = 0;
    //wait for #ms: 30ms ~8000 loop iteration
    while(dt < sampleInterval)
    {
        timeNow = time();
        dt = timeNow - lastTime;
        steps++;
    }
    
    lastTime = timeNow;

    bool checkFIFO = false;
    for (int i=0; i<numSensors; i+=2)
    {
      EnableSensor(i/2);
      if (EnabledMPU[i])
          ReadMPU(checkFIFO, i, &mpu);
      if (i+1 < numSensors)
      {
        if (EnabledMPU[i+1])
          ReadMPU(checkFIFO, i, &mpu2);
      }
      //delay(500);
    }
    if (!checkFIFO)
    {
      printBinary(steps, count);
      //printData(steps, count);
    } else {
      //test when fifo out of alignment
      bool printFlag = false;
      for (int i=0; i<numSensors; i++)
      {
        printFlag |= (count[i] & 0x63 != 42) || count[i] == 0;
      }
      
      if (printFlag)
        printData(steps, count);
      if (false)
      {
        Serial.print(iteration);
        Serial.print(": ");
        for (int i=0; i<numSensors; i++)
        {
          Serial.print(count[i]);
          Serial.print(", ");
        }
        Serial.println();
      }
    }
    iteration++;
}

void printBinary(long steps, int *count) {
  for (int i=0; i<numEnabledMPUs; i++)
    {
      if (true)
      {
        Serial.print(q[i].w);
        Serial.print(",");
        Serial.print(q[i].x);
        Serial.print(",");
        Serial.print(q[i].y);
        Serial.print(",");
        Serial.print(q[i].z);
        Serial.print(",");
      } else {
        Serial.write((byte*)&(q[i].w),4);
        Serial.write((byte*)&(q[i].x),4);
        Serial.write((byte*)&(q[i].y),4);
        Serial.write((byte*)&(q[i].z),4);
        }
    }
    Serial.println();
}
void printData(long steps, int *count) {
    
    long timeNow = time();
    long st = timeNow - lastTime;
    long lastTime2 = timeNow;
    
    Serial.print(timeNow);
    Serial.print(" steps: ");
    Serial.print(steps);
    
    Serial.print(" st: ");
    Serial.print(st);
    
    Serial.print(" lpt: ");
    Serial.print(pt);
    ///*
    for (int i=0; i<numSensors; i++)
    {
      Serial.print(" fifoCount: ");
      Serial.print(count[i]);
#if true
      Serial.print(" q ");
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
    timeNow = time();
    pt = timeNow - lastTime2;
}
