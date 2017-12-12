
//#include "Metro.h"
#include <Servo.h>
Servo myservo;
int pos = 0;
#include <Stepper.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

//////
//Metro stepperMetro = Metro(25000); //subject to change
const int stepsPerRevolution = 200;
Stepper myStepperFish(stepsPerRevolution, 6, 7, 8, 9);
Stepper myStepperShark(stepsPerRevolution, 10, 11, 12, 13);
//////

float spd;
float gap;
//reset_button = False
float target_distance = 1000; //CAN CHANGE
//float sharkacceleration = 5*(dt); //after 10 seconds, goes from 30 to 80. CAN CHANGE
float fishspeed = 0;
float sharkspeed = 10; //CAN CHANGE
float fishdistance = 0;
float sharkstartingdistance = -100; //CAN CHANGE
float sharkdistance = sharkstartingdistance; //CAN CHANGE
int stepperstepsShark = 0;
int stepperstepsFish = 0;
int steps = -2;
float time1 = 0;
float time2 = 0;
float dt = 0;
//int minimumFishSteps = -24;
//int maximumFishSteps = 180;

float getfishspeed() { //reads accelerometer and gets fish speed
if (!dmpReady) return;
while (!mpuInterrupt && fifoCount < packetSize) {
}
mpuInterrupt = false;
mpuIntStatus = mpu.getIntStatus();
fifoCount = mpu.getFIFOCount();
if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
} else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    spd = abs(0.7*aaWorld.y+0.2*aaWorld.z+0.1*aaWorld.x-500);
    //Serial.print(spd);
    //Serial.print("\n");
}
 return spd;
} 

int movefishposition(float fishdistance, float sharkdistance, int stepperstepsFish)
{
    gap = fishdistance-sharkdistance;
    int gaptoposition = -floor((200/(1+exp(-0.05*(0.25*gap-70))))-20); 
    //Serial.println(gaptoposition);
    //if gap == 30: position = 0. if gap >350: position = 180.if gap < 5: position = -24+4
    int stepstoposition = (gaptoposition-stepperstepsFish);
    //Serial.println(stepstoposition);
    myStepperFish.step(stepstoposition);
    //If the distance is great, make the fish position near 100 (but approaches asymptotically), and vice-versa.
    //stepperstepsFish += stepstoposition; //stepswalked
    stepperstepsFish = gaptoposition; //stepswalked
    //Serial.println(stepperstepsFish);
    return stepperstepsFish;
}

int moveshark(float sharkspeed, int stepperstepsShark)
{   
    int SSF = floor(sharkspeed); //needs to be integer
    myStepperShark.setSpeed(SSF);
    if (stepperstepsShark > 70)
    {
      steps = 2;
    }
    else if (stepperstepsShark < 0)
    {
      steps = -2;
    }
    myStepperShark.step(steps); //2 steps worth of movement    
    stepperstepsShark += steps; //steps walked
    
    return stepperstepsShark;
}

void sharkeat(float sharkspeed, int stepperstepsShark)
{
    int SSF = floor(sharkspeed);
    myStepperShark.setSpeed(SSF);
    int stepstoorigin = (-(200+stepperstepsShark%200));
    //myStepperShark.step(stepstoorigin); //if shark is moving
    myStepperFish.step(-(stepperstepsFish-375)); //move fish to back, 400 is from 0 position to back
    //myStepperFish.step(700); //move all the way back
    myStepperShark.step(-100);
    Serial.println("tasty");
    delay(5000);
    myStepperShark.step(100);
    myStepperFish.step(-375);
    exit(0);
}

void setup() {
    myservo.attach(5);  
    myStepperFish.setSpeed(60);
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);
    while (!Serial);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


}



void loop() {
    Serial.println("--------loop working------");
    
 
    dt = (time2 - time1)/1000;
    Serial.println(dt);

        
    time1 = millis();
    //Serial.println(time1);
    ///*
    spd = getfishspeed();
    //Serial.println(spd);
    //Serial.print("\n");
    if (spd<12000)
    {
      fishspeed = (spd/45);
    }
    else
    {
      fishspeed = 12000/120;
    }
 
  
    sharkspeed += 5*dt;
    fishdistance += fishspeed*dt;
    sharkdistance += sharkspeed*dt;
    
    Serial.print("fishspeed: "); Serial.print(fishspeed);
    Serial.println("\n");
    Serial.print("fishdistance: "); Serial.print(fishdistance);
    Serial.println("\n");
    Serial.print("sharkspeed: "); Serial.print(sharkspeed);
    Serial.println("\n");
    Serial.print("sharkdistance: "); Serial.print(sharkdistance);
    Serial.println("\n");


    if (fishdistance >= target_distance) //fish made it!
    {
        Serial.println("YOU SURVIVED");
        delay(5000);
        myStepperFish.step(-stepperstepsFish);
        exit(0);
        
    }
    
    else if (sharkdistance >= (fishdistance)) //shark reached fish
    {
        sharkeat(sharkspeed, stepperstepsShark);
        //Serial.println("DED");
        //delay(500);
        //exit(0);
    }
                    
    
    stepperstepsFish = movefishposition(fishdistance, sharkdistance, stepperstepsFish);
    //stepperstepsShark = moveshark(sharkspeed, stepperstepsShark);
    /*
    if (pos = 0; pos <= 90; pos += 1;) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              
      //delay(2);                       
    }
    if (pos = 90; pos >= 0; pos -= 1;) { 
      myservo.write(pos);              
      //delay(2);                       
    }
    */
    time2 = millis(); 
    //Serial.println(time2);

}

