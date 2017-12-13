#include <Stepper.h>

//Start of MPU6050 Accelerometer setup code
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
//End of MPU6050 Accelerometer setup code

//Setup stepper motors
const int stepsPerRevolution = 200;
Stepper myStepperFish(stepsPerRevolution, 6, 7, 8, 9);
Stepper myStepperShark(stepsPerRevolution, 10, 11, 12, 13);

//setup variables for the game, these are better explained in context
float spd;
float gap;
float target_distance = 1000;
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

float getfishspeed() { //reads accelerometer and gets fish speed. Taken from example MPU6050 code, but is modified.
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
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q); //Takes world acceleration, adjusted to ignore acceleration due to gravity
    spd = abs(0.7*aaWorld.y+0.2*aaWorld.z+0.1*aaWorld.x-500); //The modified part that takes a linear combination of the three
    //accelerometer outputs, prioritising movement in the y direction. -500 calibrates the speed to 0 for no movement.
}
 return spd; //return the speed
} 

int movefishposition(float fishdistance, float sharkdistance, int stepperstepsFish) //function that moves the fish position
{
    gap = fishdistance-sharkdistance;
    int gaptoposition = -floor((200/(1+exp(-0.05*(0.25*gap-70))))-20); //fish position is based on the distance between fish and shark
    //if gap == 30: position = 0. if gap >350: position = 0180.if gap < 5: position = 20
    //This is done using a sigmoid curve so that the fish asymptotically reaches a position between of 20 to -180.
    int stepstoposition = (gaptoposition-stepperstepsFish); //calculates the steps to get to the position, factoring in current position
    myStepperFish.step(stepstoposition);
    stepperstepsFish = gaptoposition; //update the current position of the fish
    return stepperstepsFish;
}

int moveshark(float sharkspeed, int stepperstepsShark)
{   
    int SSF = floor(sharkspeed); //needs to be integer
    myStepperShark.setSpeed(SSF); //shark stepper speed scales with theoretical shark speed
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

