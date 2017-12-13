#include <Stepper.h>

//Start of MPU6050 Accelerometer initial set up code
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
//End of MPU6050 Accelerometer initial set up code

//Set up stepper motors
const int stepsPerRevolution = 200;
Stepper myStepperFish(stepsPerRevolution, 6, 7, 8, 9);
Stepper myStepperShark(stepsPerRevolution, 10, 11, 12, 13);

/*

THE BASICS:
The shark chases the fish. They begin a distance away from each other. The shark has a speed that increases over time. 
The fish has a speed that is determined by the user's movement. They travel over a targer distance.
If for any reason the fish gets to the same position as the shark, the shark eats the fish and you lose.
If the fish gets to the maximum distance x = 1000, it escapes and you win.
The motors change their behaviour as functions of the speeds of the shark and fish.

Fish tail and wave movement are constant but it is possible for them to be worked into the code.

*/

//set up variables for the game
float spd; //raw speed value from accelerometer
float gap; //gap between fish and shark
float target_distance = 1000; //the finish line distance
float fishspeed = 0; //initialise fish speed, set to 0
float sharkspeed = 10; //initialise shark speed, CAN CHANGE 
float fishdistance = 0; //initialise fish distance, set to 0
float sharkstartingdistance = -100; //initialise shark distance, set to -100 to give the fish a headstart, CAN CHANGE
float sharkdistance = sharkstartingdistance; //CAN CHANGE
int stepperstepsShark = 0; //initialise shark current position
int stepperstepsFish = 0; //initialise fish current position
int steps = -2; //initialise steps per loop for shark movement
float time1 = 0; //initialise time marker variable 1
float time2 = 0; //initialise time marker variable 2
float dt = 0; //initialise time for loop to run

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
    spd = abs(0.7*aaWorld.y+0.2*aaWorld.z+0.1*aaWorld.x-500); //Takes a linear combination of the three
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

int moveshark(float sharkspeed, int stepperstepsShark) //function that moves the shark
{   
    int SSF = floor(sharkspeed); //needs to be integer
    myStepperShark.setSpeed(SSF); //shark stepper speed scales with theoretical shark speed
    if (stepperstepsShark > 70) //makes the stepper go back and forth instead of around due to physical limitations.
    {
      steps = 2;
    }
    else if (stepperstepsShark < 0)
    {
      steps = -2;
    }
    myStepperShark.step(steps); //only do a short movement each loop to keep perception that everything is running smoothly 
    stepperstepsShark += steps; //update current position of the shark
    
    return stepperstepsShark;
}

void sharkeat(float sharkspeed, int stepperstepsShark) //function to eat the fish if it gets too close
{
    int SSF = floor(sharkspeed);
    myStepperShark.setSpeed(SSF);
    //int stepstoorigin = (-(200+stepperstepsShark%200)); // if shark is moving, return shark to 0 position
    //myStepperShark.step(stepstoorigin); //if shark is moving
    myStepperFish.step(-(stepperstepsFish-375)); //move fish to back, 400 is from 0 position to back. Factors in current fish position
    myStepperShark.step(-100); //makes the shark eat
    Serial.println("tasty");
    delay(5000);
    myStepperShark.step(100); //reset position of shark
    myStepperFish.step(-375); //reset position of fish
    exit(0); //exits the code, an automatic restart can be programmed
}

void setup() 
    myStepperFish.setSpeed(60); //set fish stepper speed

    //Start of MPU6050 Accelerometer set up code
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
    //End of MPU6050 Accelerometer set up code

}



void loop() {
    Serial.println("--------loop working------"); //checks that the loop is running properly
    
 
    dt = (time2 - time1)/1000; //time taken for the loop to run once, calculated from the two time markers

    time1 = millis(); //time marker 1

    spd = getfishspeed(); //gets the speed from the accelerometer

    if (spd<12000)
    {
      fishspeed = (spd/45); //fishspeed is taken to be the reading from the accelerometer divided by scale factor for calibration
    }
    else
    {
      fishspeed = 12000/45; //ensures that the speed is capped
    }
 
  
    sharkspeed += 5*dt; //shark gets faster at 5 speed units per unit time. This makes the game harder over time
    fishdistance += fishspeed*dt; //fish distance increases. Distance = Speed * Time.
    sharkdistance += sharkspeed*dt; //shark distance increases
    
    //Serial command line information to see what is happening
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
        myStepperFish.step(-stepperstepsFish); //reset fish position
        exit(0);
        
    }
    
    else if (sharkdistance >= (fishdistance)) //shark reached fish
    {
        sharkeat(sharkspeed, stepperstepsShark);
    }
                    
    
    stepperstepsFish = movefishposition(fishdistance, sharkdistance, stepperstepsFish); //updates fish position
    //stepperstepsShark = moveshark(sharkspeed, stepperstepsShark); //updates shark speed and position.
    // ^ can be reenabled if shark stepper movement is smoothed to avoid stalling

    time2 = millis(); //time marker 2

}

