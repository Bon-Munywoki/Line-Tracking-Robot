

// AUTHORED BY JKD



#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include <Servo.h>

#include <Ramp.h>


#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

HUSKYLENS huskylens;
//SoftwareSerial mySerial(52, 50); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11


// Motor pins
#define RFMpin1  38     // Right Front Motor pin1
#define RFMpin2  48     // Right Front Motor pin2

#define LFMpin1  40     // Left Front Motor pin1
#define LFMpin2  42     // Left Front Motor pin2

#define RRMpin1  41     // Right Rear Motor pin1
#define RRMpin2  39     // Right Rear Motor pin2

#define LRMpin1  43     // Left Rear Motor pin1
#define LRMpin2  47     // Left Rear Motor pin2

#define RFM_EN   11     // Right Front Motor Enable Pin
#define LFM_EN   12    // Left Front Motor Enable Pin


#define RRM_EN   10    // Right Rear Motor Enable Pin
#define LRM_EN   9    // Left Rear Motor Enable Pin

int motor_speed = 120;

int DRMP = 0;
int DLMP = 0;
int RLRMP = 0;
int RLLMP = 0;
int FLRMP = 0;
int FLLMP = 0;

int us_distance;
int left_us_distance;
int right_us_distance = 400;
int leftFront_us_distance;
int rightFront_us_distance;

MPU6050 mpu;

bool dmpReady = false;                                      // set true if DMP init was successful
uint8_t mpuIntStatus;                                       // holds actual interrupt status byte from MPU
uint8_t devStatus;                                          // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                                        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                                         // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                                     // FIFO storage buffer

// orientation/motion vars
Quaternion q;                                                // [w, x, y, z]         quaternion container
VectorInt16 aa;                                              // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;                                          // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;                                         // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;                                         // [x, y, z]            gravity vector
float euler[3];                                              // [psi, theta, phi]    Euler angle container
float ypr[3];                                                // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;                          // indicates whether MPU interrupt pin has gone high
bool MPU6050Ready = true;


int IMU_INTVALCHECK_SEC = 500;
int IMUcm;
int IMUpm;

float yawOffset = 20;
float YAWVAL = 0.0;
float yawVal = 0.0;
float yawval_correction = 0;
int yawCalibPeriod = 0;
int global_fifo_count = 0;

const float default_Heading = 200;

bool init_serial = false;

#define red_color             1
#define green_color           2
#define blue_color            3


int avgRear;
int avg;
int Task =      0;
int subTask =   0;
int Robo_Status;
int color =     0;
int OBJcolor =  0;

int lineCount = 0;
int rear_lineCount = 0;
int rear_lineCountOne = 0;

int gap = 0;

void setup() {

  Serial.begin(115200);
  sensorSETUP();
  motorPinSETUP();
  servoSETUP();
  //MPU6050_SETUP();
  //setupDelay(20000);
  //led_colorSet(5);
  //heading_update();

  //EnableRobot();
  setupDelay(1500);
  Task =      3;
}

void loop() {


  //rearLine_intersectionDetectionOne();
  //Serial.println(right_us_distance);
  //left_us_Distance();
  //us_Distance();
  //leftFront_us_Distance();
  //right_us_Distance();
  //Serial.println(right_us_distance);
  //delay(100);

  //maintain_rightWalldistance(20,0);
  //  MPU6050_YAW();
  //  heading_update();
  //find_heading(90, 0);
  //DELAY_an_action(450, RobotForward);
  //maintain_rightWalldistance(75, 0);

  //pick_objectTrailer(0);
  //Object_pickUp();
  //pick_object(0);
  //placeItemAtMidLevel();

  //arm_parking(30);
  //arm_parking(80);
  //setAngle(50, 95, 140, 150, 80, 1500);
  //setAngle(50, 95, 140, 150, 40, 1500);

  //track_whiteline();
  //track_RearWhiteline();
  //frontWall_alignment(20, 0);
  //serialPrintData();
  //delay(250);
  //positionRobot_lineLocus(4500, 4500, 0);
  //pick_lowerWhiteWheel(0);
  //pick_firstDeckWheel();
  //F4W_independent_MecDrive(-100, 80, 100, -80);
  //maintain_SideWalldistance(30);

  trailer_pickUp();
  place_trailer();
  pick_wheels();
  pick_engine();
  place_engine();
  //place_wheels();
  //pick_cabin();
  //place_cabin();
  //home_RUN();


}

void trailer_pickUp()
{
  while (Task == 0)
  {
    left_us_Distance();
    int prevDist = left_us_distance;
    int currentDist = left_us_distance;
    int setDist = left_us_distance;
    switch (subTask)
    {
      case 0: // Leave start Zone
        while (gap < 5 && subTask == 0) {
          left_us_Distance();
          currentDist = left_us_distance;
          //maintain_heading(1, 180);
          follow_leftSideWall(setDist);
          gap = abs(currentDist - prevDist);
          prevDist = currentDist;

          //heading_update();
          if (gap > 5) {
            DELAY_an_action(150, RobotForward, 80);
            RobotStop();
            delay(50);
            //maintain_distance(60, 1);
            subTask = 1;
            break;
          }
        }
        break;

      case 1: // Turn towards Trailer
        //heading_update();
        //find_heading(90, 2);
        DELAY_an_action(850, RobotRotate_CClockwise, 70);
        DELAY_an_action(500, RobotStop, 150);
        subTask = 2;
        //CLEAR_lineCounter();
        break;

      case 2:
        frontWall_alignment(17, 3);
        break;

      case 3:
        pick_objectTrailer(4);
        CLEAR_lineCounter();
        break;

      case 4:
        frontWall_alignment(17, 5);
        //CLEAR_lineCounter();
        break;

      case 5:
        //rear_lineCount = 0; // Drive towards chasis to place Trailer
        //DELAY_an_action(3500, RobotReverse, 90);
        while (rear_lineCountOne < 1 ) { //rear_lineCountOne
          //heading_update();
          rearLine_intersectionDetectionOne();
          RobotReverse(80);
          //maintain_heading(-1, 90);
          if (rear_lineCountOne == 1) {
            //RobotStop();
            DELAY_an_action(500, RobotReverse, 80);
            //RobotStop();
            //delay(100);
            subTask = 6;
            //CLEAR_lineCounter();
            //break;
          }
        }
        break;

      case 6:
        CLEAR_lineCounter();
        subTask = 7;
        break;

      case 7:
        //        DELAY_an_action(1200, RobotReverse, 80);
        //        RobotStop();
        //CLEAR_lineCounter();
        //F4W_independent_MecDrive(-100, 80, 100, -80);
        //delay(1300);
        while (rear_lineCountOne < 1 ) {
          rearLine_intersectionDetectionOne();
          F4W_independent_MecDrive(-100, 80, 100, -80);
          //headingControlled_strafing(1, 90);
          if (rear_lineCountOne == 1) {
            F4W_independent_MecDrive(-100, 80, 100, -80);
            delay(150);
            RobotStop();
            delay(50);
            DELAY_an_action(1740, RobotRotate_CClockwise, 70);
            DELAY_an_action(150, RobotStop, 150);
            subTask = 8;
            //CLEAR_subTask();
            //Task = 1;
          }
        }
        CLEAR_lineCounter();
        break;

      case 8:
        DELAY_an_action(6000, track_whiteline, 80);
        RobotStop();
        delay(50);
        DELAY_an_action(1740, RobotRotate_CClockwise, 70);
        DELAY_an_action(50, RobotStop, 150);
        while (rear_lineCountOne < 1 ) {
          rearLine_intersectionDetectionOne();
          F4W_independent_MecDrive(-100, 80, 100, -80);
          //headingControlled_strafing(1, 90);
          if (rear_lineCountOne == 1) {
            F4W_independent_MecDrive(-100, 80, 100, -80);
            delay(150);
            RobotStop();
            delay(100);

            CLEAR_subTask();
            Task = 1;
          }
        }
    }
  }
}


void place_trailer()
{

  while (Task == 1)
  {
    switch (subTask)
    {
      case 0:
        us_Distance();
        while (us_distance > 15)
        {
          us_Distance();
          track_whiteline();

          if (us_distance <= 15) {
            RobotStop();
            delay(50);
            subTask = 1;
            break;
          }
        }

        break;

      case 1:
        maintain_distance(10, 2);
        break;

      case 2:
        place_objectTrailer(3);
        break;

      case 3:
        //maintain_distance(20, 4);
        DELAY_an_action(500, RobotReverse, 80);
        CLEAR_subTask();
        Task = 2;
        break;

    }
  }
}


void pick_wheels()
{
  while (Task == 2)
  {
    switch (subTask)
    {
      case 0:
        //recalibrate_yaw();
        DELAY_an_action(1740, RobotRotate_CClockwise, 70);
        DELAY_an_action(50, RobotStop, 150);
        subTask = 1;
        break;

      case 1:
        us_Distance();
        while (us_distance > 25) {
          //RobotForward(110);
          track_whiteline();
          us_Distance();
          if (us_distance <= 25) {
            RobotStop();
            delay(250);
            //break;
            subTask = 2;
          }
        }
        break;

      case 2:
        maintain_distance(23, 3);
        //CLEAR_subTask();
        //Task = 3;
        break;

      case 3:
        pick_lowerWhiteWheel(4);
        CLEAR_subTask();
        CLEAR_lineCounter();
        right_us_Distance();
        Task = 3;
        break;
    }
  }
}


void pick_engine()
{
  //CLEAR_subTask();
  right_us_Distance();
  while (Task == 3) {
    //recalibrate_yaw();

    switch (subTask)
    {   
      case 0:
        while (right_us_distance > 65)
        {
          right_us_Distance();
          //headingControlled_strafing(0, 180);
          RobotRightSlide(100);
          //F4W_independent_MecDrive(-100, 80, 100, 80);
          if (right_us_distance <= 65)
          {
            RobotStop();
            subTask = 1;
          }
        }
        break;

      case 1:
        frontWall_alignment(40, 2);
        break;

      case 2:
        while (leftFront_us_distance > 25)
        {
          leftFront_us_Distance();
          track_whiteline();

          if (leftFront_us_distance <= 25) {
            RobotStop();
            delay(50);
            subTask = 3;
          }
        }

      case 3:
        //positionRobot_lineLocus(4500, 4500);
        //maintain_distance(10, 2);
        frontWall_alignment(25, 4);
        break;
      //Pick Engine Subroutine

      case 4:
        pickEngine(5);
        break;


      case 5:
        //find_heading(340, 3);
        DELAY_an_action(1740, RobotRotate_CClockwise, 70);
        
        subTask = 6;
        break;

      case 6:
        DELAY_an_action(4500, track_whiteline, 80);
        RobotStop();
        CLEAR_lineCounter();
        //recalibrate_yaw();
        while (rear_lineCountOne < 1 ) {
          rearLine_intersectionDetectionOne();
          track_whiteline();
          //headingControlled_strafing(1, 90);
          if (rear_lineCountOne == 1) {
            DELAY_an_action(1500, track_whiteline, 80);
            RobotStop();
            delay(100);

            CLEAR_subTask();
            Task = 4;
          }
        }
        //CLEAR_lineCounter();
        break;
    }
  }
}



void place_engine()
{
  if (Task == 4)
  {
    switch (subTask)
    {
      case 0:
        DELAY_an_action(1600, RobotRotateClockwise, 70);
        CLEAR_lineCounter();
        subTask = 1;
        break;

      case 1:
        while (rear_lineCountOne < 1 ) {
          rearLine_intersectionDetectionOne();
          //headingControlled_strafing(0, 180);
          F4W_independent_MecDrive(-100, 80, 100, -80);
          if (rear_lineCountOne == 1) {
            F4W_independent_MecDrive(-100, 80, 100, -80);
            delay(150);
            DELAY_an_action(250, RobotStop, 150);
            //heading_update();
            us_Distance(); 
            subTask = 2;
          }
        }
        break;

      case 2:
        while (us_distance > 15)
        {
          us_Distance();
          track_whiteline();
          if (us_distance <= 15)
          {
            RobotStop();
            delay(500);
            subTask = 3;
          }
        }
        break;

      case 3:
        maintain_distance(15, 4);
        CLEAR_lineCounter();
        break;

      case 4:
        place_objectTrailer(5);
        CLEAR_subTask();
        Task = 5;
        break;
    }
  }
}


void place_wheels()
{
  if (Task == 5)
  {
    switch (subTask) {
      case 0:
        while (rear_lineCountOne < 1)
        {
          rearLine_intersectionDetectionOne();
          //headingControlled_strafing(0, 90);
          RobotRightSlide(100);

          if (rear_lineCountOne == 1)
          {
            RobotStop;
            subTask = 1;
            recalibrate_yaw();
            CLEAR_lineCounter();
          }
        }
        break;

      case 1:
        while (rear_lineCount < 1)
        {
          rearLine_intersectionDetection();
          track_whiteline();
          if (rear_lineCount == 1)
          {
            RobotStop;
            //find_heading(90, 2);
            DELAY_an_action(570, RobotRotate_CClockwise, 70);
            DELAY_an_action(500, RobotStop, 150);

            //CLEAR_subTask();
            Task = 6;
            CLEAR_subTask();
            CLEAR_lineCounter();
          }
        }
        break;
        // Placement of wheels on the chasis
    }
  }
}



void pick_cabin()
{
  if (Task == 6)
  {
    switch (subTask)
    { // After recalibrating YAW while facing chasis
      case 0:
        DELAY_an_action(970, RobotRotateClockwise, 70);
        CLEAR_lineCounter();
        subTask = 1;
        break;

      case 1:
        while (rear_lineCount > 2)
        {
          track_whiteline();
          rearLine_intersectionDetection();
          if (rear_lineCount == 2)
          {
            RobotStop();
            //find_heading(90, 2);
            DELAY_an_action(570, RobotRotateClockwise, 70);
            DELAY_an_action(500, RobotStop, 150);
            CLEAR_lineCounter();
            subTask = 2;
          }
        }
        break;

      case 2:

        while (us_distance > 17)
        {
          us_Distance();
          track_whiteline();
          if (us_distance == 1)
          {
            RobotStop();
            maintain_distance(15, 3);

          }
        }
        break;

      case 3:
        //Pick cabin
        recalibrate_yaw();
        Task = 7;
        CLEAR_lineCounter();
        CLEAR_subTask();
        break;
    }
  }
}



void place_cabin()
{
  if (Task == 7)
  {
    switch (subTask)
    {
      case 0:
        find_heading(340, 1);
        break;

      case 1:
        while (rear_lineCount < 1)
        {
          rearLine_intersectionDetection();
          track_whiteline();
          if (rear_lineCount == 1)
          {
            RobotStop();
            find_heading(270, 2);
            CLEAR_lineCounter();
          }
        }
        break;

      case 2:
        while (rear_lineCount < 1)
        {
          track_whiteline();
          rearLine_intersectionDetection();
          if (rear_lineCount == 1)
          {
            RobotStop();
            maintain_distance(15, 4);
            Task = 8;
            CLEAR_subTask();
            CLEAR_lineCounter();
            recalibrate_yaw();
          }
        }
        break;
    }
  }
}



void home_RUN()
{
  if (Task == 9)
  {
    switch (subTask)
    {
      case 0:
        while (rear_lineCount < 1)
        {
          RobotReverse(120);
          rearLine_intersectionDetection();
          if (rear_lineCount == 1)
          {
            RobotStop();
            find_heading(270, 1);
            CLEAR_lineCounter();
          }
        }
        break;

      case 1:
        DELAY_an_action(850, RobotForward, 100);
        RobotStop();

        while (rear_lineCount < 3)
        {
          rearLine_intersectionDetection();
          headingControlled_strafing(1, 90);
          if (rear_lineCount == 3)
          {
            RobotStop();
            subTask = 2;
            CLEAR_lineCounter();
          }
        }
        break;

      case 2:
        while (rear_lineCount < 1)
        {
          track_whiteline();
          rearLine_intersectionDetection();
          if (rear_lineCount == 1)
          {
            RobotStop();
            find_heading(180, 3);
          }
        }
        break;

      case 3:
        DELAY_an_action(550, RobotForward, 100);
        find_heading(10, 4);
        RobotStop;
        Task = 10;
        break;
    }
  }
}



void serialPrintData()
{
  Serial.println(leftFront_us_distance);
  Serial.println(rightFront_us_distance);
  Serial.println(us_distance);
  Serial.println(left_us_distance);
}






void CLEAR_subTask() {
  subTask = 0;
}

void CLEAR_lineCounter() {
  rear_lineCount = lineCount = rear_lineCountOne = 0 ;
}


/*
          while (subTask==0)
        {
          readRearLines();
          readLines();
          positionRobot_lineLocus(3500, 3500);

          if (((avgRear < 3500 + 1000) && (avgRear > 3500 - 1000)) && (( avg < 3500 + 1000) && ( avg > 3500 - 1000))) {
            RobotStop();
            delay(50);
            subTask = 1;
            break;
          }
        }
*/
