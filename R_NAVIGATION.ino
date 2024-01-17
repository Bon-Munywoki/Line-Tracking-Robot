
// AUTHORED BY JKD

// VARIOUS APPS ON ROBOT'S NAVIGATION

/*
  unsigned long heading_cm;
  unsigned long heading_pm;
  const unsigned long headingPeriod = 100;
*/


void DELAY_an_action(int delayTime, void(*f)(int), int m_speed)//
{
  unsigned long delayMillis = 0;
  unsigned long delayPMillis = 0;
  for (int i = 0; i < 100; i++) {
    delayMillis = millis();
  }
  while (millis() < delayMillis + delayTime) {
    (*f)(m_speed);
  }
}






void maintain_distance(int distance_setpoint, int subtask) // int subtask

{

  us_Distance();
  float KP = 0.08;
  float gap = abs(distance_setpoint - us_distance);                //distance away from setpoint

  const int cruiseSpeed = 0;
  int maxSpeed = 60;

  if (gap < 20) {                                                 //we're close to setpoint, use conservative tuning parameters
    KP = 0.008;
    maxSpeed = 60;
  }
  else KP = 0.8;                                                 //we're far from setpoint, use aggressive tuning parameters

  PIDoutput = PIDController(us_distance, distance_setpoint, KP, 0.02, 0.2); // 0.008 , 0.02, 0.2


  int Lmotor_power = (constrain(cruiseSpeed + PIDoutput, 0 , maxSpeed));
  int Rmotor_power = (constrain(cruiseSpeed - PIDoutput, 0 , maxSpeed));
  //DLMP = Lmotor_power;
  //DRMP = Rmotor_power;
  //return DLMP, DRMP;


  if (us_distance > (distance_setpoint + 0.5)) RobotForward(Lmotor_power);
  else if (us_distance < (distance_setpoint - 0.5)) RobotReverse(Rmotor_power);
  else if ((us_distance > distance_setpoint - 0.5) && (us_distance < distance_setpoint + 0.5))
  {
    RobotStop();
    delay(10);
    subTask = subtask;

  }

}


void follow_leftSideWall(int distance_setpoint) // int subtask

{

  //left_us_Distance();
  float KP = 0.08;
  float gap = abs(distance_setpoint - left_us_distance);                //distance away from setpoint

  const int cruiseSpeed = 0;
  int maxSpeed = 120;

//  if (gap < 20) {                                                 //we're close to setpoint, use conservative tuning parameters
//    KP = 0.008;
//    maxSpeed = 90;
//  }
//  else KP = 0.8;                                                 //we're far from setpoint, use aggressive tuning parameters

  int PIDoutput1 = PIDController(left_us_distance, distance_setpoint, KP, 0.02, 0.2); // 0.008 , 0.02, 0.2


  int Lmotor_power = (constrain(cruiseSpeed + PIDoutput1, 0 , maxSpeed));
  int Rmotor_power = (constrain(cruiseSpeed - PIDoutput1, 0 , maxSpeed));
  //DLMP = Lmotor_power;
  //DRMP = Rmotor_power;
  //return DLMP, DRMP;

  //RobotForwardCentering(Rmotor_power, Lmotor_power);
  if (left_us_distance > (distance_setpoint + 1.5)) RobotForwardCentering(60, Lmotor_power);
  else if (left_us_distance < (distance_setpoint - 1.5)) RobotForwardCentering(Rmotor_power, 60);
  else if ((left_us_distance > distance_setpoint - 1.5) && (left_us_distance < distance_setpoint + 1.5))
  {
    RobotForwardCentering(60, 60);
  }

}

void maintain_rightWalldistance(int distance_setpoint, int subtask) // int subtask

{

  right_us_Distance();
  float KP = 6.08;
  float gap = abs(distance_setpoint - right_us_distance);                //distance away from setpoint

  const int cruiseSpeed = 50;
  int maxSpeed = 130;
  //we're far from setpoint, use aggressive tuning parameters

  PIDoutput = PIDController(right_us_distance, distance_setpoint, KP, 0.02, 1.2); // 0.008 , 0.02, 0.2


  int Lmotor_power = (constrain(cruiseSpeed + PIDoutput, 100 , maxSpeed));
  int Rmotor_power = (constrain(cruiseSpeed - PIDoutput, 100 , maxSpeed));

  if (right_us_distance > (distance_setpoint + 2.5)) RobotRightSlide(Rmotor_power);//(Rmotor_power, Lmotor_power);
  else if (right_us_distance < (distance_setpoint - 2.5)) RobotLeftSlide(Lmotor_power);//(Lmotor_power, Rmotor_power);
  else if ((right_us_distance > distance_setpoint - 2.5) && (right_us_distance < distance_setpoint + 2.5))
  {
    RobotStop();
    delay(10);
    subTask = subtask;

  }

}

void frontWall_alignment(int distance_setpoint, int subtask) // int subtask

{

  leftFront_us_Distance();
  rightFront_us_Distance();

  float KP = 0.08;

  const int cruiseSpeed = 0;
  int maxSpeed = 60;                                               //we're far from setpoint, use aggressive tuning parameters

  int leftPID_Out = PIDController(leftFront_us_distance, distance_setpoint, KP, 0.02, 0.2); // 0.008 , 0.02, 0.2
  int rightPID_Out = PIDController(rightFront_us_distance, distance_setpoint, KP, 0.02, 0.2);

  int Lmotor_power = (constrain(cruiseSpeed + leftPID_Out, 0 , maxSpeed));
  int Rmotor_power = (constrain(cruiseSpeed + rightPID_Out, 0 , maxSpeed));

  //F4W_independent_MecDrive(Lmotor_power, Lmotor_power, Rmotor_power, Rmotor_power );

  if (rightFront_us_distance < distance_setpoint - 0.5) {
    Rmotor_power = -Rmotor_power;
  }
  else if (rightFront_us_distance > distance_setpoint + 0.5) {
    Rmotor_power = Rmotor_power;
  }

  if (leftFront_us_distance < distance_setpoint - 0.5) {
    Lmotor_power = -Lmotor_power;
  }
  else if (leftFront_us_distance > distance_setpoint + 0.5) {
    Lmotor_power = Lmotor_power;
  }


  F4W_independent_MecDrive(Lmotor_power, Lmotor_power, Rmotor_power, Rmotor_power );

  if (((rightFront_us_distance > distance_setpoint - 0.5) && (rightFront_us_distance < distance_setpoint + 0.5)) && ((leftFront_us_distance > distance_setpoint - 0.5) && (leftFront_us_distance < distance_setpoint + 0.5)))
  {
    RobotStop();
    delay(10);
    subTask = subtask;

  }

}








void heading_update()

{

  //MPU6050_YAW();

  if (yawCalibPeriod < 300)
  {
    yawval_correction = yawVal;
    yawCalibPeriod ++;
    //recalib_heading = false;
  }

  else {
    YAWVAL = (yawVal - yawval_correction) + default_Heading ;
    if (YAWVAL < 0) {
      YAWVAL = YAWVAL + 360;
    }
    else if (YAWVAL > 360) {
      YAWVAL = YAWVAL - 360;
    }
  }

}




void recalibrate_yaw()
{
  yawCalibPeriod = 0;
  heading_update();
}




void find_heading(int heading_setpoint, int subtask)// int subtask

{
  MPU6050_YAW();
  heading_update();

  float KP = 0.08;
  float gap = abs(heading_setpoint - YAWVAL);             // heading away from setpoint

  if (gap < 10) KP = 0.008;                               //we're close to setpoint, use conservative tuning parameters
  else KP = 0.8;                                          //we're far from setpoint, use aggressive tuning parameters
  PIDoutput = PIDController(YAWVAL, heading_setpoint, KP, 0.02, 0.2); // 0.008 , 0.02, 0.2

  const int cruiseSpeed = 0;
  const int maxSpeed = 80;
  int Lmotor_power = (constrain(cruiseSpeed + PIDoutput, 0 , maxSpeed));
  int Rmotor_power = (constrain(cruiseSpeed - PIDoutput, 0 , maxSpeed));

  if (YAWVAL > (heading_setpoint + 1.0))  RobotRotate_CClockwise(Lmotor_power);
  else if (YAWVAL < (heading_setpoint - 1.0)) RobotRotateClockwise(Rmotor_power);
  else {
    RobotStop();
    subTask = subtask;
  }
}

/*
  void sidewaysRecenter_byDistance(int dist_setpoint, int heading_setpoint, int radar, int subtask)// int subtask
  {

  us_Distance(); // sideUltrasonic

  float KP = 0.8;
  float gap = abs(heading_setpoint - YAWVAL);             // heading away from setpoint

  if (gap < 10) KP = 0.8; //0.008;                               //we're close to setpoint, use conservative tuning parameters
  else KP = 2.08;                                          //we're far from setpoint, use aggressive tuning parameters
  PIDController(YAWVAL, heading_setpoint, KP, 0.002, 6.25); // 0.008 , 0.02, 0.2 //6.25

  const int cruiseSpeed = 150;
  const int maxSpeed = 190;


  if (us_distance < dist_setpoint - 0.5)
  {
    us_Distance();
    int Lmotor_power = (constrain(cruiseSpeed + PIDoutput, 100 , maxSpeed));
    int Rmotor_power = (constrain(cruiseSpeed - PIDoutput, 100 , maxSpeed));

    RobotLeftSlideCtl(Lmotor_power, Rmotor_power);
  }

  else if (us_distance > dist_setpoint + 0.5) {
    us_Distance();
    int Lmotor_power = (constrain(cruiseSpeed + PIDoutput, 100 , maxSpeed));
    int Rmotor_power = (constrain(cruiseSpeed - PIDoutput, 100 , maxSpeed));

    RobotRightSlideCtl(Rmotor_power, Lmotor_power);
  }

  else if ((us_distance > dist_setpoint - 0.5) && (us_distance < dist_setpoint + 0.5))
  {
    RobotStop();
    //delay(10);
    subTask = subtask;

  }
  }
*/






void maintain_heading(int _direction, float heading_setpoint)

{
  heading_update();

  float KP = 0.8;
  float gap = abs(heading_setpoint - YAWVAL);             // heading away from setpoint

  if (gap < 10) KP = 0.8; //0.008;                               //we're close to setpoint, use conservative tuning parameters
  else KP = 2.08;                                          //we're far from setpoint, use aggressive tuning parameters
  PIDoutput = PIDController(YAWVAL, heading_setpoint, KP, 0.002, 6.25); // 0.008 , 0.02, 0.2 //6.25

  const int cruiseSpeed = 80;
  const int maxSpeed = 130;

  PIDController(YAWVAL, heading_setpoint, 0.8, 0.002, 0.25);

  int Lmotor_power = (constrain(cruiseSpeed + PIDoutput, 50 , maxSpeed));
  int Rmotor_power = (constrain(cruiseSpeed - PIDoutput, 50 , maxSpeed));

  if (_direction == 1)RobotForwardCentering(Rmotor_power, Lmotor_power);
  if (_direction == -1)RobotReverseCentering(Rmotor_power, Lmotor_power);

}







void headingControlled_strafing(int strafe_direction, int heading_setpoint)// int subtask

{
  heading_update();
  float KP = 0.8;
  float gap = abs(heading_setpoint - YAWVAL);             // heading away from setpoint

  if (gap < 10) KP = 0.8; //0.008;                               //we're close to setpoint, use conservative tuning parameters
  else KP = 2.08;                                          //we're far from setpoint, use aggressive tuning parameters
  PIDoutput = PIDController(YAWVAL, heading_setpoint, KP, 0.002, 6.25); // 0.008 , 0.02, 0.2 //6.25

  const int cruiseSpeed = 150;
  const int maxSpeed = 190;


  if (strafe_direction == 1)
  {
    int Lmotor_power = (constrain(cruiseSpeed + PIDoutput, 150 , maxSpeed));
    int Rmotor_power = (constrain(cruiseSpeed - PIDoutput, 150 , maxSpeed));

    RobotLeftSlideCtl(Lmotor_power, Rmotor_power);
  }

  else if (strafe_direction == 0) {
    int Lmotor_power = (constrain(cruiseSpeed + PIDoutput, 150 , maxSpeed));
    int Rmotor_power = (constrain(cruiseSpeed - PIDoutput, 150 , maxSpeed));

    RobotRightSlideCtl(Rmotor_power, Lmotor_power);
  }

  //subTask = subtask;

}









void readLines() {

  lineSensor[0] = !digitalRead(36);
  lineSensor[1] = !digitalRead(34);
  lineSensor[2] = !digitalRead(32);
  lineSensor[3] = !digitalRead(30);
  lineSensor[4] = !digitalRead(28);
  lineSensor[5] = !digitalRead(26);
  lineSensor[6] = !digitalRead(24);
  lineSensor[7] = !digitalRead(22);



  //int avg;
  avg = (7000 * lineSensor[0] + 6000 * lineSensor[1] + 5000 * lineSensor[2] + 4000 * lineSensor[3] + 3000 * lineSensor[4] + 2000 * lineSensor[5] + 1000 * lineSensor[6] + 0 * lineSensor[7]) / (lineSensor[0] + lineSensor[1] + lineSensor[2] + lineSensor[3] + lineSensor[4] + lineSensor[5] + lineSensor[6] + lineSensor[7]);

  //Serial.print("Position =  ");
  //Serial.println(avg);
  //return avg;

}





void readRearLines()
{
  rear_lineSensor[0] = !digitalRead(37);
  rear_lineSensor[1] = !digitalRead(35);
  rear_lineSensor[2] = !digitalRead(33);
  rear_lineSensor[3] = !digitalRead(31);
  rear_lineSensor[4] = !digitalRead(29);
  rear_lineSensor[5] = !digitalRead(27);
  rear_lineSensor[6] = !digitalRead(25);
  rear_lineSensor[7] = !digitalRead(23);


  
  avgRear = (7000 * rear_lineSensor[0] + 6000 * rear_lineSensor[1] + 5000 * rear_lineSensor[2] + 4000 * rear_lineSensor[3] + 3000 * rear_lineSensor[4] + 2000 * rear_lineSensor[5] + 1000 * rear_lineSensor[6] + 0 * rear_lineSensor[7]) / (rear_lineSensor[0] + rear_lineSensor[1] + rear_lineSensor[2] + rear_lineSensor[3] + rear_lineSensor[4] + rear_lineSensor[5] + rear_lineSensor[6] + rear_lineSensor[7]);

  //Serial.print("RearPosition =  ");
  //Serial.println(avg);
  //return avg;

}







void track_whiteline()
{
  int setpoint = 3500;
  readLines();
  int motorDiff = 0;
  float Kp, Ki, Kd;
  unsigned int locus;
  int derivative, proportional;

  locus = avg;
  proportional = ((int)locus - setpoint);
  derivative = proportional + lastProportional;
  integral = integral + proportional;
  lastProportional = proportional;

  float gap = abs(setpoint - locus);
  if (gap < 100) Kp = 0.008;


  Kp = 1.08; // 0.082
  Ki = 0; // 0.02
  Kd = 1.28;  // 0.2
  motorDiff = proportional * Kp + integral * Ki + derivative * Kd;

  const int cruiseSpeed = 60;
  const int minSpeed = 0;
  const int maxSpeed = 100;

  Lmotor_power = (constrain(cruiseSpeed - motorDiff, minSpeed , maxSpeed));
  Rmotor_power = (constrain(cruiseSpeed + motorDiff, minSpeed , maxSpeed));
  RobotForwardCentering(Rmotor_power, Lmotor_power);

}



void track_RearWhiteline()
{
  readRearLines();
  int setpoint = 3500;
  int motorDiff = 0;
  float Kp, Ki, Kd;
  unsigned int locus;
  int derivative, proportional;

  locus = avgRear;
  proportional = ((int)locus - setpoint);
  derivative = proportional + lastProportional;
  integral = integral + proportional;
  lastProportional = proportional;

  float gap = abs(setpoint - locus);
  if (gap < 100) Kp = 0.008;


  Kp = 2.08; // 0.082
  Ki = 0.2; // 0.02
  Kd = 1.28;  // 0.2
  motorDiff = proportional * Kp + integral * Ki + derivative * Kd;

  const int cruiseSpeed = 60;
  const int minSpeed = 0;
  const int maxSpeed = 100;

  Lmotor_power = (constrain(cruiseSpeed - motorDiff, minSpeed, maxSpeed));
  Rmotor_power = (constrain(cruiseSpeed + motorDiff, minSpeed, maxSpeed));
  RobotReverseCentering(Lmotor_power, Rmotor_power);

}






void positionRobot_lineLocus(int frontline_setPoint, int rearline_setPoint)
{
  front_lineLocus (frontline_setPoint);
  rear_lineLocus (rearline_setPoint);
//  if (avgRear == rearline_setPoint && avg == frontline_setPoint)
//  {
//    RobotStop();
// 
//    }
}






void rear_lineLocus (int locus_setPoint)

{
  int motorDiff = 0;
  float Kp, Ki, Kd;
  unsigned int locus;
  int derivative, proportional;
  readRearLines();
  locus = avgRear;
  proportional = ((int)locus - locus_setPoint);
  derivative = proportional + lastProportional;
  integral = integral + proportional;
  lastProportional = proportional;

  float gap = abs(locus_setPoint - locus);
  if (gap < 100) Kp = 0.008;
  else {
    Kp = 2.08;
  }
  Ki = 0;
  Kd = 1.8;
  motorDiff = proportional * Kp + integral * Ki + derivative * Kd;
  const int cruiseSpeed = 0;
  const int maxSpeed = 80;

  Lmotor_power = (constrain(cruiseSpeed - motorDiff, -100 , maxSpeed));
  Rmotor_power = (constrain(cruiseSpeed + motorDiff, -100 , maxSpeed));
  RightRearMotor(Rmotor_power);
  LeftRearMotor(Lmotor_power);
  //RLRMP = Rmotor_power;
  //RLLMP = Lmotor_power;

  if ((locus < locus_setPoint + 500) && (locus > locus_setPoint - 500)) {
    RobotStop();

  }
}





void front_lineLocus (int locus_setPoint)

{
  int motorDiff = 0;
  float Kp, Ki, Kd;
  unsigned int locus;
  int derivative, proportional;
  readLines();
  locus = avg;
  proportional = ((int)locus - locus_setPoint);
  derivative = proportional + lastProportional;
  integral = integral + proportional;
  lastProportional = proportional;

  float gap = abs(locus_setPoint - locus);
  if (gap < 100) Kp = 0.008;
  else {
    Kp = 2.08;
  }
  Ki = 0;
  Kd = 1.8;
  motorDiff = proportional * Kp + integral * Ki + derivative * Kd;
  const int cruiseSpeed = 0;
  const int maxSpeed = 100;

  Lmotor_power = (constrain(cruiseSpeed - motorDiff, -100 , maxSpeed));
  Rmotor_power = (constrain(cruiseSpeed + motorDiff, -100 , maxSpeed));
  RightFrontMotor(Lmotor_power);
  LeftFrontMotor(Rmotor_power);
  if ((locus < locus_setPoint + 500) && (locus > locus_setPoint - 500)) {
    RobotStop();
    //break;
  }
  //RLRMP = Rmotor_power;
  //RLLMP = Lmotor_power;
}






int intersectionDetection() { // detect intersections and keep their counts

  if (digitalRead(23) == LOW && digitalRead(25) == LOW && digitalRead(27) == LOW && digitalRead(35) == LOW && digitalRead(37) == LOW) {

    while (digitalRead(23) == LOW && digitalRead(25) == LOW && digitalRead(27) == LOW && digitalRead(35) == LOW && digitalRead(37) == LOW) {

    }
    lineCount++;
  }
}


int rearLine_intersectionDetection() { // detect intersections and keep their counts

  if (digitalRead(23) == LOW && digitalRead(25) == LOW && digitalRead(27) == LOW && digitalRead(29) == LOW ) { //digitalRead(22) == LOW && digitalRead(24) == LOW && digitalRead(26) == LOW && digitalRead(28) == LOW &&

    while (digitalRead(23) == LOW && digitalRead(25) == LOW && digitalRead(27) == LOW && digitalRead(29) == LOW) { //digitalRead(22) == LOW && digitalRead(24) == LOW && digitalRead(26) == LOW && digitalRead(28) == LOW && digitalRead(30) == LOW && digitalRead(32) == LOW && digitalRead(34) == LOW)

    }
    rear_lineCount++;
    //return rear_lineCount;

  }
}

int rearLine_intersectionDetectionOne() { // detect intersections and keep their counts

  if (digitalRead(14) == LOW && digitalRead(15) == LOW ) { //digitalRead(22) == LOW && digitalRead(24) == LOW && digitalRead(26) == LOW && digitalRead(28) == LOW &&

    while (digitalRead(14) == LOW && digitalRead(15) == LOW ) { //digitalRead(22) == LOW && digitalRead(24) == LOW && digitalRead(26) == LOW && digitalRead(28) == LOW && digitalRead(30) == LOW && digitalRead(32) == LOW && digitalRead(34) == LOW)
      //delay(250);
      digitalRead(14);
      digitalRead(15);
    }
    rear_lineCountOne++;
    //return rear_lineCount;
  }
}




void Husky_camera_trackObject() {

  if (!huskylens.request()) RobotStop();
  else if (!huskylens.isLearned()) RobotStop();
  else if (!huskylens.available()) RobotStop();
  else
  {

    while (huskylens.available())
    {
      HUSKYLENSResult result = huskylens.read();

      if (result.xCenter <= 140) RobotLeftSlide(motor_speed);
      else if (result.xCenter >= 200) RobotRightSlide(motor_speed);
      else if ((result.xCenter >= 140) && (result.xCenter >= 140)) {
        if (result.width <= 20) RobotForward(motor_speed);
        else if (result.width > 20) RobotStop();

      }
      else RobotStop();

    }
  }
}
