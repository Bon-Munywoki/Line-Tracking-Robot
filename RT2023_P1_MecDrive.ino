
// AUTHORED BY JKD

void motorForward(int pin1, int pin2, int enPin, int mspeed)
{
  analogWrite(enPin, mspeed);
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
}

void motorReverse(int pin1, int pin2, int enPin, int mspeed)
{
  analogWrite(enPin, mspeed);
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
}

void motorStop(int pin1, int pin2, int enPin, int mspeed)
{
  analogWrite(enPin, mspeed);
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
}

void RightFrontWheel(int _direction, int motor_speed)
{
  if (_direction == -1) motorReverse(RFMpin1, RFMpin2, RFM_EN, motor_speed);
  else if (_direction == 1) motorForward(RFMpin1, RFMpin2, RFM_EN, motor_speed);
  else if (_direction == 0) motorStop(RFMpin1, RFMpin2, RFM_EN, motor_speed);
}

void LeftFrontWheel(int _direction, int motor_speed)
{
  if (_direction == -1) motorReverse(LFMpin1, LFMpin2, LFM_EN, motor_speed);
  else if (_direction == 1) motorForward(LFMpin1, LFMpin2, LFM_EN, motor_speed);
  else if (_direction == 0) motorStop(LFMpin1, LFMpin2, LFM_EN, motor_speed);
}

void RightRearWheel(int _direction, int motor_speed)
{
  if (_direction == -1) motorReverse(RRMpin1, RRMpin2, RRM_EN, motor_speed);
  else if (_direction == 1) motorForward(RRMpin1, RRMpin2, RRM_EN, motor_speed);
  else if (_direction == 0) motorStop(RRMpin1, RRMpin2, RRM_EN, motor_speed);
}

void LeftRearWheel(int _direction, int motor_speed)
{
  if (_direction == -1) motorReverse(LRMpin1, LRMpin2, LRM_EN, motor_speed);
  else if (_direction == 1) motorForward(LRMpin1, LRMpin2, LRM_EN, motor_speed);
  else if (_direction == 0) motorStop(LRMpin1, LRMpin2, LRM_EN, motor_speed);
}

void RobotForward(int motor_speed)
{
  RightFrontWheel(1, motor_speed);
  LeftFrontWheel(1, motor_speed);
  RightRearWheel(1, motor_speed);
  LeftRearWheel(1, motor_speed);
}

void RobotReverse(int motor_speed)
{
  RightFrontWheel(-1, motor_speed);
  LeftFrontWheel(-1, motor_speed);
  RightRearWheel(-1, motor_speed);
  LeftRearWheel(-1, motor_speed);
}


void RobotReverseCentering(int motor_speedA, int motor_speedB)
{
  RightFrontWheel(-1, motor_speedA);
  LeftFrontWheel(-1, motor_speedB);
  RightRearWheel(-1, motor_speedA);
  LeftRearWheel(-1, motor_speedB);
}


void RobotForwardCentering(int motor_speedA, int motor_speedB)
{
  RightFrontWheel(1, motor_speedB);
  LeftFrontWheel(1, motor_speedA);
  RightRearWheel(1, motor_speedB);
  LeftRearWheel(1, motor_speedA);
}


void RobotLeftSlide(int motor_speed)
{
  RightFrontWheel(1, motor_speed);
  LeftFrontWheel(-1, motor_speed);
  RightRearWheel(-1, motor_speed);
  LeftRearWheel(1, motor_speed);
}

void RobotRightSlide(int motor_speed)
{
  RightFrontWheel(-1, motor_speed);
  LeftFrontWheel(1, motor_speed);
  RightRearWheel(1, motor_speed);
  LeftRearWheel(-1, motor_speed);
}

void RobotForwardLeft()
{
  RightFrontWheel(1, motor_speed);
  LeftFrontWheel(0, motor_speed);
  RightRearWheel(0, motor_speed);
  LeftRearWheel(1, motor_speed);
}

void RobotForwardRight()
{
  RightFrontWheel(0, motor_speed);
  LeftFrontWheel(1, motor_speed);
  RightRearWheel(1, motor_speed);
  LeftRearWheel(0, motor_speed);
}

void RobotReverseRight()
{
  RightFrontWheel(-1, motor_speed);
  LeftFrontWheel(0, motor_speed);
  RightRearWheel(0, motor_speed);
  LeftRearWheel(-1, motor_speed);
}

void RobotReverseLeft()
{
  RightFrontWheel(0, motor_speed);
  LeftFrontWheel(-1, motor_speed);
  RightRearWheel(-1, motor_speed);
  LeftRearWheel(0, motor_speed);
}

void RobotRotateClockwise(int motor_speed)
{
  RightFrontWheel(-1, motor_speed);
  LeftFrontWheel(1, motor_speed);
  RightRearWheel(-1, motor_speed);
  LeftRearWheel(1, motor_speed);
}

void RobotRotate_CClockwise(int motor_speed)
{
  RightFrontWheel(1, motor_speed);
  LeftFrontWheel(-1, motor_speed);
  RightRearWheel(1, motor_speed);
  LeftRearWheel(-1, motor_speed);
}

void RobotRearOffset_CW()
{
  RightFrontWheel(-1, motor_speed);
  LeftFrontWheel(1, motor_speed);
  RightRearWheel(0, motor_speed);
  LeftRearWheel(0, motor_speed);
}

void RobotRearOffset_CCW(int motor_speed)
{
  RightFrontWheel(1, motor_speed);
  LeftFrontWheel(-1, motor_speed);
  RightRearWheel(0, motor_speed);
  LeftRearWheel(0, motor_speed);
}

void RobotConcerning_CW()
{
  RightFrontWheel(0, motor_speed);
  LeftFrontWheel(1, motor_speed);
  RightRearWheel(0, motor_speed);
  LeftRearWheel(1, motor_speed);
}

void RobotConcerning_CCW()
{
  RightFrontWheel(1, motor_speed);
  LeftFrontWheel(0, motor_speed);
  RightRearWheel(1, motor_speed);
  LeftRearWheel(0, motor_speed);
}

void RobotLeftSlideCtl(int motor_speedA, int motor_speedB)
{
  RightFrontWheel(1, motor_speedA);
  LeftFrontWheel(-1, motor_speedA);
  RightRearWheel(-1, motor_speedB);
  LeftRearWheel(1, motor_speedB);
}

void RobotRightSlideCtl(int motor_speedA, int motor_speedB)
{
  RightFrontWheel(-1, motor_speedA);
  LeftFrontWheel(1, motor_speedA);
  RightRearWheel(1, motor_speedB);
  LeftRearWheel(-1, motor_speedB);
}

void RobotStop()
{
  int motor_Sspeed = 250;
  RightFrontWheel(0, motor_Sspeed);
  LeftFrontWheel(0, motor_Sspeed);
  RightRearWheel(0, motor_Sspeed);
  LeftRearWheel(0, motor_Sspeed);
}


void RightFrontMotor(int motor_speed)
{
  if (motor_speed < 0) motorReverse(RFMpin1, RFMpin2, RFM_EN, abs(motor_speed));
  else  motorForward(RFMpin1, RFMpin2, RFM_EN, motor_speed);
  //else motorStop(RFMpin1, RFMpin2, RFM_EN, motor_speed);

}

void LeftFrontMotor(int motor_speed)
{
  if (motor_speed < 0) motorReverse(LFMpin1, LFMpin2, LFM_EN, abs(motor_speed));
  else  motorForward(LFMpin1, LFMpin2, LFM_EN, motor_speed);

}

void RightRearMotor(int motor_speed)
{
  if (motor_speed < 0) motorReverse(RRMpin1, RRMpin2, RRM_EN, abs(motor_speed));
  else  motorForward(RRMpin1, RRMpin2, RRM_EN, motor_speed);

}

void LeftRearMotor(int motor_speed)
{
  if (motor_speed < 0) motorReverse(LRMpin1, LRMpin2, LRM_EN, abs(motor_speed));
  else motorForward(LRMpin1, LRMpin2, LRM_EN, motor_speed);
}


void F4W_independent_MecDrive(int LFM, int LRM, int RFM, int RRM)

{
  RightRearMotor(RRM);
  RightFrontMotor(RFM);
  LeftRearMotor(LRM);
  LeftFrontMotor(LFM);
}
