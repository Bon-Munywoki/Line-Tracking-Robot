

// AUTHORED BY JKD

// ROBOT INITIAL SETUP AND CALIBRATION OF SENSORS SUBROUTINE APP

bool robot_active = false;

void setupDelay(int period) {
  delay(period);
}



void sensorSETUP()
{

  //if (!init_serial)Serial.begin(115200);

  pinMode(trigPin, OUTPUT); // ultrasonic pins
  pinMode(echoPin, INPUT);
  pinMode(left_us_trigPin, OUTPUT); // ultrasonic pins
  pinMode(left_us_echoPin, INPUT);
  pinMode(right_us_trigPin, OUTPUT); // ultrasonic pins
  pinMode(right_us_echoPin, INPUT);
  pinMode(leftFront_us_trigPin, OUTPUT); // ultrasonic pins
  pinMode(leftFront_us_echoPin, INPUT);
  pinMode(rightFront_us_trigPin, OUTPUT); // ultrasonic pins
  pinMode(rightFront_us_echoPin, INPUT);



  pinMode(colorS0, OUTPUT); // color pins
  pinMode(colorS1, OUTPUT);
  pinMode(colorS2, OUTPUT);
  pinMode(colorS3, OUTPUT);
  pinMode(colorSensorOut, INPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(ledPin, LOW);
  digitalWrite(colorS0, HIGH);
  digitalWrite(colorS1, LOW);

  //  pinMode(redpin, OUTPUT); // LED indicator RGB pins
  //  pinMode(bluepin, OUTPUT);
  //  pinMode(greenpin, OUTPUT);
}


void motorPinSETUP()
{
  pinMode(RFMpin1, OUTPUT);
  pinMode(RFMpin2, OUTPUT);
  pinMode(LFMpin1, OUTPUT);
  pinMode(LFMpin2, OUTPUT);
  pinMode(RRMpin1, OUTPUT);
  pinMode(RRMpin2, OUTPUT);
  pinMode(LRMpin1, OUTPUT);
  pinMode(LRMpin2, OUTPUT);
  pinMode(RFM_EN, OUTPUT);
  pinMode(LFM_EN, OUTPUT);
  pinMode(RRM_EN, OUTPUT);
  pinMode(LRM_EN, OUTPUT);
}


//void huskylensSETUP()
//{
//  mySerial.begin(9600);
//  while (!huskylens.begin(mySerial))
//  {
//    //digitalWrite(13, HIGH); Indicator Light
//  }
//  //digitalWrite(13, LOW); Indicator
//
//}



void servoSETUP()
{
  
  arm_parking(90);
  //setAngle(50, 145, 180, 170, 90, 700);
  //us_servo.write(90);
  base_servo.attach(BASE_SERVO, 440, 2400);
  shld_servo.attach(SHLD_SERVO, 440, 2400);
  elb_servo.attach(ELB_SERVO, 440, 2400);
  wrist_servo.attach(WRST_SERVO, 440, 2400);
  gripper_servo.attach(GRIPPER_SERVO, 440, 2400);
  //us_servo.attach(US_SERVO, 440, 2400);

}





void MPU6050_SETUP()
{
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(93);
  mpu.setYGyroOffset(70);
  mpu.setZGyroOffset(31);
  mpu.setZAccelOffset(1038);                                // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {

    mpu.setDMPEnabled(true);

    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    MPU6050Ready = true;

  } else {
    MPU6050Ready = false;
  }
  MPU6050_YAW();

}

//void EnableRobot()
//{
//  while (digitalRead(RotaryPushButton) == HIGH)
//  {
//    digitalRead(RotaryPushButton);
//    if (digitalRead(RotaryPushButton) == LOW) {
//      robot_active = true;
//      break;
//    }
//  }
//}

/*

  void INFOTAINMENT_SETUP(){

  lcd.begin(16, 2);

  menu.add_screen(welcome_screen);
  }

*/
