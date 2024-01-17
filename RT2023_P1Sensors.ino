

// AUTHORED BY JKD


int lineSensor[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int rear_lineSensor[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//#define RotaryPushButton     


#define counterPin           A0

#define colorS0              A8 // yellow
#define colorS1              A9 // green
#define colorS2              A10 //black
#define colorS3              A11 //blue
#define colorSensorOut       A12 //brown
#define ledPin               A13 //orange



#define echoPin              A14
#define trigPin              A15

#define left_us_echoPin      51//42 // attach pin 42 to pin Echo of HC-SR04
#define left_us_trigPin      53//44

#define right_us_echoPin     49//30 //rightside ultrasonic sensor
#define right_us_trigPin     50//34


#define leftFront_us_echoPin      19//42 // attach pin 42 to pin Echo of HC-SR04
#define leftFront_us_trigPin      18//44

#define rightFront_us_echoPin     A7//30 //rightside ultrasonic sensor
#define rightFront_us_trigPin     A6//34





unsigned long us_cm;
unsigned long us_pm;
const unsigned long us_period = 50;

long us_duration;


unsigned long left_us_cm;
unsigned long left_us_pm;
const unsigned long left_us_period = 50;

long left_us_duration;

unsigned long right_us_cm;
unsigned long right_us_pm;
const unsigned long right_us_period = 50;

long right_us_duration;



const float maxDistance = 400.00;
const float distanceFactor = maxDistance / 100;





unsigned long leftFront_us_cm;
unsigned long leftFront_us_pm;
const unsigned long leftFront_us_period = 50;

long leftFront_us_duration;

unsigned long rightFront_us_cm;
unsigned long rightFront_us_pm;
const unsigned long rightFront_us_period = 50;

long rightFront_us_duration;











int us_Distance() {                                          // measure distance with ultrasonic

  us_cm = millis();

  if (us_cm > us_pm + us_period) {                           //

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    us_duration = pulseIn(echoPin, HIGH, 38000);
    //Calculating the distance
    us_distance = us_duration * 0.034 / 2;

    if (us_distance < 3) {
      us_distance = maxDistance;
    }

    us_pm = us_cm;
  }
}



int leftFront_us_Distance() { // measure distance with ultrasonic

  leftFront_us_cm = millis();

  if (leftFront_us_cm > leftFront_us_pm + leftFront_us_period) {

    int maxDistance = 400;
    digitalWrite(leftFront_us_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(leftFront_us_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(leftFront_us_trigPin, LOW);

    //Reads the echoPin
    leftFront_us_duration = pulseIn(leftFront_us_echoPin, HIGH, 38000);
    //Calculating the distance
    leftFront_us_distance = leftFront_us_duration * 0.034 / 2;

    if (leftFront_us_distance < 3) {
      leftFront_us_distance = maxDistance;
    }
    leftFront_us_pm = leftFront_us_cm;
  }
}



int rightFront_us_Distance() {                                          // measure distance with ultrasonic

  rightFront_us_cm = millis();

  if (rightFront_us_cm > rightFront_us_pm + rightFront_us_period) {                           //

    digitalWrite(rightFront_us_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(rightFront_us_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(rightFront_us_trigPin, LOW);

    rightFront_us_duration = pulseIn(rightFront_us_echoPin, HIGH, 38000);
    //Calculating the distance
    rightFront_us_distance = rightFront_us_duration * 0.034 / 2;

    if (rightFront_us_distance < 3) {
      rightFront_us_distance = maxDistance;
    }

    rightFront_us_pm = rightFront_us_cm;
  }
}





int left_us_Distance() { // measure distance with ultrasonic

  left_us_cm = millis();

  if (left_us_cm > left_us_pm + left_us_period) {

    int maxDistance = 400;
    digitalWrite(left_us_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(left_us_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(left_us_trigPin, LOW);

    //Reads the echoPin
    left_us_duration = pulseIn(left_us_echoPin, HIGH, 38000);
    //Calculating the distance
    left_us_distance = left_us_duration * 0.034 / 2;

    if (left_us_distance < 3) {
      left_us_distance = maxDistance;
    }
    left_us_pm = left_us_cm;
  }
}



int right_us_Distance() {                                          // measure distance with ultrasonic

  right_us_cm = millis();

  if (right_us_cm > right_us_pm + right_us_period) {                           //

    digitalWrite(right_us_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(right_us_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(right_us_trigPin, LOW);

    right_us_duration = pulseIn(right_us_echoPin, HIGH, 38000);
    //Calculating the distance
    right_us_distance = right_us_duration * 0.034 / 2;

    if (right_us_distance < 3) {
      right_us_distance = maxDistance;
    }

    right_us_pm = right_us_cm;
  }
}








// MPU 6050 IMU FOR ROBOT ORIENTATION AND HEADING STATUS

void dmpDataReady() {
  mpuInterrupt = true;
}




bool MPU6050_YAW()
{

  if (!dmpReady) return;                                 // if programming failed, don't try to do anything

  //mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();                        // get current FIFO count

  //if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {      // check for overflow (this should never happen unless our code is too inefficient)
  if (fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    int mpu_delayCount = 0;
    while (!mpu.dmpPacketAvailable() && mpu_delayCount < 1000) {

      delay(1);
      mpu_delayCount++;
    }

    if (mpu_delayCount >= 1000) return false;

  }

  while (fifoCount >= packetSize)
  {
    mpu.getFIFOBytes(fifoBuffer, packetSize);          // read a packet from FIFO

    fifoCount -= packetSize;                           // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
  }

  global_fifo_count = mpu.getFIFOCount();

  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


  yawVal = ypr[0] * 180 / M_PI;
  return true;

}


int color_detection()
{
  int redFreq = 0;
  int greenFreq = 0;
  int blueFreq = 0;

  digitalWrite(colorS0, HIGH);
  digitalWrite(colorS1, LOW);

  digitalWrite(colorS2, LOW);
  digitalWrite(colorS3, LOW);
  redFreq = pulseIn(colorSensorOut, LOW);

  digitalWrite(colorS2, HIGH);
  digitalWrite(colorS3, HIGH);
  greenFreq = pulseIn(colorSensorOut, LOW);

  digitalWrite(colorS2, LOW);
  digitalWrite(colorS3, HIGH);
  blueFreq = pulseIn(colorSensorOut, LOW);
  //int blueColor = map(blueFreq, 0, 100, 255, 0);

  if (redFreq < greenFreq && redFreq < blueFreq) color = red_color; // red color detected
  else if (greenFreq < redFreq && greenFreq < blueFreq) color = green_color; //  green color detected
  else if (blueFreq < redFreq && blueFreq < greenFreq) color = blue_color;
  else if (redFreq > 30 && greenFreq > 33 && blueFreq < 50) color = 4; // Yellow color detected
//    Serial.print("redFrequency: ");
//    Serial.print(redFreq);
//    Serial.print("        blueFrequency: ");
//    Serial.print(blueFreq);
//    Serial.print("        greenFrequency: ");
//    Serial.println(redFreq-blueFreq);
//    Serial.print(greenFreq);
//    Serial.println("");
//    delay(500);

  return color;

}
