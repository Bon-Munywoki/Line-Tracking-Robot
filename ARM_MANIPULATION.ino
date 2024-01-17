

// AUTHORED BY JKD


#define BASE_SERVO 3
#define SHLD_SERVO 4
#define ELB_SERVO  5
#define WRST_SERVO 6
#define GRIPPER_SERVO 7
//#define US_SERVO   44

Servo base_servo;
Servo shld_servo;
Servo elb_servo;
Servo wrist_servo;
Servo gripper_servo;
Servo us_servo;


rampDouble arm1ramp;
rampDouble arm2ramp;
rampDouble baseramp;
rampDouble wrstramp;
rampDouble gripramp;

int baseAngle;


// ROBOT ARM MOTION CONTROLL


void moveToAngle(double b, double a1, double a2, double wrst, double gripper)
{
  base_servo.write(b);
  shld_servo.write(a1);
  elb_servo.write(a2);
  wrist_servo.write(wrst);
  gripper_servo.write(gripper);
}


void setAngle(double b, double a1, double a2, double wrst, double gripper, unsigned long t)
{
  arm1ramp.go(a1, t, LINEAR, ONCEFORWARD);
  arm2ramp.go(a2, t, LINEAR, ONCEFORWARD);
  baseramp.go(b, t, LINEAR, ONCEFORWARD);
  wrstramp.go(wrst, t, LINEAR, ONCEFORWARD);
  gripramp.go(gripper, t, LINEAR, ONCEFORWARD);
  while (baseramp.isRunning()) {
    moveToAngle(baseramp.update(), arm1ramp.update(), arm2ramp.update(), wrstramp.update(), gripramp.update());
  }
}


void arm_parking(double gripper) {
  setAngle(50, 145, 180, 170, gripper, 1500);
  //setAngle(110, 20, 45, 0, gripper, 700);
}



void pick_objectTrailer(int subtask)

{
  setAngle(50, 95, 170, 150, 0, 1500);
  setAngle(50, 95, 170, 150, 80, 1500);
  //setAngle(50, 145, 180, 170, 90, 1500);
  arm_parking(80);
  subTask = subtask;
}

void place_objectTrailer(int subtask)
{
   setAngle(50, 95, 140, 150, 80, 1500);
   setAngle(50, 95, 140, 150, 40, 1500);
   arm_parking(80);
   subTask = subtask;
  }




void pick_lowerWhiteWheel(int subtask)

{
  arm_parking(30);
  //setAngle(80, 145, 180, 0, 0, 1250);
  //setAngle(50, 145, 180, 0, 0, 1250);
  setAngle(50, 48, 180, 0, 0, 1500);
  setAngle(50, 48, 180, 0, 90, 2000);
  setAngle(80, 48, 180, 0, 90, 1500);
  setAngle(80, 145, 180, 170, 90, 1500);
  arm_parking(90);
  subTask = subtask;
}




void pick_firstDeckWheel() {
  arm_parking(30);
  setAngle(50, 145, 180, 0, 30, 1250);
  setAngle(50, 60, 180, 5, 30, 1500);
  setAngle(50, 60, 180, 5, 90, 1500);
  setAngle(80, 60, 180, 5, 90, 1500);
  setAngle(80, 145, 180, 170, 90, 1500);
  setAngle(50, 145, 180, 170, 90, 500);
  //arm_parking(90);
}



void pickEngine(int subtask) {
  arm_parking(30);
  setAngle(50, 45, 180, 0, 0, 2000);
  setAngle(50, 45, 180, 0, 80, 1500);
  arm_parking(80);
  subTask = subtask;
}


void pickItemAtTopLevel() {
  arm_parking(60);
  setAngle(120, 70, 82, 85, 60, 500);
  setAngle(120, 70, 82, 90, 170, 250);
  setAngle(120, 70, 89, 110, 170, 250);
  setAngle(120, 50, 95, 45, 170, 250);
  arm_parking(170);
  armATcolor_pod();
}


void placeItemAtlowLevel() {
  arm_parking(170);
  setAngle(120, 40, 25, 30, 170, 500);
  setAngle(120, 95, 45, 120, 170, 1000);
  setAngle(120, 115, 40, 120, 70, 500);
  setAngle(120, 105, 50, 120, 70, 250);
  arm_parking(70);
}


void placeItemAtMidLevel() {
  arm_parking(110);
  setAngle(50, 100, 160, 90, 110, 1500);
  setAngle(50, 100, 160, 90, 30, 500);
  arm_parking(110);
}


void placeItemAtTopLevel() {
  arm_parking(170);
  setAngle(120, 60, 100, 80, 170, 500);
  setAngle(120, 70, 105, 45, 170, 500);
  setAngle(120, 70, 105, 45, 120, 250);
  setAngle(120, 70, 110, 45, 70, 250);
  setAngle(120, 50, 105, 45, 70, 250);
  arm_parking(70);
}

void pack_wheel(int itemColor)

{
  if (itemColor == red_color)  baseAngle = 35;
  else if (itemColor == green_color)  baseAngle = 85;
  else if (itemColor == blue_color)  baseAngle = 180;
  setAngle(baseAngle, 60, 100, 80, 170, 500);
}

void armATcolor_pod()
{
  setAngle(110, 20, 50, 20, 170, 500);
  setAngle(90, 70, 40, 15, 170, 1000);
  delay(10);
  readColor();
  delay(5);
}


void readColor()
{
  int redBuffer = 0;
  int blueBuffer = 0;
  int greenBuffer = 0;
  for (int i = 0; i < 500; i++)
  {
    int COLOR = color_detection();
    if (COLOR == red_color) redBuffer++;
    else if (COLOR == green_color) greenBuffer++;
    else if (COLOR == blue_color) blueBuffer++;
  }

  if (greenBuffer > redBuffer && greenBuffer > blueBuffer) OBJcolor = green_color;
  else if (blueBuffer > redBuffer && greenBuffer < blueBuffer) OBJcolor = blue_color;
  else if (greenBuffer < redBuffer && redBuffer > blueBuffer) OBJcolor = red_color;
}
