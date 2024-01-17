


// AUTHORED BY JKD


int Lmotor_power;
int Rmotor_power;
int lastProportional = 0;
int integral = 0;
int PIDoutput = 0;




//********************************* PID CONTROL ADAPTABLE TO VARIOUS SENSORS OR APPLICATIONS **********************************************************




int PIDController( int input, int setpoint, float Kp, float Ki, float Kd) {

  int PIDout = 0;
  int k = 40;
  int myPosition;
  int derivative, proportional;

  myPosition = input * k;
  proportional = ((int)myPosition - (setpoint * k));
  derivative = proportional + lastProportional;
  integral = integral + proportional;
  lastProportional = proportional;

  return PIDout  = proportional * Kp + integral * Ki + derivative * Kd;

}
