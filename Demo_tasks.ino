



void Object_pickUp()
{
  while (Task == 0)
  {
    us_Distance();
    switch (subTask)
    {
      case 0: // Leave start Zone
        while (us_distance > 22) {
          us_Distance();
          RobotForward(70);
          if (us_distance < 22) {
            RobotStop();
            subTask = 1;
          }
        }
        break;

      case 1: 
        
        maintain_distance(10, 2);
        
        break;

      case 2:
        pick_objectTrailer(3);
        break;

      case 3:
        DELAY_an_action(1140, RobotRotate_CClockwise, 70);
        RobotStop();
        subTask = 4;
        break;

      case 4:
        while (us_distance > 22) {
          us_Distance();
          //maintain_heading(1, 180);
          RobotForward(70);
          //heading_update();
          if (us_distance < 22) {
            RobotStop();
            placeItemAtMidLevel();
            subTask = 5;
          }
        }
        break;
    }
  }
}
