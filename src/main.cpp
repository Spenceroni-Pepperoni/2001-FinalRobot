#include <Arduino.h>
#include "robot.h"
#include <utils.h>
#include "Romi32U4Buttons.h"

Robot robot;
Romi32U4ButtonC ButtonC;
Romi32U4ButtonB ButtonB;

void setup() 
{
  Serial.begin(115200);

/**
 * If you define __SETUP_DEBUG__ in your .ini file, this line will make the program wait
 * until the Serial is set up so you can debug. WARNING: If you do that, you _must_ open 
 * the Serial Monitor for anything to happen!
 */
#ifdef __SETUP_DEBUG__
  while(!Serial){delay(5);}
#endif

  robot.InitializeRobot();
  

  // robot.elevator.moveTo(2000);

  //while(ButtonC.isPressed()){

  //robot.ClawState(Robot::CLAW_CLOSED);



    // delay(20000000000);
    // delay(2000);
    robot.cubePhase();
    // delay(1000);
    // robot.LiftState(Robot::LIFT_GROUND); //bring to first shelf

}

Pose TargetPoint[] = {
  {0, 25,0},
  {-25, 25,0},
  {-37.5,-7.5,0},
  // {24,24,0},
  // {-24,48,0},
  // {0,-24,0},
  // {12,40,0},
  // {-12,-40,0},
  // {0,0,0}

};

void loop() 
{
  //delay(5000);
  // robot.InitializeRobot();

  //robot.ClawState(Robot::CLAW_OPEN);

  //  delay(1000);

  while(ButtonC.isPressed()){


  }


  // robot.ClawState(Robot::CLAW_OPEN);
  while(ButtonB.isPressed()){
    robot.elevator.setEffort(-200);
    robot.elevator.setEffort(0);
  }
 
  
  robot.SetDestination(TargetPoint, 2);
  robot.RobotLoop();
}
