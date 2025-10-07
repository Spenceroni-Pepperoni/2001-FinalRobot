#include "robot.h"
#include "servo32u4.h"
#include "chassis.h"
#include "BlueMotor.h"

Servo32U4Pin12 claw;
Servo32U4Pin5 slide;

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();
    elevator.setup();
    elevator.reset();
    claw.attach();
    claw.setTargetPos(2000);

    /**
     * TODO: Set pin 13 HIGH when navigating and LOW when destination is reached.
     * Need to set as OUTPUT here.
     */
}

void Robot::EnterIdleState(void)
{
    chassis.Stop();

    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;
}



void Robot::LiftState(LIFT_STATE state)
{
    String stateString;
    if(state == LIFT_GROUND)
    {
        liftState = LIFT_GROUND;
        stateString = "LIFT_GROUND";
        elevator.moveTo(LIFT_GROUND); 
    }
    else if(state == LIFT_FIRST)
    {
        liftState = LIFT_FIRST;
        stateString = "LIFT_FIRST";

        elevator.moveTo(LIFT_FIRST);
    }
    else if(state == LIFT_SECOND)
    {
        liftState = LIFT_SECOND;
        stateString = "LIFT_SECOND";
        elevator.moveTo(LIFT_SECOND); 
    }
    else if(state == LIFT_TOP)
    {
        liftState = LIFT_TOP;
        stateString = "LIFT_TOP";
        elevator.moveTo(LIFT_TOP); 
    }
    TeleplotPrint(("STATE: " + stateString).c_str(), 0);
}

void Robot::LiftDelay()
{
    // int target = liftState;
    // while(elevator.getPosition() != target)
        delay(500);
}

void Robot::ClawState(CLAW_STATE state) {
    if(state == CLAW_OPEN)
    {
        claw.setTargetPos(1500); // Open the claw
        clawDelay();
        delay(100);
        slide.setTargetPos(500); // Extend the slide
        slideDelay();
        delay(100);
        TeleplotPrint("STATE: CLAW OPEN", 0);

    }
    else if(state == CLAW_CLOSED)
    {
        claw.setTargetPos(1900); // Close the claw
        clawDelay();
        delay(100);
        slide.setTargetPos(2500); // Retract the slide
        slideDelay();
        delay(100);
        TeleplotPrint("STATE: CLAW OPEN", 0);

    }
}

void Robot::clawDelay() {
    while(!claw.update())
        delay(20);
}

void Robot::slideDelay() {
    while(!slide.update())
        delay(20);
}


void Robot::cubePhase(void)
{
    //Grabbing bottom cube and bringing it to bottom shelf
    LiftState(LIFT_GROUND); // Make sure lift is down
    LiftDelay();
    ClawState(CLAW_CLOSED); //grab bottom cube
    LiftState(LIFT_FIRST); //bring to first shelf
    LiftDelay();
    ClawState(CLAW_OPEN); //release cube

    //Grabbing middle cube and bringing it to top shelf

    LiftState(LIFT_SECOND); //bring lift to middle shelf
    LiftDelay();
    ClawState(CLAW_CLOSED); //grab middle cube
    LiftState(LIFT_TOP); //bring to top shelf
    LiftDelay();
    ClawState(CLAW_OPEN); //release cube
    //LiftState(LIFT_GROUND); //bring lift back to ground
    LiftDelay();
    

}





/**
 * The main loop for your robot. Process both synchronous events (motor control),
 * and asynchronous events (distance readings, etc.).
*/
void Robot::RobotLoop(void) 
{
     /**
     * Run the chassis loop, which handles low-level control.
     */
    Twist velocity;
    if(chassis.ChassisLoop(velocity))
    {
        // We do FK regardless of state
        UpdatePose(velocity);
        //chassis.SetMotorEfforts(220,-220);
        
        /**
         * Here, we break with tradition and only call these functions if we're in the 
         * DRIVE_TO_POINT state. CheckReachedDestination() is expensive, so we don't want
         * to do all the maths when we don't need to.
         * 
         * While we're at it, we'll toss DriveToPoint() in, as well.
         */ 
        if(robotState == ROBOT_DRIVE_TO_POINT)
        {
            DriveToPoint();
            if(CheckReachedDestination()) HandleDestination();
        }
        if(robotState == ROBOT_DRIVE_DIST)
        {
            DriveToDist();
            if(CheckReachedDestination()) HandleDestination();
        }
        if(robotState == ROBOT_TURN_TO_ANGLE)
        {
            TurnToAngle();
            if(CheckReachedDestination()) HandleDestination();
        }
        if(robotState == ROBOT_ELEVATOR){
            
        }
    }
}
