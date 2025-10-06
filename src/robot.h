#pragma once

#include "chassis.h"
#include "BlueMotor.h"

class Robot
{
protected:
    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE,
        ROBOT_DRIVE_TO_POINT,
        ROBOT_DRIVE_DIST,
        ROBOT_TURN_TO_ANGLE,
        ROBOT_ELEVATOR,
        
    };
    ROBOT_STATE robotState = ROBOT_IDLE;




    /* Define the chassis*/
    Chassis chassis;

    // For managing key presses
    String keyString;

    /**
     * For tracking current pose and the destination.
     */
    Pose currPose;
    Pose destPose;
    float targetDist;
    float targetAngle;
    bool liftPhase = false;
   
    
public:

    enum LIFT_STATE
    {
        LIFT_GROUND = 0,
        LIFT_FIRST = 500,
        LIFT_SECOND = 1000,
        LIFT_TOP = 1500
    };
    LIFT_STATE liftState = LIFT_GROUND;

    enum CLAW_STATE
    {
        CLAW_OPEN,
        CLAW_CLOSED
    };
    CLAW_STATE clawState = CLAW_OPEN;

    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);
    void LiftState(LIFT_STATE state);
    void LiftDelay();
    void ClawState(CLAW_STATE state);
    void clawDelay();
    void slideDelay();
    void cubePhase(void);
    void SetDestination(Pose path[], int size);
    void SetDistance(float distance);
    void SetAngle(float angle);

    BlueMotor elevator;

int AmmountOfPoints = 0;
int CurrentPoint = 0;
    Pose TargetPose[];



protected:
    /* State changes */    
    void EnterIdleState(void);

    // /* Navigation methods.*/
    void UpdatePose(const Twist& u);
    void DriveToPoint(void);
    void DriveToDist(void);
    void TurnToAngle(void);
    bool CheckReachedDestination(void);
    void DriveElevatorToPoint(int level);
    void HandleDestination(void);

    float DistToPoint(const Pose& dest);
    float AngleToPoint(const Pose& dest);
};

