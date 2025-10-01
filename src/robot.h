#pragma once

#include "chassis.h"

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
    
    
public:
    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);
    void SetDestination(Pose path[], int size);
    void SetDistance(float distance);
    void SetAngle(float angle);

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
    void HandleDestination(void);

    float DistToPoint(const Pose& dest);
    float AngleToPoint(const Pose& dest);
};
