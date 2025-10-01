/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"
#include "chassis.h"
#include "Romi32U4MotorTemplate.h"

void Robot::UpdatePose(const Twist& twist)
{   
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */
    double deltaTime = (20.0/1000.0);
    //double prevU, prevOmega;


    double deltaTheta = twist.omega * deltaTime;
    double prevTheta = currPose.theta;
    currPose.theta += deltaTheta;



    double avgTheta = (prevTheta + currPose.theta)/2.0;

    currPose.x += deltaTime * (twist.u * cos(avgTheta));
    currPose.y += deltaTime * (twist.u * sin(avgTheta));


    //prevU = twist.u;
    //prevOmega = twist.omega;

    currPose.leftTick = twist.leftTick;
    currPose.rightTick = twist.rightTick;




#ifdef __NAV_DEBUG__
     TeleplotPrint("x", currPose.x);
     TeleplotPrint("y", currPose.y);
     TeleplotPrint("theta", (currPose.theta * (180/M_PI)));
#endif

}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(Pose path[], int size)
{ 
    AmmountOfPoints = size;
    destPose = path[CurrentPoint];
    /**
     * TODO: Turn on LED, as well.
     */
    //Serial.print("Setting dest to: ");
    // Serial.print(dest.x);
    // Serial.print(", ");
    // Serial.print(dest.y);
    // Serial.print('\n');

    //destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

void Robot::SetDistance(float distance) {
    targetDist = distance;
    robotState = ROBOT_DRIVE_DIST;
}

void Robot::SetAngle(float angle) {
    targetAngle = angle;
    robotState = ROBOT_TURN_TO_ANGLE;
}

float Robot::DistToPoint(const Pose& dest) {
    return sqrt(pow(dest.x-currPose.x, 2) + pow(dest.y-currPose.y, 2));
}

float Robot::AngleToPoint(const Pose& dest) {
    return WrapAngle180(atan2(dest.y-currPose.y, dest.x-currPose.x)*(180/M_PI) - (currPose.theta*(180/M_PI)));
}

bool Robot::CheckReachedDestination(void)
{
    bool retVal = false;
    /**
     * TODO: Add code to check if you've reached destination here.
     */
    if (robotState == ROBOT_DRIVE_DIST) {
        if(0 > targetDist - ((currPose.leftTick/LEFT_TICKS_PER_CM + currPose.rightTick/RIGHT_TICKS_PER_CM) / 2.0))
            retVal = true;
    }
    if (robotState == ROBOT_TURN_TO_ANGLE) {
        if(0 > (targetAngle - (currPose.theta * (180/M_PI))) )
            retVal = true;
    }
    if (robotState == ROBOT_DRIVE_TO_POINT) {
        if (5 > fabs(DistToPoint({destPose.x, destPose.y, destPose.theta})))
            retVal = true;
    }

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        
        float distToPoint = DistToPoint({destPose.x, destPose.y, destPose.theta});
        float angleToPoint = AngleToPoint({destPose.x, destPose.y, destPose.theta});

        if(distToPoint < 7) {
            angleToPoint = 0;
        }
        if(distToPoint < 4) {
            distToPoint = 0;
        }

#ifdef __NAV_DEBUG__
        TeleplotPrint("Distance: ", distToPoint);
        TeleplotPrint("Angle: ", (angleToPoint));

#endif

        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */
        if (fabs(distToPoint*50) > 50)   distToPoint = 5;
        //if (fabs(angleToPoint) > 75)    distToPoint = 0;
        if (fabs(angleToPoint*3) > 100)  angleToPoint = 50;

        chassis.SetMotorEfforts(distToPoint*10  - angleToPoint*3, distToPoint*10 + angleToPoint*2);
    }
}

void Robot::DriveToDist(void)
{
    if(robotState == ROBOT_DRIVE_DIST)
    {
        //double currentLeft = targetDist - (currPose.leftTick/LEFT_TICKS_PER_CM);
        //double currentRight = targetDist - (currPose.rightTick/LEFT_TICKS_PER_CM);



        

#ifdef __NAV_DEBUG__
        // if(robotState == ROBOT_DRIVE_DIST) TeleplotPrint("leftError", targetDist - (currPose.leftTick/LEFT_TICKS_PER_CM));
        //  if(robotState == ROBOT_DRIVE_DIST) TeleplotPrint("rightError", targetDist - (currPose.rightTick/RIGHT_TICKS_PER_CM));
        // //TeleplotPrint("Reached Destination", CheckReachedDestination());
#endif

        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */
        chassis.SetMotorEfforts(120,100);
    }
}

void Robot::TurnToAngle(void)
{
    if(robotState == ROBOT_TURN_TO_ANGLE)
    {   
        //double currentLeft = targetDist - (currPose.leftTick/LEFT_TICKS_PER_CM);
        //double currentRight = targetDist - (currPose.rightTick/LEFT_TICKS_PER_CM);



        

#ifdef __NAV_DEBUG__
        // if(robotState == ROBOT_DRIVE_DIST) TeleplotPrint("leftError", targetDist - (currPose.leftTick/LEFT_TICKS_PER_CM));
        //  if(robotState == ROBOT_DRIVE_DIST) TeleplotPrint("rightError", targetDist - (currPose.rightTick/RIGHT_TICKS_PER_CM));
        // //TeleplotPrint("Reached Destination", CheckReachedDestination());
#endif

        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */
        chassis.SetMotorEfforts(-60,50);
    }
}

// void Robot::EnterIdleState() {
//     chassis.SetMotorEfforts(0,0);
// }


void Robot::HandleDestination(void)
{
    /**
     * TODO: Stop and change state. Turn off LED.
     */

    // if the current point + 1 is less that or equal to the total points, then currentPoint++ (this lets the code move to the next point in the array) 
    if ((CurrentPoint + 1) <= AmmountOfPoints) CurrentPoint++;

    if (CurrentPoint == AmmountOfPoints) {
    chassis.SetMotorEfforts(0,0);
    robotState = ROBOT_IDLE;
    EnterIdleState();
    }
    else {
        destPose = TargetPose[CurrentPoint];
    }

    #ifdef __NAV_DEBUG__
        Serial.print("Done");
    #endif

}

