#pragma once

class BlueMotor
{
public:
    BlueMotor();
    void setEffort(int effort);
    void moveTo(long position);
    long getPosition();
    void reset();
    void setup();
    void setPIDGains(float kP, float kI, float kD);
    void updatePID();
    void setTarget(long target);
    bool atTarget();

private:
    void setEffort(int effort, bool clockwise);
    static void isr();
    const int tolerance = 5;
    const int PWMOutPin = 11;
    const int AIN2 = 4;
    const int AIN1 = 13;

    float Kp;
    float Ki;
    float Kd;
    long targetPosition;
    long lastError;
    long integral;
    unsigned long lastTime;
};