#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>

long oldValue = 0;
long newValue;
volatile long count = 0;
unsigned time = 0;

const int ENCA = 0;
const int ENCB = 1;

int oldA = 0;
int oldB = 0;
int newA = 0;
int newB = 0;

BlueMotor::BlueMotor()
{
    Kp = 2.0;
    Ki = 0.1;
    Kd = 0.5;
    targetPosition = 0;
    lastError = 0;
    integral = 0;
    lastTime = 0;
}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    attachInterrupt(digitalPinToInterrupt(ENCA), isr, CHANGE);
    reset();
}

long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}


void BlueMotor::isr()
{
    if ( digitalRead(13) == 0 )
    {
        count++;
    }
    else{
        count--;
    }

    oldA = newA;
    oldB = newB;
}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

void BlueMotor::setPIDGains(float kp, float ki, float kd){
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void BlueMotor::setTarget(long target){
    targetPosition = target; 
    integral = 0;
}

void BlueMotor::updatePID(){
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastTime;

    if (deltaTime >= 10){
        long currentPosition = getPosition();
        long error = targetPosition - currentPosition;

        float P = Kp * error;

        integral += error * deltaTime;

        integral = constrain(integral, -5000, 5000);
        float I = Ki * integral;

        float derivative ((error - lastError)/((float)deltaTime));
        float D = Kd * derivative;

        float output = P+I+D;

        int motorOutput = constrain((int)output, -200, 200);
        setEffort(motorOutput);

        lastError = error;
        lastTime = currentTime;

    }
}

bool BlueMotor::atTarget(){
    long currentPosition = getPosition();
    return abs(targetPosition-currentPosition) <= tolerance;
}


void BlueMotor::moveTo(long target)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                     //then stop
    if (getPosition() < target-5){
        
    } else if (getPosition() > target + 5){

    }else{
        setEffort(0);
    }

}
