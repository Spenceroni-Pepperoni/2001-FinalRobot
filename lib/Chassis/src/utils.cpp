#include "utils.h"

void TeleplotPrint(const char* var, float value)
{
    Serial.print('>');
    Serial.print(var);
    Serial.print(':');
    Serial.print(value);
    Serial.print('\n');
}

float WrapAngle180(float theta) {
    while (theta > 180) theta -= 360;
    while (theta < -180) theta += 360;
    return theta;
}
