#include "flight/safetydroneflip.h"
#include "drivers/time.h"
#include "drivers/dshot_command.h"
#include "flight/mixer.h"
#include "flight/imu.h"
#include "msp/msp.h"
#include "sensors/battery.h"
#include "io/motors.h"

#define SAFETY_FLIP_THRESHOLD_VOLTAGE 435
#define PANIC_THRESHOLD_VOLTAGE 480 

#define FLIP_THROTTLE_START_VALUE 500

#define MINUTE 60000
#define SECOND 1000

bool motorsReversed = false; 

bool flipDroneMode = false;
bool panicMode = false;

bool flipStarted = false;

void setBackMotorsDirectionReversed(void) {
    dshotCommandWrite(2, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_REVERSED, DSHOT_CMD_TYPE_INLINE);
    dshotCommandWrite(3, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_REVERSED, DSHOT_CMD_TYPE_INLINE);       
    motorsReversed = true;     
}

void setBackMotorsDirectionNormal(void) {
    dshotCommandWrite(2, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL, DSHOT_CMD_TYPE_INLINE);
    dshotCommandWrite(3, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL, DSHOT_CMD_TYPE_INLINE);
    motorsReversed = false;
}

void motorsOff(void) {
    uint8_t motorCount = getMotorCount();
    for (int i = 0; i < motorCount; i++)
    {
        motor[i] = 0;
    }
}

void applyThrottle(int Motorthrottle) { 
    if (!motorsReversed) 
        setBackMotorsDirectionReversed();

    motor[0] = Motorthrottle; //right front
    motor[1] = Motorthrottle; //left front
    motor[2] = Motorthrottle; //right back
    motor[3] = Motorthrottle; //left back
}

uint16_t flipThrottle() {
    static uint16_t flipThrottle = FLIP_THROTTLE_START_VALUE; 
    static unsigned long flipTime = 0;

    if (!flipStarted) {
        flipThrottle = FLIP_THROTTLE_START_VALUE;
        flipTime = millis();
        flipStarted = true;
    }

    if (millis() - flipTime < 1000)
        return flipThrottle;
    if (millis() - flipTime < 2000)
        return 0;
    
    flipThrottle += 50;
    if (flipThrottle > 1500)
        flipThrottle = 1500;
    flipTime = millis();

    return flipThrottle;
}

void returnToNormal(){
    motorsOff();
    flipStarted = false;

    if (motorsReversed)   
        setBackMotorsDirectionNormal();
    
    panicMode = false;
    flipDroneMode = false;
}

float lowestVoltageOverTime(float smoothedVoltage) { 
    // measure the voltage in the intervals between charging
    static float lowestValue = 500;
    static float previousLowestValue = 0;
    static unsigned long voltageMeasureStartTime = 0;
    
    if  (smoothedVoltage < lowestValue)
        lowestValue = smoothedVoltage;
    
    if (millis() - voltageMeasureStartTime > MINUTE * 1.2){ // charging breaks every 60 seconds
        voltageMeasureStartTime = millis();
        previousLowestValue = lowestValue ;
        lowestValue = 500;
    }
    return previousLowestValue;
}

float movingAverage(float alpha, float value, float smoothed_value) {
    return alpha * value + (1.0 - alpha) * smoothed_value;
}

void flipDroneIfVoltageTooHigh(void){
    if (millis() < 10000)
        return;

    static float smoothedVoltage = 0.0;
    static unsigned long lastMsSafeVoltage = 0;
    static unsigned long lastMsUnsafeVoltage = 0;

    smoothedVoltage = movingAverage(0.01, (float)getBatteryVoltageLatest(), smoothedVoltage);
    float batteryVoltage = lowestVoltageOverTime(smoothedVoltage);

    if (batteryVoltage < SAFETY_FLIP_THRESHOLD_VOLTAGE) {
       lastMsSafeVoltage = millis();
    }
    else {
        flipDroneMode = true;
        lastMsUnsafeVoltage = millis();
    }

    if (smoothedVoltage > PANIC_THRESHOLD_VOLTAGE || (millis() - lastMsSafeVoltage) > 5 * MINUTE) 
        panicMode = true;
    
    if  ((panicMode || flipDroneMode) && millis() - lastMsUnsafeVoltage > 15 * SECOND) { 
        returnToNormal();
    }
    
    if (panicMode) {
        applyThrottle(1500);
        return;
    }

    if (flipDroneMode) { 
        if (!isUpsideDown()) {
            uint16_t throttle = flipThrottle();
            applyThrottle(throttle);
            return;
        }
        returnToNormal();
    }
}

bool unsafeBatteryVoltage(void) { 
    return panicMode || flipDroneMode;
}