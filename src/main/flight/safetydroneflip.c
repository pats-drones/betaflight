#include <stdbool.h>

#include "drivers/time.h"
#include "drivers/dshot_command.h"

#include "flight/mixer.h"
#include "flight/safetydroneflip.h"
#include "flight/imu.h"

#include "msp/msp.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/boardalignment.h"

#include "io/motors.h"

#include "build/debug.h"

#define UNSAFE_VALUE 430 //unsafe value for the battery normally 430// meervoud van 1 = 0.01 v
#define UNSAFE_VALUE_HIGH 480 //unsafe value for the battery normally 430// meervoud van 1 = 0.01 v

bool batteryCriticalStatus= false;
bool setReverse = false; 

void Motors_reversed(void){
    dshotCommandWrite(2, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_REVERSED, DSHOT_CMD_TYPE_INLINE);
    dshotCommandWrite(0, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_REVERSED, DSHOT_CMD_TYPE_INLINE);       
    setReverse = true;     
}

void Motors_normal(void){
    dshotCommandWrite(2, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL, DSHOT_CMD_TYPE_INLINE);
    dshotCommandWrite(0, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL, DSHOT_CMD_TYPE_INLINE);
    setReverse = false;

}

void Motors_out(void) //tuns off motors 
{
    uint8_t motorCount = getMotorCount();

    for (int i = 0; i < motorCount; i++)
    {
        motor[i] = 0;
    }
}

void flipDrone(int Motorthrotle) //controlls the motors
{ 
    uint8_t motorCount = getMotorCount();
    for (int i = 0; i < motorCount; i++)
    {
        motor[i] = 0;
    }

    if (!setReverse) { 
        Motors_reversed();
    }  
    //led is the back
    if (Motorthrotle > 500){
    motor[0] = Motorthrotle-500; //right back
    motor[1] = Motorthrotle;   //right front
    motor[2] = Motorthrotle-500;   //left back
    motor[3] = Motorthrotle;   //left front
    }
}

bool batteryIsCritical(void){
    return batteryCriticalStatus;
}

uint16_t GetThrottle(int reset) //get the throttle for the drone
  {
    static uint16_t Motorthrottle = 950;
    static bool ValueUp = false; //sees if the value for Motorthrottle hase gone up
    static unsigned long Pervious_millis = 0;

    if (reset == 1){ //if the drone is upside down the function is reset
        Motorthrottle = 950;
        Pervious_millis = millis();
        return 0;
    }

    if ((millis()-Pervious_millis) <= 3000) //tries to flip for 3 sec after this the drone will up the motor throtle
    {
        if (ValueUp == false)
        {
            Motorthrottle += 50;
            if (Motorthrottle >= 1500)
            {
                Motorthrottle = 1500;

            }
        ValueUp = true;
        }
        return Motorthrottle;
    }
    if (((millis()-Pervious_millis)  > 3000)&& ((millis()-Pervious_millis)  < 6000)) //after 3 sec the motors will turn of for 3 sec
    {
        ValueUp = false;
        return 0;
    }
    if ((millis()-Pervious_millis) >= 6000) //after total of 6 sec time is reset
    {
        Pervious_millis = millis();
        return 0;
    }
    return 0;
}


void safetydroneflipMain (void){

    float orientation = isupsidedown(); //used to check in matrix if the drone is upside down <-.5 is fine

    static uint16_t Motorthrottle = 950; //motor throtle given to the motors of the drone
   
    uint16_t Voltage = getBatteryVoltageLatest();
    static uint16_t lowestvalue = 0;
    static uint16_t Previouslowestvalue = 500;
    
    static float voltageSmoothed = 0.0;
    const float Alpha = 0.15;
    voltageSmoothed = (1.0-Alpha)*voltageSmoothed+Alpha*Voltage;
   
    static unsigned long previousMS = 0;
    static unsigned long previousMsSafeVoltage = 0;

    if ( (millis()-previousMS) <= 30000){
       if  (voltageSmoothed < Previouslowestvalue){
           Previouslowestvalue = voltageSmoothed;
       }   
    }
    if ((millis()-previousMS) > 30000){
        previousMS = millis();
        lowestvalue = Previouslowestvalue;
        Previouslowestvalue = 500;
    }

    if ((lowestvalue >= UNSAFE_VALUE) && (batteryCriticalStatus == true)){
        previousMsSafeVoltage = millis(); 
    }

    if ((lowestvalue >= UNSAFE_VALUE) || (batteryCriticalStatus == true)|| (voltageSmoothed > UNSAFE_VALUE_HIGH))
    { 
        if (orientation < -0.5) //if upside down
        {
            if (setReverse) {        
                Motors_normal();
            }
            Motors_out();
            GetThrottle(1);
            batteryCriticalStatus = false;
            Motorthrottle = 950;
        }
        else //not upside down or sensor not working
        {
            batteryCriticalStatus = true;
            Motorthrottle = GetThrottle(0);
            flipDrone(Motorthrottle);
        }
    }

   if (((lowestvalue < UNSAFE_VALUE) && (batteryCriticalStatus == true))){ //extra safety turns the function off after a minute
        if ((millis()-previousMsSafeVoltage) > 60000){ 
            if (setReverse) {        
                Motors_normal();
            }
            Motorthrottle = GetThrottle(1);
            Motors_out();
            batteryCriticalStatus = false;
        }
    }
}