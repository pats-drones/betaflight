#include "safetydroneflip/safetydroneflip.h"

#include "drivers/time.h"

#include "flight/mixer.h"

#include "msp/msp.h"

#include "drivers/dshot_command.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/boardalignment.h"

#include "io/motors.h"

#include <stdbool.h>

#include "build/debug.h"

#define UNSAFE_VALUE 430 //unsafe value for the battery normally 430// meervoud van 1 = 0.01 v
#define UNSAFE_VALUE_HIGH 480 //unsafe value for the battery normally 430// meervoud van 1 = 0.01 v

bool batteryCriticalStatus= false;

void Motors_out(void) //tuns off motors when neccesary
{
    uint8_t motorCount = getMotorCount();

    for (int i = 0; i < motorCount; i++)
    {
        motor[i] = 0;
    }
}

void flipDrone(int Motorthrotle) //controlls the motors
{ 
    static bool setReverse = false;

    if (!setReverse) {        
        dshotCommandWrite(2, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_REVERSED, DSHOT_CMD_TYPE_INLINE);
        dshotCommandWrite(0, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_REVERSED, DSHOT_CMD_TYPE_INLINE);
        setReverse = true;
    }
    uint8_t motorCount = getMotorCount();
    for (int i = 0; i < motorCount; i++)
    {
        motor[i] = 0;
    }
    //Motorthrotle = 1000;
        
    //led is the back
    if (Motorthrotle !=0 ){
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
    static bool ValueUp = false;
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



 /*   if ( (millis()-previousMS) <= 30000){
       if  (voltageSmoothed < Previouslowestvalue){
           Previouslowestvalue = voltageSmoothed;
       }   
    }
    if ((millis()-previousMS) > 30000){
        previousMS = millis();
        lowestvalue = Previouslowestvalue;
        Previouslowestvalue = 500;

    }*/
    int16_t debugtest = (int16_t)(orientation * 10);
    int16_t debugtest2;
    debugtest2 = isupsidedown() * 10;
    static int test2 = 0; 
    test2++;
    debug[0]= test2;
    debug[1]= debugtest2;

    int16_t test =  isupsidedown() * 10;
    debug[2] = test;
    if ((lowestvalue >= UNSAFE_VALUE) || (batteryCriticalStatus == true)){
        previousMsSafeVoltage = millis(); 
    }

    //if ((lowestvalue >= UNSAFE_VALUE) || (batteryCriticalStatus == true)|| (voltageSmoothed > UNSAFE_VALUE_HIGH))
    if (millis()>10000)
    { 
        if (orientation < -0.5) //if upside down
        {
            Motors_out();
            GetThrottle(1);
            batteryCriticalStatus = false;
            Motorthrottle = 950;
            debug[3]= 30;
        }
        else //not upside down or sensor not working
        {
            batteryCriticalStatus = true;
            Motorthrottle = GetThrottle(0);
            flipDrone(Motorthrottle);
            debug[3]= 50;
        }
    }

   if (((lowestvalue < UNSAFE_VALUE) && (batteryCriticalStatus == true))){ //extra safety
        if (millis()-previousMsSafeVoltage > 60000){ // aproximitly a minute
            
            Motorthrottle = GetThrottle(1);
            Motors_out();
            batteryCriticalStatus = false;

        }
    }
}