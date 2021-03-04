#include "safetydroneflip/safetydroneflip.h"

#include "drivers/time.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "msp/msp.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/boardalignment.h"

#include "io/motors.h"

#include "build/debug.h"

//check the battery

#define UnsafeValue 430 //unsafe value for the battery normally 430// meervoud van 1 = 0.01 v
#define UnsafeValueToHigh 480 //unsafe value for the battery normally 430// meervoud van 1 = 0.01 v

int BATTERYCRITICAL= 0;


void Motors_out(void) //tuns off motors when neccesary
{
    uint8_t motorCount = getMotorCount();

    for (int i = 0; i < motorCount; i++)
    {
        motor[i] = 0;

    }

}

void FlipDrone(int Motorthrotle) //controlls the motors
{ 
    uint8_t motorCount = getMotorCount();

    for (int i = 0; i < motorCount; i++)
    {
        motor[i] = 0;
    }
    //led is the back
    motor[0] = Motorthrotle; //right back
    motor[1] = 0;   //right front
    motor[2] = Motorthrotle;   //left back
    motor[3] = 0;   //left front
    

}

int safetydroneflipBatteryisunsafeget(void){
    return BATTERYCRITICAL;
}

uint16_t GetThrottle(int reset) //get the throttle for the drone
{
    static unsigned long time = 0;
    static uint16_t Motorthrotle = 950;
    time += 1;

    if (reset == 1){ //if the drone is upside down the function is reset
        Motorthrotle = 950;
        time = 0;
        return 0;
    }

    if (time <= 12500) //tries to flip for 3 sec after this the drone will up the motor throtle
    {
        if ((time%12000)== 0)
        {
            Motorthrotle += 50;
            if (Motorthrotle >= 1500)
            {
                Motorthrotle = 1500;

            }
        }


        return Motorthrotle;
    }

    if ((time > 12500)&& (time < 25000)) //after 3 sec the motors will turn of for 3 sec
    {
        return 0;
    }
    if (time >= 25000) //after total of 6 sec time is reset
    {
        time = 0;
        return 0;
    }

    return 0;
}



void safetydroneflipMain (void){

    float orientation = isupsidedown(); //used to check in matrix if the drone is upside down <-.5 is fine

    static uint16_t Motorthrotle = 950; //motor throtle given to the motors of the drone

    static unsigned long time = 0; 
    time += 1;
    
    uint16_t Voltage_curent = getBatteryVoltageLatest();
    static uint16_t lowestvalue = 0;
    static uint16_t Previouslowestvalue = 500;
    
    static float Voltage_sm = 0.0;
    const float Alpha = 0.15;
    Voltage_sm = (1.0-Alpha)*Voltage_sm+Alpha*Voltage_curent;

   
    static uint32_t previousMS = 0;

    if ( (millis()-previousMS) <= 30000){
       if  (Voltage_sm < Previouslowestvalue){
           Previouslowestvalue = Voltage_sm;
       }   
       debug[2]= Previouslowestvalue;     
    }
    if ((millis()-previousMS) > 30000){
        previousMS = millis();
        lowestvalue = Previouslowestvalue;
        Previouslowestvalue = 500;
        debug[3]=50;

    }
    
    if ((lowestvalue >= UnsafeValue) || (BATTERYCRITICAL == 1)|| (Voltage_sm > UnsafeValueToHigh))
    { 

        if (orientation < -0.5) //if upside down
        {
            Motors_out();
            GetThrottle(1);

            BATTERYCRITICAL = 0;
            time = 0;
            Motorthrotle = 950;
        }
        else //not upside down or sensor not working
        {
            Motorthrotle = GetThrottle(0);
            FlipDrone(Motorthrotle);
        }
    }

    if (((lowestvalue < UnsafeValue) && (BATTERYCRITICAL == 1))){ //extra safety
        time += 1;
        if (time > 300000){ // aproximitly a minute
            
            Motorthrotle = GetThrottle(1);
            Motors_out();
            time = 0;
            BATTERYCRITICAL = 0;

        }
    }
}