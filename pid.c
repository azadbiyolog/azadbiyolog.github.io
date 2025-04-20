#include "pid.h"
#include <math.h>

PID_Param_t myPID[ENUM_WHEELMAX];
float error;

void PID_init(void) {

    myPID[FRONT_RIGHT].Kp = 0.64; //0,64
	myPID[FRONT_RIGHT].Ki = 0.43; //0,43
	myPID[FRONT_RIGHT].Kd = 0.014; //0
	myPID[FRONT_RIGHT].Ts = 0.1;
	myPID[FRONT_RIGHT].Set_point = 35.0;			// ---> encoder count value
	myPID[FRONT_RIGHT].Anti_windup = 1;
	myPID[FRONT_RIGHT].Anti_windup_error = 100;
	myPID[FRONT_RIGHT].Outmin = 0;
	myPID[FRONT_RIGHT].Outmax = 100;


	myPID[FRONT_LEFT].Kp = 0.64;
	myPID[FRONT_LEFT].Ki = 0.43;
	myPID[FRONT_LEFT].Kd = 0.014;
	myPID[FRONT_LEFT].Ts = 0.1;
	myPID[FRONT_LEFT].Set_point = 35.0;
	myPID[FRONT_LEFT].Anti_windup = 1;
	myPID[FRONT_LEFT].Anti_windup_error = 100;
	myPID[FRONT_LEFT].Outmin = 0;
	myPID[FRONT_LEFT].Outmax = 100;


	myPID[REAR_RIGHT].Kp = 0.64;
	myPID[REAR_RIGHT].Ki = 0.43;
	myPID[REAR_RIGHT].Kd = 0.014;
	myPID[REAR_RIGHT].Ts = 0.1;
	myPID[REAR_RIGHT].Set_point = 35.0;
	myPID[REAR_RIGHT].Anti_windup = 1;
	myPID[REAR_RIGHT].Anti_windup_error = 100;
	myPID[REAR_RIGHT].Outmin = 0;
	myPID[REAR_RIGHT].Outmax = 100;


	myPID[REAR_LEFT].Kp = 0.64;
	myPID[REAR_LEFT].Ki = 0.43;
	myPID[REAR_LEFT].Kd = 0.014;
	myPID[REAR_LEFT].Ts = 0.1;
	myPID[REAR_LEFT].Set_point = 35.0;
	myPID[REAR_LEFT].Anti_windup = 1;
	myPID[REAR_LEFT].Anti_windup_error = 100;
	myPID[REAR_LEFT].Outmin = 20;
	myPID[REAR_LEFT].Outmax = 100;
}


float PID_Calculation(float input, PID_Param_t *par) {
    error = par->Set_point - input;
    par->Pterm = par->Kp * error;
    par->Dterm = par->Kd * (input - par->prev_input) / par->Ts;
    par->Iterm = par->Ki * par->error_sum * par->Ts;

    if(par->Anti_windup) {
        if(fabs(error) > par->Anti_windup_error) {

            par->out = par->Pterm + par->Dterm;
        }
        else {

        	par->out = par->Pterm +par->Iterm + par->Dterm;
        }
    }
    else {

    	par->out = par->Pterm +par->Iterm - par->Dterm;
    }

    par->error_sum += error;
    if(par->error_sum > ERRORSUM_MAX){
    	par->error_sum = ERRORSUM_MAX;
    }
    else if(par->error_sum < ERRORSUM_MIN){
    	par->error_sum = ERRORSUM_MIN;
    }
    if(par->out > par->Outmax) {
        par->out = par->Outmax;
    }
    if(par->out < par->Outmin) {
        par->out = par->Outmin;
    }

    par->prev_input = input;
    return par->out;
}


