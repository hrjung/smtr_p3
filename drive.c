/*
 * drive.c
 *
 * 		support 3 types of input
 *		1. potentiometer : 0 ~ 10V
 *		2. voltage input : 0 ~ 10V
 *		3. current input : 0 ~ 20mA
 *
 *		support 1 output can be voltage, current, frequency
 *
 *  Created on: 2017. 4. 5.
 *      Author: hrjung
 */


#include "uartstdio.h"
#include "hal.h"

#include "parameters.h"
//#include "inv_param.h"
#include "drive.h"
#include "state_func.h"
#include "freq.h"
//#include "err_trip.h"


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

const float_t pwm_tbl[4] = { 4.0, 8.0, 12.0, 16.0 };


/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
//extern HAL_Handle halHandle;
//extern USER_Params gUserParams;

uint16_t vf_foc_control = FOC_CONTROL;

/*******************************************************************************
 * EXTERNS
 */
#ifdef SUPPORT_VAR_PWM_FREQ
extern uint16_t pwm_freq_updated;
#endif

/*
 *  ======== local function ========
 */


/*
 *  ======== public function ========
 */

int DRV_setAccelTime(float_t value)
{
	if(value < MIN_ACCEL_TIME || value > MAX_ACCEL_TIME) return 1;

	iparam[ACCEL_TIME_INDEX].value.f = value;

	//STA_setResolution(ACCEL, DRV_calculateAccelRate_krpm(value));

	return 0;
}

int DRV_setDecelTime(float_t value)
{
	if(value < MIN_ACCEL_TIME || value > MAX_ACCEL_TIME) return 1;

	iparam[DECEL_TIME_INDEX].value.f = value;

	//STA_setResolution(DECEL, DRV_calculateAccelRate_krpm(value));

	return 0;
}

#ifdef SUPPORT_ACCEL_TIME_BASE
inline int DRV_isAccelTimeBaseMaxFreq(void)
{
	return(iparam[ACCEL_BASE_INDEX].value.l == ACC_TIME_BASE_MAX_FREQ);
}

int DRV_setAccelTimeBase(uint16_t base)
{
	if(base > ACC_TIME_BASE_NEXT_FREQ) return 1; // 0 or 1

	iparam[ACCEL_BASE_INDEX].value.l = (uint32_t)base;

	return 0;
}
#endif


float_t DRV_calculateAccelRate_krpm(float_t time, float_t diff)
{
	float_t spd_rpm, rate_krpm; //spd_range_rpm;

#ifdef SUPPORT_ACCEL_TIME_BASE
	if(DRV_isAccelTimeBaseMaxFreq())
		diff = FREQ_getMaxFreqValue();
#endif
	spd_rpm = FREQ_convertToSpeed(diff);
	rate_krpm = spd_rpm/(time * 1000.0); // scale to 1ms unit

	UARTprintf("Accel rate = %f\n", rate_krpm);
	if(rate_krpm == 0.0)
		UARTprintf("Accel rate is too small\n");

	return rate_krpm;
}

void DRV_enableVfControl(void)
{
	vf_foc_control = VF_CONTROL;
	//iparam[VF_FOC_SEL_INDEX].value.l = VF_CONTROL;
}

void DRV_enableFocControl(void)
{
	vf_foc_control = FOC_CONTROL;
	//iparam[VF_FOC_SEL_INDEX].value.l = FOC_CONTROL;
}

#if 0
int DRV_setTorqueLimit(float_t limit)
{
	if(limit < 100.0 || limit > 220.0) return 0;

	iparam[FOC_TORQUE_LIMIT_INDEX].value.f = limit;

	return 0;
}
#endif

int DRV_setEnergySave(int method)
{
	if(method < ESAVE_UNUSED || method > ESAVE_BOTH) return 1;

	iparam[ENERGY_SAVE_INDEX].value.l = (uint32_t)method;

	return 0;
}

#if 0
int DRV_setVoltageBoost(float_t value)
{
	if(value < 0.0 || value > 15.0) return 1;

	if(MAIN_isSystemEnabled()) return 1; // cannot update during motor running

	iparam[V_BOOST_INDEX].value.f = value;

	MAIN_applyBoost();

	return 0;
}
#endif

int DRV_setPwmFrequency(int value)
{
	if(value < PWM_4KHz || value > PWM_16KHz) return 1;

	if(MAIN_isSystemEnabled()) return 1;

#ifdef SUPPORT_VAR_PWM_FREQ
	if(iparam[PWM_FREQ_INDEX].value.l != value)
		pwm_freq_updated = 1;
#endif

	iparam[PWM_FREQ_INDEX].value.l = (uint32_t)value; //

	//gUserParams.pwmPeriod_kHz = pwm_tbl[param.ctrl.pwm_freq];
	//gUserParams.pwmPeriod_usec = 1000.0/gUserParams.pwmPeriod_kHz;

	return 0;
}

#if 0
int DRV_setSpdGainP(float_t value)
{
	if(value < 0.0 || value > 32767.0) return 1;

	param.ctrl.spd_P_gain = value; //

	return 0;
}

int DRV_setSpdGainI(float_t value)
{
	if(value < 0.0 || value > 32767.0) return 1;

	param.ctrl.spd_I_gain = value; //

	return 0;
}
#endif

float_t DRV_getPwmFrequency(void)
{
	return pwm_tbl[(int)iparam[PWM_FREQ_INDEX].value.l];
}

int DRV_setFanControl(uint16_t value)
{
	if(value != 0 && value != 1) return 1;

	iparam[FAN_COMMAND_INDEX].value.l = (uint32_t)value; //

	return 0;
}


int DRV_runForward(void)
{
	if(MAIN_isSystemEnabled())
		STA_setNextFreq(iparam[FREQ_VALUE_INDEX].value.f);
	else
	{
		MAIN_enableSystem();
		UARTprintf("start running motor freq=%f\n", iparam[FREQ_VALUE_INDEX].value.f);
	}

    return 0;
}

int DRV_runBackward(void)
{
	if(MAIN_isSystemEnabled())
		STA_setNextFreq(iparam[FREQ_VALUE_INDEX].value.f);
	else
	{
		MAIN_enableSystem();
		UARTprintf("start running motor freq=%f\n", iparam[FREQ_VALUE_INDEX].value.f);
	}

    return 0;
}

int DRV_stopMotor(void)
{
	int result = 1;

	STA_setStopCondition();
	UARTprintf("reduce speed to stop\n");

    return result;
}
