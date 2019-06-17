/*
 * protect.c
 *
 *  Created on: 2017. 4. 17.
 *      Author: hrjung
 */
#include <stdint.h>
#include <math.h>

#include "uartstdio.h"

#include "parameters.h"
#include "motor_param.h"
//#include "inv_param.h"
#include "drive.h"
#include "timer_handler.h"
#include "protect.h"
#include "err_trip.h"
#include "common_tools.h"

/*******************************************************************************
 * MACROS
 */

#define MAIN_isInitRelayInitialized()   (internal_status.relay_enabled==1)

/*******************************************************************************
 * CONSTANTS
 */
#define ETH_ONE_MINUTE		60

#define TRIP_LEVEL_MIN		30
#define TRIP_LEVEL_WARN		200
#define TRIP_LEVEL_TRIP		200

#define TRIP_TIME_WARN_MAX	30
#define TRIP_TIME_TRIP_MAX	60

#define IPM_TEMPERATURE_FAN_ON	(60.0)
#define IPM_TEMPERATURE_LIMIT	(100.0)
#define MOTOR_TEMPERATURE_LIMIT	(50.0)

#define REGEN_RESISTANCE_VALUE_MIN	(150.0)
#define REGEN_RESISTANCE_VALUE_MAX	(500.0)

/*******************************************************************************
 * TYPEDEFS
 */
// 220V
#define DC_VOLTAGE_INIT_RELAY_OFF_220V		(100.0)
#define DC_VOLTAGE_INIT_RELAY_ON_220V		(186.0)
#define DC_VOLTAGE_OVER_TRIP_LEVEL_220V		(404.5)

#define DC_VOLTAGE_START_REGEN_LEVEL_220V 	(373.4)
#define DC_VOLTAGE_END_REGEN_LEVEL_220V		(357.8)

// 380V
#define DC_VOLTAGE_INIT_RELAY_OFF_380V		(100.0)
#define DC_VOLTAGE_INIT_RELAY_ON_380V		(322.4)
#define DC_VOLTAGE_OVER_TRIP_LEVEL_380V		(740.6) //(698.6)

//#define DC_VOLTAGE_START_REGEN_LEVEL_380V 	(550.9) // for test only
//#define DC_VOLTAGE_END_REGEN_LEVEL_380V 	(535.0)
#define DC_VOLTAGE_START_REGEN_LEVEL_380V 	(630.0)//(644.9)
#define DC_VOLTAGE_END_REGEN_LEVEL_380V 	(618.0)

#define REGEN_V_MARGIN			(0.9)
#define MAX_REGEN_CURRENT		(10.0)

// OVL warning state
enum
{
	OVL_WARN_NOMAL = 0,
	OVL_WARN_WARNING,
};

typedef struct
{
	//float_t dc_volt_init_relay_off;
	float_t dc_volt_init_relay_on;
	float_t dc_volt_over_trip_level;

	float_t dc_volt_start_regen_level;
	float_t dc_volt_end_regen_level;

} protect_dc_st;

/*******************************************************************************
 * LOCAL VARIABLES
 */
#ifdef SUPPORT_OFFSET_MEASURE
extern uint16_t gOffsetMeasureFlag;
#endif

static int evt_flag=0;
int regen_duty=70; //default

uint16_t ud_volt_count=0, ov_volt_count=0;

protect_dc_st protect_dc;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */




/*******************************************************************************
 * GLOBAL VARIABLES
 */
uint16_t ovl_alarm_enable=0;

/*******************************************************************************
 * EXTERNS
 */
extern uint32_t secCnt;


extern float_t MAIN_getIave(void);
extern void MAIN_readCurrent(void);
extern int MAIN_isOverCurrent(void);
//extern void MAIN_setRegenDuty(float_t resist, uint32_t power);

/*
 *  ======== local function ========
 */


/*
 *  ======== public function ========
 */

int OVL_isOverloadTripEnabled(void)
{
	return (iparam[OVL_ENABLE_INDEX].value.l == 1);
}

void OVL_enbleOverloadTrip(uint16_t enable)
{
	iparam[OVL_ENABLE_INDEX].value.l = enable;
}

int OVL_setWarningLevel(uint16_t level)
{
	if(level < TRIP_LEVEL_MIN || level > TRIP_LEVEL_WARN) return 1;

	iparam[OVL_WARN_LIMIT_INDEX].value.l = level;

	MPARAM_setOvlWarnLevel(iparam[OVL_WARN_LIMIT_INDEX].value.l);

	return 0;
}

int OVL_setTripLevel(uint16_t level)
{
	if(level < TRIP_LEVEL_MIN || level > TRIP_LEVEL_TRIP) return 1;

	iparam[OVL_TR_LIMIT_INDEX].value.l = level;

	MPARAM_setOvlTripLevel(iparam[OVL_TR_LIMIT_INDEX].value.l);

	return 0;
}

int OVL_setWarningTime(uint16_t dur)
{
	if(dur > TRIP_TIME_WARN_MAX) return 1;

	iparam[OVL_WR_DURATION_INDEX].value.l = dur;

	return 0;
}

int OVL_setTripTime(uint16_t dur)
{
	if(dur > TRIP_TIME_TRIP_MAX) return 1;

	iparam[OVL_TR_DURATION_INDEX].value.l = dur;

	return 0;
}


int OVL_isOverloadWarnCondition(float_t cur_level)
{
	return (cur_level > dev_const.warn_level);
}

int OVL_processOverloadWarning(float_t cur_val)
{
	static int ovl_state = OVL_WARN_NOMAL;

	switch(ovl_state)
	{
	case OVL_WARN_NOMAL:
		// check OVL_warning condition
		if(OVL_isOverloadWarnCondition(cur_val))
		{
			if(!TMR_isTimerEnabled(OVERLOAD_WARN_START_TSIG))
			{
				//start timer
				TMR_startTimerSig(OVERLOAD_WARN_START_TSIG, (float_t)iparam[OVL_WR_DURATION_INDEX].value.l);
				UARTprintf("OVL warning started %f at %d\n", cur_val, (int)(secCnt/10));
			}
		}

		// if timer is running, condition resolved before timeout
		if(TMR_isTimerEnabled(OVERLOAD_WARN_START_TSIG))
		{
			if(!OVL_isOverloadWarnCondition(cur_val)) // condition resolved
			{
				TMR_disableTimerSig(OVERLOAD_WARN_START_TSIG);
				UARTprintf("OVL warning resolved %f\n", cur_val);
			}
		}

		// timeout -> change WARNING state
		if(TMR_isTimeout(OVERLOAD_WARN_START_TSIG))
		{
			TMR_disableTimerSig(OVERLOAD_WARN_START_TSIG);
			ovl_state = OVL_WARN_WARNING;

			if(ovl_alarm_enable == 0)
			{
				UARTprintf("OVL ALARM set at %d\n", (int)(secCnt/10));
				ovl_alarm_enable = 1; // TODO : set alarm
			}
		}
		break;

	case OVL_WARN_WARNING:
		// check OVL_warning condition
		if(!OVL_isOverloadWarnCondition(cur_val))
		{
			if(!TMR_isTimerEnabled(OVERLOAD_WARN_END_TSIG))
			{
				//start timer
				TMR_startTimerSig(OVERLOAD_WARN_END_TSIG, (float_t)iparam[OVL_WR_DURATION_INDEX].value.l);
				UARTprintf("OVL warning end started %f at %d\n", cur_val, (int)(secCnt/10));
			}
		}

		// if timer is running, condition resolved before timeout
		if(TMR_isTimerEnabled(OVERLOAD_WARN_END_TSIG))
		{
			if(OVL_isOverloadWarnCondition(cur_val)) // return to warn condition
			{
				TMR_disableTimerSig(OVERLOAD_WARN_END_TSIG);
				UARTprintf("OVL warning returned %f\n", cur_val);
			}
		}

		// timeout -> change WARNING state
		if(TMR_isTimeout(OVERLOAD_WARN_END_TSIG))
		{
			TMR_disableTimerSig(OVERLOAD_WARN_END_TSIG);
			ovl_state = OVL_WARN_NOMAL;

			if(ovl_alarm_enable)
			{
				UARTprintf("OVL ALARM cleared at %d\n", (int)(secCnt/10));
				ovl_alarm_enable = 0; //TODO : clear alarm
			}
		}
		break;
	}

	return 0;
}


int OVL_isOverloadTripCondition(float_t cur_level)
{
	return (cur_level > dev_const.trip_level);
}

int OVL_processOverloadTrip(float_t cur_val)
{
	//uint32_t st_time;

	if(!OVL_isOverloadTripEnabled()) return 0;

	// start condition ?
	if(OVL_isOverloadTripCondition(cur_val))
	{
		if(!TMR_isTimerEnabled(OVERLOAD_TRIP_TSIG))
		{
			//start timer
			TMR_startTimerSig(OVERLOAD_TRIP_TSIG, (float_t)iparam[OVL_TR_DURATION_INDEX].value.l);
			UARTprintf("OVL trip started %f at %d\n", cur_val, (int)(secCnt/10));
		}
	}

	// condition resolved before timeout
	if(TMR_isTimerEnabled(OVERLOAD_TRIP_TSIG))
	{
		if(!OVL_isOverloadTripCondition(cur_val)) // condition resolved
		{
			TMR_disableTimerSig(OVERLOAD_TRIP_TSIG);
			UARTprintf("OVL trip resolved %f\n", cur_val);
		}
	}

	// timeout ?
	if(TMR_isTimeout(OVERLOAD_TRIP_TSIG))
	{
		TMR_disableTimerSig(OVERLOAD_TRIP_TSIG);
		return 1;
	}

	return 0;
}

int OVL_isOverCurrentCondition(float_t cur_level)
{
	return (cur_level > dev_const.ovc_level);
}

int OVL_processOverCurrentTrip(float_t cur_val)
{
	//uint32_t st_time;

	// start condition ?
	if(OVL_isOverCurrentCondition(cur_val))
	{
		if(!TMR_isTimerEnabled(OVERLOAD_OVC_TSIG))
		{
			//start timer
			TMR_startTimerSig(OVERLOAD_OVC_TSIG, OVER_CURRENT_TIMEOUT); // 2 sec
			UARTprintf("OV current trip started %f at %d\n", cur_val, (int)(secCnt/10));
		}
	}

	// condition resolved before timeout
	if(TMR_isTimerEnabled(OVERLOAD_OVC_TSIG))
	{
		if(!OVL_isOverCurrentCondition(cur_val)) // condition resolved
		{
			TMR_disableTimerSig(OVERLOAD_OVC_TSIG);
			UARTprintf("OV current trip resolved %f\n", cur_val);
		}
	}

	// timeout ?
	if(TMR_isTimeout(OVERLOAD_OVC_TSIG))
	{
		TMR_disableTimerSig(OVERLOAD_OVC_TSIG);
		return 1;
	}

	return 0;
}

#if 0
// valid resistance 150 ~ 500 ohm
int REGEN_setRegenResistance(float_t resist)
{
	if(resist < REGEN_RESISTANCE_VALUE_MIN || resist > REGEN_RESISTANCE_VALUE_MAX) return 1;

	iparam[REGEN_RESISTANCE_INDEX].value.f = resist;

	MAIN_setRegenDuty(resist, iparam[REGEN_POWER_INDEX].value.l);

	return 0;
}

int REGEN_setRegenResistancePower(uint16_t power)
{
	if(power < 10) return 1;

	iparam[REGEN_POWER_INDEX].value.l = (uint32_t)power;

	MAIN_setRegenDuty(iparam[REGEN_RESISTANCE_INDEX].value.f, power);

	return 0;
}

int REGEN_setRegenThermal(float_t value)
{
	iparam[REGEN_THERMAL_INDEX].value.f = value;

	return 0;
}
#endif

int REGEN_setRegenBand(uint32_t value)
{
	if(value > 150) return 1;

	iparam[REGEN_BAND_INDEX].value.l = value;

	protect_dc.dc_volt_end_regen_level = DC_VOLTAGE_START_REGEN_LEVEL_380V - (float_t)value;

	return 0;
}

int REGEN_getDuty(void)
{
	return regen_duty;
}

#if 1
int REGEN_setRegenDuty(uint32_t value)
{
	if(value > 100) return 1;

	iparam[REGEN_DUTY_INDEX].value.l = value;

	return 0;
}
#else
int REGEN_setRegenDuty(float_t dc_value)
{
	//int i;
	float_t exp_duty; //exp_I, exp_W

	exp_duty = dev_const.regen_max_V/dc_value;
	return (int)(exp_duty*100.0); // rounded integer
}
#endif

int REGEN_isEnabled(void)
{
	return internal_status.regen_enabled;
}

void REGEN_active(float_t dc_value)
{
	//int regen_duty;

	internal_status.regen_enabled = 1;

	//regen_duty = REGEN_setRegenDuty(dc_value);
	//regen_duty = 70; 	// use fixed value
	UTIL_setRegenPwmDuty(regen_duty);
}

void REGEN_end(void)
{
	internal_status.regen_enabled = 0;
	//regen_duty = 0;

	UTIL_setRegenPwmDuty(0);
}

int REGEN_process(float_t dc_volt)
{
	static int under_flag=0, over_flag=0, regen_flag=0; //, off_flag=0;

	if(!MAIN_isInitRelayInitialized())
	{
		if(dc_volt > protect_dc.dc_volt_init_relay_on)
		{
			UTIL_setInitRelay();
			//UTIL_setFanOn();
//			UARTprintf("Relay on at Vdc %f\n", dc_volt);
			under_flag=0; //initialize
			//off_flag=0; //initialize
		}
	}
	else
	{
		if(dc_volt < protect_dc.dc_volt_init_relay_on - 20.0) //&& dc_volt > protect_dc.dc_volt_init_relay_off)
		{
			ud_volt_count++;
			if(ud_volt_count > 50)
			{
				// DC under voltage trip
				if(under_flag==0)
				{
					//UTIL_setFanOff();
					UARTprintf("DC under voltage %f trip event happened at %d\n", dc_volt, (int)(secCnt/10));
					under_flag=1;
				}
				ERR_setTripInfo();
				trip_info.Vdc_inst = dc_volt;
				ERR_setTripFlag(TRIP_REASON_VDC_UNDER);
				return 1; // disable PWM after return
			}
		}
		else
			ud_volt_count=0;

#if 0 // remove relay off
		if(dc_volt < protect_dc.dc_volt_init_relay_off) // force to disable relay
		{
			if(off_flag == 0)
			{
				UTIL_clearInitRelay();
				UARTprintf("Relay off at Vdc %f\n", dc_volt);
				off_flag=1;
				under_flag=0; //initialize
			}
		}
#endif
	}

	if(dc_volt > protect_dc.dc_volt_over_trip_level)
	{
		ov_volt_count++;
		if(ov_volt_count > 50)
		{
			// DC over voltage trip
			if(over_flag==0)
			{
				UARTprintf("DC over voltage %fV trip event happened at %d\n", dc_volt, (int)(secCnt/10));
				over_flag=1;
			}
			ERR_setTripInfo();
			trip_info.Vdc_inst = dc_volt;
			ERR_setTripFlag(TRIP_REASON_VDC_OVER);
			return 1; // disable PWM after return
		}
	}
	else
		ov_volt_count=0;

	if(dc_volt > protect_dc.dc_volt_start_regen_level) //dev_const.regen_limit + param.protect.regen.band)
	{
		REGEN_active(dc_volt);
		if(regen_flag==0)
		{
			UARTprintf("REGEN_start() DC=%f, duty=%d \n", dc_volt, regen_duty);
			regen_flag++;
		}
	}


	if(REGEN_isEnabled())
	{
		if(dc_volt < protect_dc.dc_volt_end_regen_level) //dev_const.regen_limit)
		{
			UARTprintf("REGEN_end() DC=%f, duty=%d \n", dc_volt, regen_duty);
			REGEN_end();
			regen_flag=0;
		}
//		else
//			REGEN_active(dc_volt);
	}

	return 0;
}

//bool ovTempWarn = 0;
//bool ovTempFault = 0;
////bool ipmFault = 0;
//
//int OSC_setDampImpact(int value)
//{
//	param.osc_damp.impact = value;
//
//	//return EEP_updateItem(OSC_DAMP_IMPACT_ADDR, (unsigned char *)&param.osc_damp.impact);
//	return 0;
//}
//
//int OSC_setDampFilter(int value)
//{
//	param.osc_damp.filter = value;
//
//	//return EEP_updateItem(OSC_DAMP_FILTER_ADDR, (unsigned char *)&param.osc_damp.filter);
//	return 0;
//}

int TEMP_isFanOn(float_t ipm_temp)
{
	if(internal_status.fan_enabled == 1) return 0; // already on

	if(ipm_temp > IPM_TEMPERATURE_FAN_ON) return 1;
	else 	return 0;

}

int TEMP_isFanOff(float_t ipm_temp)
{
	if(internal_status.fan_enabled == 0) return 0; // already off

	if(ipm_temp > IPM_TEMPERATURE_FAN_ON) return 0;
	else 	return 1;

}

uint16_t ipm_count=0, mtr_count=0;
int TEMP_monitorTemperature(void)
{
	float_t ipm_temp, mtr_temp;
	static int ipm_status=0, mtr_status=0;

	ipm_temp = UTIL_readIpmTemperature();

	if(iparam[FAN_COMMAND_INDEX].value.l == 1) // conditional FAN control
	{
		if(TEMP_isFanOn(ipm_temp)) UTIL_setFanOn();

		if(TEMP_isFanOff(ipm_temp)) UTIL_setFanOff();
	}

	if(ipm_temp > IPM_TEMPERATURE_LIMIT)
	{
		ipm_count++;
		if(ipm_count > 10)
		{
			ERR_setTripInfo();
			trip_info.ipm_temp = ipm_temp;
			ERR_setTripFlag(TRIP_REASON_IPM_OVER_TEMP);
			if(ipm_status == 0)
			{
				UARTprintf("IPM over temperature %f\n", ipm_temp);
				ipm_status = 1;
			}
		}
	}
	else
		ipm_count=0;

	mtr_temp = UTIL_readMotorTemperature();
	if(mtr_temp > MOTOR_TEMPERATURE_LIMIT)
	{
		ERR_setTripInfo();
		trip_info.motor_temp = mtr_temp;
		ERR_setTripFlag(TRIP_REASON_MTR_OVER_TEMP);
		if(mtr_status == 0)
		{
			UARTprintf("Motor over temperature %f\n", mtr_temp);
			mtr_status = 1;
		}
	}

	return 0;
}

void PROT_init(int input)
{
	if(input == 220)
	{
		//protect_dc.dc_volt_init_relay_off = DC_VOLTAGE_INIT_RELAY_OFF_220V;
		protect_dc.dc_volt_init_relay_on = DC_VOLTAGE_INIT_RELAY_ON_220V;
		protect_dc.dc_volt_over_trip_level = DC_VOLTAGE_OVER_TRIP_LEVEL_220V;

		protect_dc.dc_volt_start_regen_level = DC_VOLTAGE_START_REGEN_LEVEL_220V;
		protect_dc.dc_volt_end_regen_level = DC_VOLTAGE_END_REGEN_LEVEL_220V;
	}
	else if(input == 380)
	{
		//protect_dc.dc_volt_init_relay_off = DC_VOLTAGE_INIT_RELAY_OFF_380V;
		protect_dc.dc_volt_init_relay_on = DC_VOLTAGE_INIT_RELAY_ON_380V;
		protect_dc.dc_volt_over_trip_level = DC_VOLTAGE_OVER_TRIP_LEVEL_380V;

		protect_dc.dc_volt_start_regen_level = DC_VOLTAGE_START_REGEN_LEVEL_380V;
		protect_dc.dc_volt_end_regen_level = DC_VOLTAGE_START_REGEN_LEVEL_380V - (float_t)iparam[REGEN_BAND_INDEX].value.l;
	}
	else
	{
		ERR_setTripInfo();
		trip_info.V_input = input;
		ERR_setTripFlag(TRIP_REASON_INPUT_VOLT_ERR);
	}

}

int processProtection(void)
{
#if 1
	float_t I_rms;

	// one shot
	if(MAIN_isSystemEnabled()
#ifdef SUPPORT_OFFSET_MEASURE
		&& !gOffsetMeasureFlag
#endif
		)
	{
		I_rms = MAIN_getIave();

		OVL_processOverloadWarning(I_rms);

		if(OVL_processOverloadTrip(I_rms))
		{
			// overload trip enabled
			// trip signal generate
			ERR_setTripInfo();
			trip_info.Irms = I_rms;
			ERR_setTripFlag(TRIP_REASON_OVERLOAD);
			// set trip error code
			// disable PWM output
//			MAIN_disableSystem();
//
			evt_flag++;
			if(evt_flag == 1)
				UARTprintf("OVL trip event happened at %d\n", (int)(secCnt/10));
		}

		if(OVL_processOverCurrentTrip(I_rms))
		{
			ERR_setTripInfo();
			trip_info.Irms = I_rms;
			ERR_setTripFlag(TRIP_REASON_OVER_CURRENT);
//			MAIN_disableSystem();
			evt_flag++;
			if(evt_flag == 1)
				UARTprintf("OV Current trip event happened at %d\n", (int)(secCnt/10));
		}
	}
#endif

	if( REGEN_process(MAIN_getVdcBus()) ) MAIN_disableSystem();

	return 0;
}

