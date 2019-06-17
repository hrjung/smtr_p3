/*
 * timer_handler.c
 *
 *  Created on: 2017. 5. 16.
 *      Author: hrjung
 */

#include <stdint.h>

#include "hal.h"

#include "uartstdio.h"

#include "inv_param.h"
#include "parameters.h"
#include "drive.h"
#include "state_func.h"
#include "timer_handler.h"
#include "err_trip.h"

#ifdef FLASH
#pragma CODE_SECTION(timer0ISR,"ramfuncs");
#endif

/*******************************************************************************
 * MACROS
 */
//#define SUPPORT_OFFSET_MEASURE

/*******************************************************************************
 * CONSTANTS
 */
#define OFFSET_COUNT_MAX		1000

/*******************************************************************************
 * TYPEDEFS
 */

typedef struct
{
	int 		enable;
	uint32_t	st_time;
	uint32_t	duration;
	int			timeout_flag;
} timer_handler_st;

/*******************************************************************************
 * LOCAL VARIABLES
 */

uint32_t on_time=0;
uint32_t run_time=0;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */

uint32_t gTimerCount = 0;
uint32_t secCnt = 0;

timer_handler_st time_sig[MAX_TIMER_TSIG];
/*******************************************************************************
 * EXTERNS
 */
extern HAL_Handle halHandle;

uint16_t gFlag_LogEnabled=0;
uint32_t dbg_flag=0;
extern void dbg_showMonitorParam(void);

extern void printLog(void);

uint32_t temp_flag=0;
extern int TEMP_monitorTemperature(void);

uint16_t wd_count=0;

#ifdef SUPPORT_MISS_PHASE_DETECT
uint16_t miss_in_phase_cnt=0;
#endif

#ifdef SUPPORT_OFFSET_MEASURE_

enum
{
	OFS_START,
	OFS_ON_GOING,
	OFS_STOP
};

extern uint16_t gOffsetMeasureFlag;
extern HAL_AdcData_t gAdcData;
uint16_t ofs_idx=0, ofs_done=0;
_iq I_sf=0.0, V_sf=0.0;
float_t ofs_total[6], ofs_value[6];
uint16_t ofs_state = OFS_STOP;
uint16_t vf_sel_bk;

extern void dbg_enableSystem(void);
extern void dbg_disableSystem(void);
#endif

extern int FREQ_setFreqValue(float_t value);
/*
 *  ======== local function ========
 */
#ifdef SUPPORT_OFFSET_MEASURE_
int OFS_isOffsetDone(void)
{
	return (ofs_done == 1);
}

void OFS_initOffsetState(void)
{
	ofs_state = OFS_START;
}

int OFS_readyOffset(void)
{
	return (ofs_state == OFS_STOP);
}

int OFS_getOffsetState(void)
{
	return ofs_state;
}

void OFS_startOffset(void)
{
	gOffsetMeasureFlag=1;
}

void OFS_endOffset(void)
{
	gOffsetMeasureFlag=0;
}

void OFS_initOffset(void)
{
	int i;

	ofs_idx = 0;
	ofs_done = 0;
	I_sf = HAL_getCurrentScaleFactor(halHandle);
	V_sf = HAL_getVoltageScaleFactor(halHandle);

	for(i=0; i<5; i++)
	{
		ofs_total[i] = 0.0;
		ofs_value[i] = 0.0;
	}
}

void OFS_state(void)
{
	int i;
	_iq value;

	switch(ofs_state)
	{
	case OFS_START:
		OFS_initOffset();
		vf_sel_bk = (int)iparam[VF_FOC_SEL_INDEX].value.l; // backup
		DRV_enableVfControl(); // set VF control
		if(MAIN_isSystemEnabled())
			dbg_enableSystem();

		ofs_state = OFS_ON_GOING;
		break;

	case OFS_ON_GOING:
		if(ofs_idx < OFFSET_COUNT_MAX)
		{
			for(i=0; i<3; i++)
			{
				value = (_iq)(gAdcData.v_adc[i]);
				value = _IQ12mpy(value,V_sf);
				ofs_total[i] += _IQtoF(value);
			}

			for(i=0; i<2; i++)
			{
				value = (_iq)(gAdcData.i_adc[i]);
				value = _IQ12mpy(value,I_sf);
				ofs_total[i+3] += _IQtoF(value);
			}

			ofs_idx++;
		}
		else
		{
			ofs_state = OFS_STOP;
		}
		break;

	case OFS_STOP:
		ofs_done=1;
		dbg_disableSystem();
		OFS_endOffset(); // clear flag
		iparam[VF_FOC_SEL_INDEX].value.l = (long)vf_sel_bk; //restore
		UARTprintf("offset done!");
		break;
	}
}
#endif

/*
 *  ======== public function ========
 */

void TMR_clearRunTime(void)
{
	run_time = 0;
}

void TMR_clearOnTime(void)
{
	on_time = 0;
}

uint32_t TMR_getRunTime(void)
{
	return run_time;
}

uint32_t TMR_getOnTime(void)
{
	return on_time;
}

int TMR_isTimerEnabled(int type)
{
	return time_sig[type].enable;
}

void TMR_disableTimerSig(int type)
{
	time_sig[type].enable = 0;
	time_sig[type].duration = 0;
	time_sig[type].st_time = 0;
	time_sig[type].timeout_flag = 0;
}

// unit of duration is 100ms
uint32_t TMR_startTimerSig(int type, float_t f_duration)
{
	if(TMR_isTimerEnabled(type)) return 0;

	time_sig[type].enable = 1;
	time_sig[type].duration = (uint32_t)(f_duration*10.0); // convert to 100ms unit
	time_sig[type].st_time = secCnt;
	time_sig[type].timeout_flag = 0;

	return secCnt;
}

void TMR_init(void)
{
	int i;

	on_time = 0;
	run_time = 0;

	gTimerCount=0;
	secCnt=0;
	for(i=0; i<MAX_TIMER_TSIG; i++)
	{
		TMR_disableTimerSig(i);
	}
}

int TMR_isTimeout(int type)
{
	return (time_sig[type].enable && time_sig[type].timeout_flag);
}

inline int TMR_isTimeOutCondition(int type)
{
	return (time_sig[type].duration <=  secCnt - time_sig[type].st_time) && (time_sig[type].timeout_flag == 0);
}

//hrjung add for timer0 interrupt 1ms period
//uint16_t adc_data;
//extern int ADC_readCurrentControl(uint16_t *value);
interrupt void timer0ISR(void)
{

	//UTIL_testbit(1);
	// toggle status LED
	gTimerCount++;
	if(gTimerCount%100 == 0) secCnt++; // 100ms

#if 1
	if(MAIN_isTripHappened())
	{
		if(gTimerCount%1000 == 0)
		{
#ifdef SUPPORT_HW_COMMON
			HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED_R);
#else
			HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED3);
#endif
		}
	}
#endif

	// acknowledge the Timer 0 interrupt
	HAL_acqTimer0Int(halHandle);

#ifdef SUPPORT_WATCHDOG
	if(gTimerCount%100 == 0) // 100ms kick
		HAL_kickWdog(halHandle);
#endif

#if 0 // only for test without debug connection
	if(internal_status.relay_enabled)
	{
		static uint32_t test_start=0, test_duration=0;
		static int freq_set=0, test_end=0;

		// set start time
		if(test_start == 0) test_start = secCnt;

		// wait 3 sec
		if(secCnt - test_start > 30)
		{
			if(freq_set == 0)
			{
				//set frequency
				FREQ_setFreqValue(40.0);
				// start
				MAIN_enableSystem();
				STA_calcResolution();
				freq_set = 1;
				test_duration = secCnt;
				UARTprintf("set freq 40Hz at %f\n", (float_t)(secCnt/10.0));
			}

#if 0 // no stop for impulse test
			// run 10s including acceleration
			if(freq_set == 1 && (secCnt - test_duration > 100))
			{
				// stop motor
				if(test_end == 0)
				{
					STA_setStopCondition();
					test_end = 1;
					UARTprintf("stop at %f\n", (float_t)(secCnt/10.0));
				}
			}
#endif
		}
	}
#endif


#ifdef SUPPORT_OFFSET_MEASURE_
	if(gOffsetMeasureFlag)
	{
		OFS_state();
	}

#endif

	if(gFlag_LogEnabled) // print log at every 1 sec
	{

		if(secCnt%10 == 0)
		{
			if(dbg_flag == 0) printLog();

			dbg_flag++;
		}
		else
		{
			dbg_flag=0;
		}
	}

	if(secCnt%5 == 0) //every 0.5 sec
	{
		if(temp_flag == 0)
			TEMP_monitorTemperature(); // IPM, Motor temperature check

		temp_flag++;
	}
	else
		temp_flag=0;

#ifdef SUPPORT_MISS_PHASE_DETECT
	if(MAIN_isSystemEnabled())
	{
		float_t dc_value = MAIN_getVdcBus();
		if(gTimerCount%100 == 0 && dc_value < 480.0) // every 100ms
		{
			miss_in_phase_cnt++;
			if(miss_in_phase_cnt > 3)
			{
				// raise trip
				ERR_setTripInfo();
				trip_info.Vdc_inst = dc_value;
				ERR_setTripFlag(TRIP_REASON_V_PHASE_MISS);
			}
		}
		else
			miss_in_phase_cnt=0;
	}
#endif

	if(time_sig[DCI_BRAKE_SIG_ON_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(DCI_BRAKE_SIG_ON_TSIG))
		{
			time_sig[DCI_BRAKE_SIG_ON_TSIG].timeout_flag = 1;
			//UARTprintf(" DCI_BRAKE_SIG_ON timeout at %d \n", (int)secCnt);

		}
	}

	if(time_sig[DCI_BRAKE_SIG_OFF_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(DCI_BRAKE_SIG_OFF_TSIG))
		{
			time_sig[DCI_BRAKE_SIG_OFF_TSIG].timeout_flag = 1;
			//UARTprintf(" DCI_BRAKE_SIG_OFF timeout at %d \n", (int)secCnt);

		}
	}

	if(time_sig[OVERLOAD_WARN_START_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(OVERLOAD_WARN_START_TSIG))
		{
			time_sig[OVERLOAD_WARN_START_TSIG].timeout_flag = 1;
		}
	}

	if(time_sig[OVERLOAD_WARN_END_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(OVERLOAD_WARN_END_TSIG))
		{
			time_sig[OVERLOAD_WARN_END_TSIG].timeout_flag = 1;
		}
	}

	if(time_sig[OVERLOAD_TRIP_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(OVERLOAD_TRIP_TSIG))
		{
			time_sig[OVERLOAD_TRIP_TSIG].timeout_flag = 1;
//			MAIN_disableSystem();
		}
	}

	if(time_sig[OVERLOAD_OVC_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(OVERLOAD_OVC_TSIG))
		{
			time_sig[OVERLOAD_OVC_TSIG].timeout_flag = 1;
//			MAIN_disableSystem();
		}
	}

	if(time_sig[TIMER_TEST_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(TIMER_TEST_TSIG))
		{
			time_sig[TIMER_TEST_TSIG].timeout_flag = 1;
			UARTprintf(" Test Timer timeout at %d \n", (int)secCnt);
			TMR_disableTimerSig(TIMER_TEST_TSIG);
		}
	}

	return;
} // end of timer0ISR() function


