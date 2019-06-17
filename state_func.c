/*
 * ctrl_func.c
 *
 *  Created on: 2017. 3. 27.
 *      Author: hrjung
 */


#include "uartstdio.h"

#include "user.h"
#include "parameters.h"
//#include "inv_param.h"
#include "state_func.h"
#include "drive.h"
#include "freq.h"
#include "err_trip.h"

#ifdef FLASH
#pragma CODE_SECTION(STA_setStopStatus,"ramfuncs");
#pragma CODE_SECTION(STA_calcResolution,"ramfuncs");
#pragma CODE_SECTION(STA_setResolution,"ramfuncs");
#pragma CODE_SECTION(STA_setNextFreq,"ramfuncs");
#pragma CODE_SECTION(STA_getCurFreq,"ramfuncs");
#endif

/*******************************************************************************
 * MACROS
 */
#if 1
#define STA_isAccelStateCond()  (fabsf(m_status.cur_freq) < fabsf(m_status.target_freq))
#define STA_isDecelStateCond()  (fabsf(m_status.cur_freq) > fabsf(m_status.target_freq))
#define STA_isStopStateCond()   (m_status.cur_freq == m_status.target_freq && m_status.cur_freq == 0.0)
//#define STA_isRunStateCond()    (m_status.cur_freq == m_status.target_freq && m_status.cur_freq != 0)
#else
#define STA_isAccelStateCond()  (m_status.cur_rpm < m_status.target_rpm)
#define STA_isDecelStateCond() 	(m_status.cur_rpm > m_status.target_rpm)
#define STA_isStopStateCond()  	(m_status.cur_rpm == m_status.target_rpm && m_status.cur_rpm == 0)
#define STA_isRunStateCond()	(m_status.cur_rpm == m_status.target_rpm && m_status.cur_rpm != 0)
#endif
/*******************************************************************************
 * CONSTANTS
 */
#define TIMER_SCALE_FACTOR	(100)

#define FREQ_DELTA			(0.0001)
/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * LOCAL VARIABLES
 */

MOTOR_working_st m_status;

uint16_t dir_changed = 0;
float_t dir_freq = 0.0;
uint16_t dir_set_freq = 0;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */
STATIC mtr_state_e func_start(void);
STATIC mtr_state_e func_stop(void);
STATIC mtr_state_e func_accel(void);
STATIC mtr_state_e func_decel(void);
STATIC mtr_state_e func_run(void);


mtr_state_e (*ctrl_func[])(void) = {
		func_start,
		func_stop,
		func_accel,
		func_decel,
		func_run
};

/*******************************************************************************
 * GLOBAL VARIABLES
 */
extern uint32_t secCnt;
/*******************************************************************************
 * EXTERNS
 */
//extern void initParam(void);
extern int MAIN_isDirectionReversed(void);

/*
 *  ======== local function ========
 */
/*
 * 	1. init variables
 * 	2. go to stop state, if run at start option disabled
 *
 */

STATIC int STA_isRunStateCond(void)
{
	if(m_status.status == STATE_ACCEL)
	{
		if(m_status.target_freq != 0.0)
		{
			if(( (fabsf(m_status.target_freq) - fabsf(m_status.cur_freq)) < FREQ_DELTA) && m_status.cur_freq != 0.0) return 1;
		}
	}
	else if(m_status.status == STATE_DECEL)
	{
		if(m_status.target_freq != 0.0)
		{
			if(( (fabsf(m_status.cur_freq) - fabsf(m_status.target_freq)) < FREQ_DELTA) && m_status.cur_freq != 0.0) return 1;
		}
	}
	else if(m_status.status == STATE_RUN) return 1;

	return 0;
}

#ifdef SUPPORT_DIRECTION_STATUS
int STA_isDirectionDone(void)
{
	return (dir_changed == 1 && dir_set_freq == 1);
}

//int STA_isDirChanged(void)
//{
//	return (dir_changed == 1);
//}

//void STA_setDirChanged(void)
//{
//	dir_changed = 1;
//}

//int STA_isDirFreqSet(void)
//{
//	return (dir_set_freq == 1);
//}

//void STA_setDirFreqSet(void)
//{
//	dir_set_freq = 1;
//}

void STA_initDirectionFlag(void)
{
	dir_changed = 0;
	dir_set_freq = 0;
	dir_freq = 0.0;
}
#endif


STATIC mtr_state_e func_start(void)
{

    m_status.cur_freq = 0.0;
    m_status.target_freq = 0.0;
	m_status.cur_rpm = 0.0;
	m_status.target_rpm = 0.0;

	m_status.direction = STOP;
//	m_status.prev_acc_time = 0;
//	m_status.prev_dec_time = 0;
	m_status.acc_res = 0.0;
	m_status.dec_res = 0.0;

	m_status.current = 0;
	m_status.status = STATE_START;

//	DRV_setAccelTime(param.ctrl.accel_time);
//	DRV_setDecelTime(param.ctrl.decel_time);

//	STA_setNextSpeed(param.ctrl.value[0]);

	STA_initDirectionFlag();

	m_status.status = STATE_STOP;
	return STATE_STOP;
}

// speed = 0
STATIC mtr_state_e func_stop(void)
{
	static int first=1;
	mtr_state_e state = STATE_STOP;

	if(first == 1)
	{
#ifdef SUPPORT_DIRECTION_STATUS
	  if(STA_isDirChanged())
	  {
		  if(!STA_isDirFreqSet())
		  {
			  FREQ_setFreqValue(dir_freq);
			  STA_setDirFreqSet();
		  }
	  }
	  else
#endif
	  {
		first=0;
		UARTprintf(" STOP %f\n", (float_t)(secCnt/10.0));

		m_status.direction = STOP;
		// STOP sig on

		// RUN sig off

		//TODO : temp testing before brake is ready
		MAIN_disableSystem();
#ifdef SUPPORT_DIRECTION_STATUS
		STA_initDirectionFlag();
#endif
	  }
	}


	if(STA_isAccelStateCond())
		state = STATE_ACCEL;

	if(state != STATE_STOP)
	{
		first=1;
		// STOP sig off

		// RUN sig on

	}

	m_status.status = state;
	return state;
}

// increase speed
STATIC mtr_state_e func_accel(void)
{
	static int first=1;
	mtr_state_e state = STATE_ACCEL;

	if(first == 1)
	{
		first=0;
		UARTprintf(" ACCEL %f\n", (float_t)(secCnt/10.0));
	}

	if(STA_isRunStateCond())
		state = STATE_RUN;
	else if(STA_isDecelStateCond())
		state = STATE_DECEL;

	if(state != STATE_ACCEL) first=1;

	//UARTprintf(" STATE_ACCEL state=%d cur_freq=%f target_freq=%f\n", (int)state, m_status.cur_freq, m_status.target_freq);

	m_status.status = state;
	return state;
}

// decrease speed
STATIC mtr_state_e func_decel(void)
{
	static int first=1;
	mtr_state_e state = STATE_DECEL;

	if(first == 1)
	{
		first=0;
		UARTprintf(" DECEL %f\n", (float_t)(secCnt/10.0));
	}

	if(STA_isRunStateCond())
		state = STATE_RUN;
	else if(STA_isStopStateCond())
		state = STATE_STOP;
	else if(STA_isAccelStateCond())
		state = STATE_ACCEL;

	if(state != STATE_DECEL) first=1;

	m_status.status = state;
	return state;
}

// steady speed
STATIC mtr_state_e func_run(void)
{
	static int first=1;
	mtr_state_e state = STATE_RUN;
	//static int prev_target_rpm=0;
	static float_t prev_target_freq=0.0;

	if(first == 1)
	{
		first=0;
		UARTprintf(" RUN freq=%f at %f\n", m_status.cur_freq, (float_t)(secCnt/10.0));

		// set RUN_SIG
#ifdef SUPPORT_DIRECTION_STATUS
		if(STA_isDirectionDone())
		{
			STA_initDirectionFlag();
		}
#endif
	}

	//if(prev_target_rpm != m_status.target_rpm)
	if(prev_target_freq != m_status.target_freq)
	{
		//prev_target_rpm = m_status.target_rpm;
		prev_target_freq = m_status.target_freq;
		if(STA_isAccelStateCond())
			state = STATE_ACCEL;
		else if(STA_isDecelStateCond())
			state = STATE_DECEL;
	}
//	else
//	{
//		// change direction
//		if(MAIN_isDirectionReversed())
//		{
//			state = STATE_DECEL;
//		}
//	}

	if(state != STATE_RUN)
	{
		first=1;
		// clear RUN_SIG
	}

	m_status.status = state;
	return state;
}

/*
 *  ======== public function ========
 */

//float STA_getCurrentSpeed(void)
//{
//	return m_status.cur_rpm;
//}

//int STA_getState(void)
//{
//    return m_status.status;
//}
//
//int STA_isStopState(void)
//{
//	return (m_status.status == STATE_STOP);
//}
//
//int STA_isRunState(void)
//{
//	return (m_status.status == STATE_RUN);
//}
//
//int STA_isAccelState(void)
//{
//	return (m_status.status == STATE_ACCEL);
//}
//
//int STA_isDecelState(void)
//{
//	return (m_status.status == STATE_DECEL);
//}

void STA_setStopStatus(void)
{
    m_status.cur_freq = 0.0;
    m_status.target_freq = 0.0;
	m_status.cur_rpm = 0.0;
	m_status.target_rpm = 0.0;

	m_status.direction = STOP;
	m_status.acc_res = 0.0;
	m_status.dec_res = 0.0;

	m_status.current = 0;
	m_status.status = STATE_STOP;
}

#if 0
int STA_getSpeedRange(void)
{
	return mtr.rpm_max - mtr.rpm_min;
}

float STA_getTargetSpeed_krpm(void)
{
	return (float)(m_status.target_rpm/1000.0);
}

void STA_setNextSpeed(int value)
{
	float ref_rpm = (float)DRV_getActualSpeed(value)*dev_param.gear_ratio;

	m_status.target_rpm = (int)(ref_rpm + 0.5);
}
#else
void STA_setCurSpeed(float_t cur_spd)
{
	if(cur_spd < 0)
	{
		cur_spd *= (-1);
		m_status.direction = REVERSE;
	}
	else
		m_status.direction = FORWARD;

	m_status.cur_rpm = cur_spd*1000.0;
	m_status.cur_freq = MAIN_convert2Freq(m_status.cur_rpm); //m_status.cur_rpm is dummy

	if(m_status.cur_freq > MAX_FREQ_VALUE)
	{
		ERR_setTripInfo();
		trip_info.over_freq = m_status.cur_freq;
		ERR_setTripFlag(TRIP_REASON_FREQ_RANGE_ERR);
	}
}

float_t STA_getCurSpeed(void)
{
	//return m_status.cur_rpm/param.gear_ratio;
	return m_status.cur_rpm;
}

//float_t STA_getTargetFreq(void)
//{
//	return m_status.target_freq;
//}

float_t STA_getCurFreq(void)
{
	return m_status.cur_freq;
}

void STA_setNextFreq(float_t value)
{
	// hrjung for direction test : direction is not controlled by sign of freq, only dir command
	// TODO : require condition for Analog input command
//	if(value >= 0.0)
//		MAIN_setForwardDirection();
//	else
//	{
//		value = -(1.0)*value;
//		MAIN_setReverseDirection();
//	}

	m_status.target_freq = FREQ_getVarifiedFreq(m_status.cur_freq, value);
}
#endif

void STA_setCurrent(float_t current)
{
	m_status.current = current;
}

float_t STA_getCurrent(void)
{
	return m_status.current;
}

float_t STA_getAccelResolution(void)
{
	return m_status.acc_res;
}

float_t STA_getDecelResolution(void)
{
	return m_status.dec_res;
}

void STA_printInvState(void)
{
	UARTprintf("speed curr_freq=%f, target_freq=%f\n", m_status.cur_freq, m_status.target_freq);
	//UARTprintf("speed curr_speed=%d, target_rpm=%d\n", m_status.cur_rpm, m_status.target_rpm);
	UARTprintf("resolution acc_res=%f, dec_res=%f\n", m_status.acc_res, m_status.dec_res);
}

float_t STA_getResolution(int flag)
{
	if(flag == ACCEL)
		return m_status.acc_res;
	else
		return m_status.dec_res;
}

void STA_setResolution(int flag, float_t value)
{
	if(value == 0.0) return;

	if(flag == ACCEL)
	{
		m_status.acc_res = value;
		internal_status.accel_resol = m_status.acc_res;
	}
	else
	{
		m_status.dec_res = value;
		internal_status.accel_resol = m_status.acc_res;
	}

	if(value > USER_MAX_ACCEL_Hzps)
	{
		ERR_setTripInfo();
		trip_info.over_accel = value;
		ERR_setTripFlag(TRIP_REASON_ACCEL_ERR);
	}
}

void STA_calcResolution(void)
{
	float_t time;
	int flag;
	float_t diff = m_status.target_freq - m_status.cur_freq;

	if(diff > 0.0)
	{
		flag = ACCEL;
		time = iparam[ACCEL_TIME_INDEX].value.f;
	}
	else if(diff < 0.0)
	{
		flag = DECEL;
		time = iparam[DECEL_TIME_INDEX].value.f;
		diff = (-1.0)*diff;
	}
	else
		return;

	STA_setResolution(flag, DRV_calculateAccelRate_krpm(time, diff));
}


void STA_calcResolution4Reverse(float_t run_freq)
{
	float_t time;
#ifdef SUPPORT_AUTO_LOAD_TEST
	float_t diff = run_freq;
#else
	float_t diff = m_status.cur_freq;
#endif

	time = iparam[ACCEL_TIME_INDEX].value.f;
	STA_setResolution(ACCEL, DRV_calculateAccelRate_krpm(time, diff));

	time = iparam[DECEL_TIME_INDEX].value.f;
	STA_setResolution(DECEL, DRV_calculateAccelRate_krpm(time, diff));
}

//int STA_isSameAccelRate(void)
//{
//	return (m_status.acc_res == m_status.dec_res);
//}


float_t STA_getTrajResolution(void)
{
	if(STA_isAccelStateCond())
		return m_status.acc_res;
	else if(STA_isDecelStateCond())
		return m_status.dec_res;
	else
		return 0.0;
}

void STA_setStopCondition(void)
{
#ifdef SUPPORT_DIRECTION_STATUS
	STA_initDirectionFlag();
#endif
	STA_setNextFreq(0.0);
	switch((int)iparam[BRK_TYPE_INDEX].value.l)
	{
	case REDUCE_SPEED_BRAKE:
	case DC_INJECT_BRAKE:
		STA_calcResolution();
		break;
	case FREE_RUN_BRAKE:
		MAIN_disableSystem();
		break;

	}
}

int STA_control(void)
{
	static mtr_state_e state = STATE_START;

	state = ctrl_func[state]();

	return (int)state;
}

