//###########################################################################
//
// FILE:   parameters.c
//
// TITLE:
//
//###########################################################################

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <Assert.h>

#include "main.h"
#include "inv_param.h"
#include "motor_param.h"
#include "parameters.h"
#include "common_tools.h"
#include "uartstdio.h"

#include "freq.h"
#include "drive.h"
#include "state_func.h"
#include "brake.h"
#include "protect.h"

//*****************************************************************************
//
//! \addtogroup
//! @{
//
//*****************************************************************************



int (*iparam_func[INV_PARAM_INDEX_MAX])(union32_st value) = {
		PARAM_setFreq, 			//FREQ_VALUE_INDEX
		PARAM_setMaxFreq,		//MAX_FREQ_INDEX
		PARAM_setAccel, 		//ACCEL_TIME_INDEX
		PARAM_setDecel, 		//DECEL_TIME_INDEX
		PARAM_setAccelTimeBase, //ACCEL_BASE_INDEX

		PARAM_setDirection, 	//DIRECTION_INDEX
//		PARAM_setVfFoc,			//VF_FOC_SEL_INDEX
		PARAM_setEnergySave,	//ENERGY_SAVE_INDEX
		PARAM_setPwmFreq,		//PWM_FREQ_INDEX

		PARAM_setEnableJump0,	//JUMP_ENABLE0_INDEX
		PARAM_setEnableJump1,	//JUMP_ENABLE1_INDEX
		PARAM_setEnableJump2,	//JUMP_ENABLE2_INDEX
		PARAM_setJumpFreqLow0,	//JUMP_LOW0_INDEX
		PARAM_setJumpFreqLow1,	//JUMP_LOW1_INDEX
		PARAM_setJumpFreqLow2,	//JUMP_LOW2_INDEX
		PARAM_setJumpFreqHigh0, //JUMP_HIGH0_INDEX
		PARAM_setJumpFreqHigh1, //JUMP_HIGH1_INDEX
		PARAM_setJumpFreqHigh2, //JUMP_HIGH2_INDEX
		//PARAM_setVoltageBoost,	//V_BOOST_INDEX
		//PARAM_setTorqueLimit,	//FOC_TORQUE_LIMIT_INDEX

		PARAM_setBrakeType, 	//BRK_TYPE_INDEX
		PARAM_setBrakeFreq, 	//BRK_FREQ_INDEX
		PARAM_setDciBrakeStartFreq,	//BRK_DCI_START_FREQ_INDEX
		PARAM_setDciBrakeBlockTime, //BRK_DCI_BLOCK_TIME_INDEX
		PARAM_setDciBrakeTime, 	//BRK_DCI_BRAKING_TIME_INDEX
		PARAM_setDciBrakeRate, 	//BRK_DCI_BRAKING_RATE_INDEX

		PARAM_setOvlWarnLevel, 	//OVL_WARN_LIMIT_INDEX
		PARAM_setOvlWarnTime, 	//OVL_WR_DURATION_INDEX
		PARAM_setOvlEnableTrip, //OVL_ENABLE_INDEX
		PARAM_setOvlTripLevel, 	//OVL_TR_LIMIT_INDEX
		PARAM_setOvlTripTime, 	//OVL_TR_DURATION_INDEX

//		PARAM_setRegenResistance,		//REGEN_RESISTANCE_INDEX
//		PARAM_setRegenResistThermal,	//REGEN_THERMAL_INDEX
//		PARAM_setRegenResistPower,		//REGEN_POWER_INDEX
		PARAM_setRegenDuty,				//REGEN_DUTY_INDEX
		PARAM_setRegenBand, 			//REGEN_BAND_INDEX

		PARAM_setFanControl,	//FAN_COMMAND_INDEX

};


inv_parameter_st iparam[INV_PARAM_INDEX_MAX];
inv_parameter_st err_info[ERR_CODE_MAX];
inv_parameter_st inv_status[INV_STATUS_MAX];

extern uint16_t ovl_alarm_enable;
extern USER_Params gUserParams;
extern MOTOR_working_st m_status;
extern motor_param_st mtr_param;
extern const char *res_str[2];

#ifdef SUPPORT_AUTO_LOAD_TEST
extern int load_test_type;
#endif

extern void STA_printInvState(void);
extern uint16_t MAIN_isRunState(void);
extern uint16_t MAIN_getDirection(void);
extern float_t MAIN_getIave(void);
extern float_t STA_getCurFreq(void);
extern float_t MAIN_getVdcBus(void);
extern float_t UTIL_readIpmTemperature(void);
//extern float_t UTIL_readMotorTemperature(void);
extern uint16_t UTIL_readMotorTemperatureStatus(void);
//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************

void PARAM_init(void)
{
#ifdef SUPPORT_AUTO_LOAD_TEST
	if(load_test_type == 0)
	{
		iparam[FREQ_VALUE_INDEX].type = PARAMETER_TYPE_FLOAT;
		iparam[FREQ_VALUE_INDEX].value.f = 50.0;
	}
	else
#endif
	{
	iparam[FREQ_VALUE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[FREQ_VALUE_INDEX].value.f = 20.0;
	}

	iparam[MAX_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[MAX_FREQ_INDEX].value.f = 60.0;

	iparam[ACCEL_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[ACCEL_TIME_INDEX].value.f = 10.0;

	iparam[DECEL_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[DECEL_TIME_INDEX].value.f = 10.0;

	iparam[ACCEL_BASE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[ACCEL_BASE_INDEX].value.l = 0;

	iparam[DIRECTION_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[DIRECTION_INDEX].value.l = 0;

//	iparam[VF_FOC_SEL_INDEX].type = PARAMETER_TYPE_LONG;
//	iparam[VF_FOC_SEL_INDEX].value.l = FOC_CONTROL; //VF_CONTROL;

	iparam[ENERGY_SAVE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[ENERGY_SAVE_INDEX].value.l = ESAVE_UNUSED; //  ESAVE_BOTH;

	iparam[PWM_FREQ_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[PWM_FREQ_INDEX].value.l = PWM_4KHz;

	iparam[JUMP_ENABLE0_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[JUMP_ENABLE0_INDEX].value.l = NOT_USED;

	iparam[JUMP_LOW0_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_LOW0_INDEX].value.f = 1.0;

	iparam[JUMP_HIGH0_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_HIGH0_INDEX].value.f = 1.0;

	iparam[JUMP_ENABLE1_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[JUMP_ENABLE1_INDEX].value.l = NOT_USED;

	iparam[JUMP_LOW1_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_LOW1_INDEX].value.f = 1.0;

	iparam[JUMP_HIGH1_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_HIGH1_INDEX].value.f = 1.0;

	iparam[JUMP_ENABLE2_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[JUMP_ENABLE2_INDEX].value.l = NOT_USED;

	iparam[JUMP_LOW2_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_LOW2_INDEX].value.f = 1.0;

	iparam[JUMP_HIGH2_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_HIGH2_INDEX].value.f = 1.0;

//	iparam[V_BOOST_INDEX].type = PARAMETER_TYPE_FLOAT;
//	iparam[V_BOOST_INDEX].value.f = 0.0;

//	iparam[FOC_TORQUE_LIMIT_INDEX].type = PARAMETER_TYPE_FLOAT;
//	iparam[FOC_TORQUE_LIMIT_INDEX].value.f = 180.0;

	iparam[BRK_TYPE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[BRK_TYPE_INDEX].value.l = REDUCE_SPEED_BRAKE;

	iparam[BRK_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_FREQ_INDEX].value.f = 1.0;

	iparam[BRK_DCI_START_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_START_FREQ_INDEX].value.f = 3.0;

	iparam[BRK_DCI_BLOCK_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_BLOCK_TIME_INDEX].value.f = 1.0;

	iparam[BRK_DCI_BRAKING_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_BRAKING_TIME_INDEX].value.f = 5.0;

	iparam[BRK_DCI_BRAKING_RATE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_BRAKING_RATE_INDEX].value.f = 50.0;

	iparam[OVL_WARN_LIMIT_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_WARN_LIMIT_INDEX].value.l = 150;

	iparam[OVL_WR_DURATION_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_WR_DURATION_INDEX].value.l = 10;

	iparam[OVL_ENABLE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_ENABLE_INDEX].value.l = 1; //use

	iparam[OVL_TR_LIMIT_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_TR_LIMIT_INDEX].value.l = 180;

	iparam[OVL_TR_DURATION_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_TR_DURATION_INDEX].value.l = 30;

//	iparam[REGEN_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
//	iparam[REGEN_RESISTANCE_INDEX].value.f = 200.0;
//
//	iparam[REGEN_THERMAL_INDEX].type = PARAMETER_TYPE_FLOAT;
//	iparam[REGEN_THERMAL_INDEX].value.f = 1000.0;
//
//	iparam[REGEN_POWER_INDEX].type = PARAMETER_TYPE_LONG;
//	iparam[REGEN_POWER_INDEX].value.l = 400;

	iparam[REGEN_DUTY_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[REGEN_DUTY_INDEX].value.l = 30;

	iparam[REGEN_BAND_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[REGEN_BAND_INDEX].value.l = 12;

	iparam[FAN_COMMAND_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[FAN_COMMAND_INDEX].value.l = 0;

#ifdef SUPPORT_MOTOR_PARAM
	iparam[STATOR_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[STATOR_RESISTANCE_INDEX].value.f = mtr_param.Rs;

	iparam[ROTATOR_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[ROTATOR_RESISTANCE_INDEX].value.f = mtr_param.Rr;

	iparam[INDUCTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[INDUCTANCE_INDEX].value.f = mtr_param.Ls;

	iparam[NOLOAD_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[NOLOAD_CURRENT_INDEX].value.f = mtr_param.noload_current;

	iparam[RATED_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[RATED_CURRENT_INDEX].value.f = mtr_param.max_current;

	iparam[POLES_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[POLES_INDEX].value.l = mtr_param.pole_pairs;

	iparam[INPUT_VOLTAGE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[INPUT_VOLTAGE_INDEX].value.l = mtr_param.voltage_in;

	iparam[RATED_FREQ_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[RATED_FREQ_INDEX].value.l = mtr_param.rated_freq;
#else
	iparam[STATOR_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[STATOR_RESISTANCE_INDEX].value.f = USER_MOTOR_Rs;

	iparam[ROTATOR_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[ROTATOR_RESISTANCE_INDEX].value.f = USER_MOTOR_Rr;

	iparam[INDUCTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[INDUCTANCE_INDEX].value.f = USER_MOTOR_Ls_d;

	iparam[NOLOAD_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[NOLOAD_CURRENT_INDEX].value.f = USER_MOTOR_NO_LOAD_CURRENT;

	iparam[RATED_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[RATED_CURRENT_INDEX].value.f = USER_MOTOR_MAX_CURRENT;

	iparam[POLES_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[POLES_INDEX].value.l = USER_MOTOR_NUM_POLE_PAIRS;

	iparam[INPUT_VOLTAGE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[INPUT_VOLTAGE_INDEX].value.l = USER_MOTOR_VOLTAGE_IN;

	iparam[RATED_FREQ_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[RATED_FREQ_INDEX].value.l = USER_MOTOR_RATED_FREQUENCY;
#endif

	iparam[INV_RUN_STOP_CMD_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[INV_RUN_STOP_CMD_INDEX].value.l = 0; // default stop

	// error parameter
	PARAM_initErrInfo();

	// inverter status
	PARAM_initInvStatus();

}


void PARAM_update(uint16_t index, uint16_t *buf)
{
	iparam[index].value.arr[0] = buf[0];
	iparam[index].value.arr[1] = buf[1];
}

uint16_t PARAM_getValue(uint16_t index, uint16_t *buf)
{
#if 0 // test only
	iparam[index].value.f = 3.14;
#endif
	buf[0] = iparam[index].value.arr[0];
	buf[1] = iparam[index].value.arr[1];

	return 2; // return size
}

int PARAM_setFreq(union32_st value)
{
	float_t freq = value.f;
	int result;

	result = FREQ_setFreqValue(freq);

	UARTprintf("set frequency=%f, result=%s\n", freq, res_str[result]);

	return result;
}

int PARAM_setMaxFreq(union32_st value)
{
	float_t freq = value.f;
	int result;

	result = FREQ_setMaxFreqValue(freq);

	UARTprintf("set max frequency=%f, result=%s\n", freq, res_str[result]);

	return result;
}

int PARAM_setAccel(union32_st value)
{
	float_t acc_rate = value.f;
	int result;

	result = DRV_setAccelTime(acc_rate);
	UARTprintf("set accel time=%f, result=%s\n", acc_rate, res_str[result]);

	return result;
}

int PARAM_setDecel(union32_st value)
{
	float_t dec_rate = value.f;
	int result;

	result = DRV_setDecelTime(dec_rate);
	UARTprintf("set decel time=%f, result=%s\n", dec_rate, res_str[result]);

	return result;
}

#ifdef SUPPORT_ACCEL_TIME_BASE
int PARAM_setAccelTimeBase(union32_st value)
{
	uint16_t base = (uint16_t)value.l;
	int result;

	result = DRV_setAccelTimeBase(base);
	UARTprintf("set accel time base=%d, result=%s\n", base, res_str[result]);

	return result;
}
#endif

int PARAM_setDirection(union32_st value)
{
	uint16_t dir = (uint16_t)value.l;
	int result=0;

	if(dir == 0)
	{
		MAIN_setForwardDirection();
		UARTprintf("set direction forward\n");
	}
	else
	{
		MAIN_setReverseDirection();
		UARTprintf("set direction backward\n");
	}
	STA_calcResolution4Reverse(m_status.cur_freq);

	return result;
}

#if 0
int PARAM_setVfFoc(union32_st value)
{
	uint16_t ctrl_type = (uint16_t)value.l;
	int result=0;

	if(MAIN_isSystemEnabled()) return 1;

	if(ctrl_type == VF_CONTROL)
	{
		DRV_enableVfControl();
		UARTprintf("set VF control\n");
	}
	else if(ctrl_type == FOC_CONTROL)
	{
		DRV_enableFocControl();
		UARTprintf("set FOC control\n");
	}
	else
		result = 1;

	return result;
}
#endif

int PARAM_setEnergySave(union32_st value)
{
	uint32_t on_off = value.l;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	//TODO : only work on FOC running
	result = DRV_setEnergySave((int)on_off);
	UARTprintf("set Energy_save %d %d is %s\n", (int)on_off, (int)value.l, res_str[result]);

	return result;
}

int PARAM_setPwmFreq(union32_st value)
{
	uint32_t pwm_freq = value.l;
	int result;
	const char *pwm_str[] = {"4kHz", "8kHz", "12kHz", "16kHz" };

	if(MAIN_isSystemEnabled()) return 1;

	result = DRV_setPwmFrequency((int)pwm_freq);
	UARTprintf("set pwm freq=%s, result=%s\n", pwm_str[pwm_freq], res_str[result]);

	return result;
}

int PARAM_setEnableJump0(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = FREQ_setJumpFreqEnable(0, (uint16_t)ldata);

	return result;
}

int PARAM_setEnableJump1(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = FREQ_setJumpFreqEnable(1, (uint16_t)ldata);

	return result;
}

int PARAM_setEnableJump2(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = FREQ_setJumpFreqEnable(2, (uint16_t)ldata);

	return result;
}

int PARAM_setJumpFreqLow0(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = FREQ_setJumpFreqLow(0, fdata);

	return result;
}

int PARAM_setJumpFreqLow1(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = FREQ_setJumpFreqLow(1, fdata);

	return result;
}

int PARAM_setJumpFreqLow2(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = FREQ_setJumpFreqLow(2, fdata);

	return result;
}

int PARAM_setJumpFreqHigh0(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = FREQ_setJumpFreqHigh(0, fdata);

	return result;
}

int PARAM_setJumpFreqHigh1(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = FREQ_setJumpFreqHigh(1, fdata);

	return result;
}

int PARAM_setJumpFreqHigh2(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = FREQ_setJumpFreqHigh(2, fdata);

	return result;
}

#if 0
int PARAM_setVoltageBoost(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	//TODO : need implementation
	result = DRV_setVoltageBoost(fdata);
	UARTprintf("set voltage boost %f is %s\n", fdata, res_str[result]);

	return result;
}


int PARAM_setTorqueLimit(union32_st value)
{
	float_t fdata = value.f;
	int result;

	result = DRV_setTorqueLimit(fdata);

	return result;
}
#endif

int PARAM_setBrakeType(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = BRK_setBrakeMethod(ldata);

	return result;
}

int PARAM_setBrakeFreq(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = BRK_setBrakeFreq(fdata);

	return result;
}

int PARAM_setDciBrakeStartFreq(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = DCIB_setStartFreq(fdata);
	UARTprintf("set brake start freq %f is %s\n", fdata, res_str[result]);

	return result;
}

int PARAM_setDciBrakeBlockTime(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = DCIB_setBlockTime(fdata);
	UARTprintf("set brake block time %f is %s\n", fdata, res_str[result]);

	return result;
}

int PARAM_setDciBrakeTime(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = DCIB_setBrakeTime(fdata);
	UARTprintf("set brake time %f for brake is %s\n", fdata, res_str[result]);

	return result;
}

int PARAM_setDciBrakeRate(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = DCIB_setBrakeRate(fdata);
	UARTprintf("set brake rate %f for brake is %s\n", fdata, res_str[result]);

	return result;
}

int PARAM_setOvlWarnLevel(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	result = OVL_setWarningLevel((uint16_t)ldata);
	UARTprintf("set Overload waning level %d is %s\n", (int)ldata, res_str[result]);

	return result;
}

int PARAM_setOvlWarnTime(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	result = OVL_setWarningTime((uint16_t)ldata);
	UARTprintf("set Overload warning duration %d is %s\n", (int)ldata, res_str[result]);

	return result;
}

int PARAM_setOvlEnableTrip(union32_st value)
{
	uint32_t ldata = value.l;
	int result=0;

	OVL_enbleOverloadTrip((uint16_t)ldata);
	UARTprintf("set Overload Trip enable=%d\n", (int)ldata);

	return result;
}

int PARAM_setOvlTripLevel(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	result = OVL_setTripLevel((uint16_t)ldata);
	UARTprintf("set Overload Trip level %d is %s\n", (int)ldata, res_str[result]);

	return result;
}

int PARAM_setOvlTripTime(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	result = OVL_setTripTime((uint16_t)ldata);
	UARTprintf("set Overload trip duration %d is %s\n", (int)ldata, res_str[result]);

	return result;
}

#if 0
int PARAM_setRegenResistance(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = REGEN_setRegenResistance(fdata);
	UARTprintf("set regen resistance %f ohm is %s\n", fdata, res_str[result]);

	return result;
}

int PARAM_setRegenResistThermal(union32_st value)
{
	float_t fdata = value.f;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = REGEN_setRegenThermal(fdata);
	UARTprintf("set regen thermal %f is %s\n", fdata, res_str[result]);

	return result;
}

int PARAM_setRegenResistPower(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = REGEN_setRegenResistancePower((uint16_t)ldata);
	UARTprintf("set regen power %d W is %s\n", (int)ldata, res_str[result]);

	return result;
}
#endif

int PARAM_setRegenDuty(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = REGEN_setRegenDuty(ldata);
	UARTprintf("set regen duty %d is %s\n", (int)ldata, res_str[result]);

	return result;
}

int PARAM_setRegenBand(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	if(MAIN_isSystemEnabled()) return 1;

	result = REGEN_setRegenBand(ldata);
	UARTprintf("set regen band %d is %s\n", (int)ldata, res_str[result]);

	return result;
}

int PARAM_setFanControl(union32_st value)
{
	uint32_t ldata = value.l;
	int result;

	result = DRV_setFanControl((uint16_t)ldata);
	UARTprintf("set Fan control %d is %s\n", (int)ldata, res_str[result]);

	return result;
}

void PARAM_startRun(void)
{
	if(!MAIN_isSystemEnabled())
	{
		MAIN_enableSystem();
		STA_calcResolution();
		UARTprintf("start running motor\n");

	    STA_printInvState();
	}
}

void PARAM_stopRun(void)
{
	if(MAIN_isSystemEnabled())
	{
		STA_setStopCondition();
		UARTprintf("STOP command\n");
	}
}

int PARAM_process(uint16_t index, union32_st data)
{
	int result=1;

	if(index >= INV_PARAM_INDEX_MAX) return result;

	result = iparam_func[index](data);
	UARTprintf("PARAM_process index=%d, fdata=%f, ldata=%d\n", index, data.f, (int)data.l);

	return result;
}

void PARAM_initErrInfo(void)
{
	err_info[ERR_CODE_INDEX].type = PARAMETER_TYPE_LONG;
	err_info[ERR_CODE_INDEX].value.l = 0;

	err_info[ERR_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	err_info[ERR_CURRENT_INDEX].value.f = 0.0;

	err_info[ERR_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	err_info[ERR_FREQ_INDEX].value.f = 0.0;
}

void PARAM_setErrInfo(uint16_t err_code, uint16_t err_status, float_t current, float_t freq)
{
	err_info[ERR_CODE_INDEX].value.arr[0] = err_code;
	err_info[ERR_CODE_INDEX].value.arr[1] = err_status;
	err_info[ERR_CURRENT_INDEX].value.f = current;
	err_info[ERR_FREQ_INDEX].value.f = freq;
}

uint16_t PARAM_getErrorInfo(uint16_t *buf)
{
#if 0 // test only
	err_info[ERR_CODE_INDEX].value.arr[0] = 0;
	err_info[ERR_CODE_INDEX].value.arr[1] = 1;
	err_info[ERR_CURRENT_INDEX].value.f = 10.5;
	err_info[ERR_FREQ_INDEX].value.f = 120.0;
#endif
	buf[0] = err_info[ERR_CODE_INDEX].value.arr[0];
	buf[1] = err_info[ERR_CODE_INDEX].value.arr[1];
	buf[2] = err_info[ERR_CURRENT_INDEX].value.arr[0];
	buf[3] = err_info[ERR_CURRENT_INDEX].value.arr[1];
	buf[4] = err_info[ERR_FREQ_INDEX].value.arr[0];
	buf[5] = err_info[ERR_FREQ_INDEX].value.arr[1];

	return (ERR_CODE_MAX*2); // return size
}

void PARAM_initInvStatus(void)
{
	inv_status[INV_STATUS_INDEX].type = PARAMETER_TYPE_LONG;
	inv_status[INV_STATUS_INDEX].value.l = 0;

	inv_status[INV_I_RMS_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_I_RMS_INDEX].value.f = 0.0;

	inv_status[INV_RUN_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_RUN_FREQ_INDEX].value.f = 0.0;

	inv_status[INV_DC_VOLTAGE_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_DC_VOLTAGE_INDEX].value.f = 0.0;

	inv_status[INV_IPM_TEMP_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_IPM_TEMP_INDEX].value.f = 0.0;

	inv_status[INV_MOTOR_TEMP_INDEX].type = PARAMETER_TYPE_LONG;
	inv_status[INV_MOTOR_TEMP_INDEX].value.f = 0;
}


void PARAM_setInvStatus(void)
{
	inv_status[INV_STATUS_INDEX].value.arr[0] = (MAIN_getDirection()<<8) | MAIN_isRunState();
	inv_status[INV_STATUS_INDEX].value.arr[1] = (internal_status.shaft_brake_enabled<<8) | ovl_alarm_enable;

	inv_status[INV_I_RMS_INDEX].value.f = MAIN_getIave();
	inv_status[INV_RUN_FREQ_INDEX].value.f = STA_getCurFreq();
	inv_status[INV_DC_VOLTAGE_INDEX].value.f = MAIN_getVdcBus();
	inv_status[INV_IPM_TEMP_INDEX].value.f = UTIL_readIpmTemperature();
	inv_status[INV_MOTOR_TEMP_INDEX].value.l = UTIL_readMotorTemperatureStatus();
}

uint16_t PARAM_getInvStatus(uint16_t *buf)
{
#if 0 // test only
	inv_status[INV_STATUS_INDEX].value.arr[0] = 0;
	inv_status[INV_STATUS_INDEX].value.arr[1] = 1;
	inv_status[INV_I_RMS_INDEX].value.f = 2.1;
	inv_status[INV_RUN_FREQ_INDEX].value.f = 30.2;
	inv_status[INV_DC_VOLTAGE_INDEX].value.f = 531.3;
	inv_status[INV_IPM_TEMP_INDEX].value.f = 63.4;
	inv_status[INV_MOTOR_TEMP_INDEX].value.l = 0;
#endif
	buf[0] = inv_status[INV_STATUS_INDEX].value.arr[0];
	buf[1] = inv_status[INV_STATUS_INDEX].value.arr[1];
	buf[2] = inv_status[INV_I_RMS_INDEX].value.arr[0];
	buf[3] = inv_status[INV_I_RMS_INDEX].value.arr[1];
	buf[4] = inv_status[INV_RUN_FREQ_INDEX].value.arr[0];
	buf[5] = inv_status[INV_RUN_FREQ_INDEX].value.arr[1];
	buf[6] = inv_status[INV_DC_VOLTAGE_INDEX].value.arr[0];
	buf[7] = inv_status[INV_DC_VOLTAGE_INDEX].value.arr[1];
	buf[8] = inv_status[INV_IPM_TEMP_INDEX].value.arr[0];
	buf[9] = inv_status[INV_IPM_TEMP_INDEX].value.arr[1];
	buf[10] = inv_status[INV_MOTOR_TEMP_INDEX].value.arr[0];
	buf[11] = inv_status[INV_MOTOR_TEMP_INDEX].value.arr[1];

	return (INV_STATUS_MAX*2); // return size
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


