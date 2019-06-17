/*
 * parameters.h
 *
 *  Created on: 2018. 7. 18.
 *      Author: hrjung
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_


typedef union
{
  uint16_t arr[2];
  float f;
  uint32_t l;
} union32_st;


typedef struct
{
	uint16_t	type;
	union32_st	value;
} inv_parameter_st;


// ctrl.vf_foc_sel
enum
{
	VF_CONTROL = 0,
	FOC_CONTROL,

};

// stop control : brake_method
enum
{
	REDUCE_SPEED_BRAKE,
	DC_INJECT_BRAKE,
	FREE_RUN_BRAKE,
	MAX_BRAKE
};

enum {
	ERR_CODE_INDEX = 0,
	ERR_CURRENT_INDEX,
	ERR_FREQ_INDEX,
	ERR_CODE_MAX,
};

enum {
	INV_STATUS_INDEX = 0,
	INV_I_RMS_INDEX,
	INV_RUN_FREQ_INDEX,
	INV_DC_VOLTAGE_INDEX,
	INV_IPM_TEMP_INDEX,
	INV_MOTOR_TEMP_INDEX,
	INV_STATUS_MAX,
};

#define PARAMETER_TYPE_LONG			0
#define PARAMETER_TYPE_FLOAT		1

enum {
	FREQ_VALUE_INDEX = 0,
	MAX_FREQ_INDEX,
	ACCEL_TIME_INDEX,
	DECEL_TIME_INDEX,
	ACCEL_BASE_INDEX,

	DIRECTION_INDEX,	 	// 5
// VF_FOC_SEL_INDEX,
	ENERGY_SAVE_INDEX,
	PWM_FREQ_INDEX,
	JUMP_ENABLE0_INDEX,
	JUMP_ENABLE1_INDEX,

	JUMP_ENABLE2_INDEX,		//10
	JUMP_LOW0_INDEX,
	JUMP_LOW1_INDEX,
	JUMP_LOW2_INDEX,
	JUMP_HIGH0_INDEX,

	JUMP_HIGH1_INDEX,			//15
	JUMP_HIGH2_INDEX,
//#define V_BOOST_INDEX
//	FOC_TORQUE_LIMIT_INDEX,
	BRK_TYPE_INDEX,
	BRK_FREQ_INDEX,
	BRK_DCI_START_FREQ_INDEX,

	BRK_DCI_BLOCK_TIME_INDEX,		//20
	BRK_DCI_BRAKING_TIME_INDEX,
	BRK_DCI_BRAKING_RATE_INDEX,
	OVL_WARN_LIMIT_INDEX,
	OVL_WR_DURATION_INDEX,

	OVL_ENABLE_INDEX,				//25
	OVL_TR_LIMIT_INDEX,
	OVL_TR_DURATION_INDEX,
//	REGEN_RESISTANCE_INDEX,
//	REGEN_THERMAL_INDEX,
//	REGEN_POWER_INDEX,
	REGEN_DUTY_INDEX,
	REGEN_BAND_INDEX,

	FAN_COMMAND_INDEX,		  	//30
	STATOR_RESISTANCE_INDEX,
	ROTATOR_RESISTANCE_INDEX,
	INDUCTANCE_INDEX,
	NOLOAD_CURRENT_INDEX,

	RATED_CURRENT_INDEX,		//35
	POLES_INDEX,
	INPUT_VOLTAGE_INDEX,
	RATED_FREQ_INDEX,
	INV_RUN_STOP_CMD_INDEX,

	INV_PARAM_INDEX_MAX,		//40
};

extern uint16_t vf_foc_control;
extern inv_parameter_st iparam[];
////////////////////////////////////////////////

extern void PARAM_init(void);

extern void PARAM_startRun(void);
extern void PARAM_stopRun(void);
extern int PARAM_setFreq(union32_st value);
#ifdef SUPPORT_ACCEL_TIME_BASE
extern int PARAM_setMaxFreq(union32_st value);
#endif
extern int PARAM_setAccel(union32_st value);
extern int PARAM_setDecel(union32_st value);
#ifdef SUPPORT_ACCEL_TIME_BASE
extern int PARAM_setAccelTimeBase(union32_st value);
#endif
extern int PARAM_setDirection(union32_st value);
//extern int PARAM_setVfFoc(union32_st value);
extern int PARAM_setEnergySave(union32_st value);
extern int PARAM_setPwmFreq(union32_st value);
extern int PARAM_setEnableJump0(union32_st value);
extern int PARAM_setEnableJump1(union32_st value);
extern int PARAM_setEnableJump2(union32_st value);
extern int PARAM_setJumpFreqLow0(union32_st value);
extern int PARAM_setJumpFreqLow1(union32_st value);
extern int PARAM_setJumpFreqLow2(union32_st value);
extern int PARAM_setJumpFreqHigh0(union32_st value);
extern int PARAM_setJumpFreqHigh1(union32_st value);
extern int PARAM_setJumpFreqHigh2(union32_st value);
//extern int PARAM_setVoltageBoost(union32_st value);
extern int PARAM_setTorqueLimit(union32_st value);
extern int PARAM_setBrakeType(union32_st value);
extern int PARAM_setBrakeFreq(union32_st value);
extern int PARAM_setDciBrakeStartFreq(union32_st value);
extern int PARAM_setDciBrakeBlockTime(union32_st value);
extern int PARAM_setDciBrakeTime(union32_st value);
extern int PARAM_setDciBrakeRate(union32_st value);
extern int PARAM_setOvlWarnLevel(union32_st value);
extern int PARAM_setOvlWarnTime(union32_st value);
extern int PARAM_setOvlEnableTrip(union32_st value);
extern int PARAM_setOvlTripLevel(union32_st value);
extern int PARAM_setOvlTripTime(union32_st value);
//extern int PARAM_setRegenResistance(union32_st value);
//extern int PARAM_setRegenResistThermal(union32_st value);
//extern int PARAM_setRegenResistPower(union32_st value);
extern int PARAM_setRegenDuty(union32_st value);
extern int PARAM_setRegenBand(union32_st value);
extern int PARAM_setFanControl(union32_st value);

extern void PARAM_update(uint16_t index, uint16_t *buf);
extern uint16_t PARAM_getValue(uint16_t index, uint16_t *buf);
extern int PARAM_process(uint16_t index, union32_st data);

extern void PARAM_initErrInfo(void);
extern void PARAM_setErrInfo(uint16_t err_code, uint16_t err_status, float current, float freq);
extern uint16_t PARAM_getErrorInfo(uint16_t *buf);

extern void PARAM_initInvStatus(void);
extern void PARAM_setInvStatus(void);
extern uint16_t PARAM_getInvStatus(uint16_t *buf);


#define JMP_ENABLE_BASE		JUMP_ENABLE0_INDEX
#define JMP_LOW_BASE		JUMP_LOW0_INDEX
#define JMP_HIGH_BASE		JUMP_HIGH0_INDEX

inline int FREQ_isJumpFreqUsed(int index)
{
	return (int)iparam[JMP_ENABLE_BASE + index].value.l;
}

inline int DRV_isVfControl(void)
{
	//return (iparam[VF_FOC_SEL_INDEX].value.l == VF_CONTROL);
	return (vf_foc_control == VF_CONTROL);
}

inline int DRV_isFocControl(void)
{
	//return (iparam[VF_FOC_SEL_INDEX].value.l == FOC_CONTROL);
	return (vf_foc_control == FOC_CONTROL);
}

inline int BRK_isDCIBrakeEnabled(void)
{
	return (iparam[BRK_TYPE_INDEX].value.l == DC_INJECT_BRAKE);
}
#endif /* PARAMETERS_H_ */
