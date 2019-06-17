/*
 * debug_module.c
 *
 *  Created on: 2017. 3. 7.
 *      Author: hrjung
 */

//#ifndef UNIT_TEST_ENABLED

#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
#include "main.h"

#include "build_defs.h"
#include "uartstdio.h"
#include "cmdline.h"

#include "parameters.h"
#include "inv_param.h"
#include "motor_param.h"

#include "drive.h"
#include "freq.h"
#include "state_func.h"
#include "brake.h"
#include "protect.h"
#include "err_trip.h"
#include "timer_handler.h"
#include "common_tools.h"
#include "cmd_queue.h"
#include "drv_spi.h"
#include "err_trip.h"
//#include "drv_accelerometer.h"

#ifdef UNIT_TEST_ENABLED
#include "test/unity.h"
#endif

/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */

#define MAX_COMMAND_NUM      2
#define NUM_OF_DEBUGCHAR    64

const char *res_str[2] = { "OK", "NOK" };
//const char *res_lmt[2] = { "MIN - MAX", "LOW - HIGH" };

/*******************************************************************************
 * TYPEDEFS
 */

typedef union
{
  uint16_t i_word[2];
  float_t f_data;
} union_fdata;

typedef union
{
  uint16_t i_word[2];
  uint32_t l_data;
} union_ldata;

typedef enum {
	DBG_CMD_SHOW_HELP,
	DBG_CMD_ECHO_CONFIG,
	//DBG_CMD_SET_SPEED,
	DBG_CMD_SET_FREQUENCY,
	DBG_CMD_SET_MAX_FREQUENCY,
	DBG_CMD_SET_JUMP_FREQ,
	DBG_CMD_SET_ACCEL_TIME,
	DBG_CMD_SET_ENERGY_SAVE,
	DBG_CMD_SET_V_BOOST,

	DBG_CMD_MAIN_CONTROL,

	DBG_CMD_START_MOTOR,
	DBG_CMD_STOP_MOTOR,
	DBG_CMD_END_MOTOR,
	DBG_CMD_SET_DIRECTION,
	DBG_CMD_SHOW_STATUS,
	DBG_CMD_READ_PARAMETER,

	DBG_CMD_BRAKE_CONTROL,
	DBG_CMD_SET_DCI_BRAKE,

	DBG_CMD_SHOW_MTR_PARAM,
	DBG_CMD_SHOW_TRIP,
	DBG_CMD_SHOW_MONITOR,
	DBG_CMD_SHOW_TEMP,
	DBG_CMD_SET_FAN,

	DBG_CMD_PROT_OVERLOAD,
	DBG_CMD_PROT_REGEN,
//	DBG_CMD_PROT_STALL,

	DBG_CMD_STEP_UP,
	DBG_CMD_STEP_DOWN,
	DBG_CMD_SET_STEP,
	DBG_CMD_SET_INIT_RELAY,
	DBG_CMD_SET_SHAFT_BRK,
	DBG_CMD_SET_LED,
#ifdef SUPPORT_OFFSET_MEASURE
	DBG_CMD_CALC_OFFSET,
#endif
	DBG_CMD_SET_PWM_AO,
	DBG_CMD_VERSION,

	// temp test debug command
#ifdef SAMPLE_ADC_VALUE
	DBG_CMD_GET_ADC_I_SAMPLE,
#endif
	DBG_CMD_QUE_TEST,
#ifdef UNIT_TEST_ENABLED
	DBG_UNIT_TEST,
#endif
	DBG_CMD_TEST,

	DBG_CMD_ENUM_MAX
} dbg_cmd_code_e;

enum
{
	FREQ_MAIN,
	FREQ_MIN,
	FREQ_MAX,
	FREQ_LOW,
	FREQ_HIGH
};

/*******************************************************************************
 * LOCAL VARIABLES
 */

int systemVersion = 0x1234;

unsigned char cmdString[NUM_OF_DEBUGCHAR];
int freq_step = 2;
uint16_t invbuf[15], errbuf[10], rparam[5];

/*******************************************************************************
 * EXTERNS
 */
extern CTRL_Handle ctrlHandle;
extern HAL_Handle halHandle;
#ifdef SUPPORT_MOTOR_PARAM
extern motor_param_st mtr_param;
#endif

extern tBoolean g_bNewCmd;
//extern monitor_param_st mnt;
extern uint32_t secCnt;
extern MOTOR_working_st m_status;
extern float_t sf4pu_rpm;
extern USER_Params gUserParams;
extern volatile MOTOR_Vars_t gMotorVars;
extern int for_rev_flag;
extern int ovl_alarm_enable;
extern uint16_t Vinst[];

extern _iq i_offset[3], v_offset[3];

extern uint16_t spi_rx_buf[];
extern inv_parameter_st err_info[ERR_CODE_MAX];

extern uint16_t gFlag_LogEnabled;

#ifdef SUPPORT_INIT_MAGNETIZE
extern float_t magnetize_rate;
#endif

extern float_t MAIN_getPwmFrequency(void);
extern float_t MAIN_getIu(void);
extern float_t MAIN_getIv(void);
extern float_t MAIN_getIw(void);
extern float_t MAIN_getIave(void);

extern float_t MAIN_getDC_lfp(void);
extern void MAIN_showPidGain(void);
extern void MAIN_setSpeedGain(int fw_enabled);

extern int regen_duty;
extern void REGEN_start(void);
extern void REGEN_end(void);
extern int REGEN_getDuty(void);

extern float_t DRV_getPwmFrequency(void);
extern void STA_printInvState(void);
extern int DCIB_getState(void);
extern void DCIB_setFlag(void);
extern void UTIL_clearInitRelay(void);
extern float_t UTIL_readIpmTemperature(void);
extern float_t UTIL_readMotorTemperature(void);
extern uint16_t UTIL_readMotorTemperatureStatus(void);

extern void ERR_printTripInfo(void);

#ifdef SUPPORT_MISS_PHASE_DETECT
#define I_MISS_SAMPLE_COUNT		50
extern float_t i_buff[3][I_MISS_SAMPLE_COUNT];
extern int u_low_cnt, u_high_cnt;
extern int v_low_cnt, v_high_cnt;
extern int w_low_cnt, w_high_cnt;
#endif

#ifdef SUPPORT_SPI_ACCELEROMETER
extern uint16_t SPI_readSensor(uint16_t regNum, uint16_t *rxData);
extern uint16_t SPI_writeSesnsor(uint16_t *txData);
#endif

#ifdef SUPPORT_AUTO_LOAD_TEST_
int ipm_disp_on = 0;
extern bool UTIL_readSwGpio(void);
extern int TEST_readSwitch(void);
#endif

#ifdef UNIT_TEST_ENABLED
uint16_t unit_test_running=0;

extern void test_setFreqParam(void); // test_freq.c
//extern void test_setSpeedParam(void); // test_speed.c
extern void test_setAccelTime(void);

//extern void test_processSpeedScaling(void); //test_resolution.c
//extern void test_processResolution(void);
extern void test_processConvertFreq(void); //test_resolution.c
//extern void test_processResolutionMinMax(void);
extern void test_processResolutionTargetFreq(void);

extern void test_setDciBrakeParam(void);
extern void test_setOverload(void);
extern void test_processDcVoltage(void);

extern void test_controlState(void);
extern void test_controlDrive(void);
extern void test_errorTrip(void); // test_trip.c

#endif

/*******************************************************************************
 * LOCAL FUNCTIONS
 */


#ifdef SUPPORT_DEBUG_TERMINAL
// Prototype statements for functions found within this file.
STATIC int dbg_processHelp(int argc, char *argv[]);
//STATIC int dbg_setSpeed(int argc, char *argv[]);
STATIC int dbg_setFreq(int argc, char *argv[]);
STATIC int dbg_setMaxFreq(int argc, char *argv[]);
STATIC int dbg_setJumpFreq(int argc, char *argv[]);
STATIC int dbg_setAccelTime(int argc, char *argv[]);
STATIC int dbg_setEnergySave(int argc, char *argv[]);
//STATIC int dbg_setVoltVoost(int argc, char *argv[]);

STATIC int dbg_setDriveControl(int argc, char *argv[]);

STATIC int dbg_runMotor(int argc, char *argv[]);
STATIC int dbg_stopMotor(int argc, char *argv[]);
STATIC int dbg_endMotor(int argc, char *argv[]);
STATIC int dbg_setDirection(int argc, char *argv[]);
STATIC int dbg_showMotorState(int argc, char *argv[]);
STATIC int dbg_readParameter(int argc, char *argv[]);

STATIC int dbg_setBrakeControl(int argc, char *argv[]);
STATIC int dbg_setDcInjBrake(int argc, char *argv[]);

STATIC int dbg_processMotorParam(int argc, char *argv[]);
STATIC int dbg_processTripInfo(int argc, char *argv[]);
STATIC int dbg_showInverterStatus(int argc, char *argv[]);
STATIC int dbg_showTempStatus(int argc, char *argv[]);
STATIC int dbg_setFanControl(int argc, char *argv[]);

STATIC int dbg_setOverload(int argc, char *argv[]);
STATIC int dbg_setRegen(int argc, char *argv[]);

STATIC int dbg_stepUpFreq(int argc, char *argv[]);
STATIC int dbg_stepDownFreq(int argc, char *argv[]);
STATIC int dbg_setStepFreq(int argc, char *argv[]);
STATIC int dbg_setInitRelay(int argc, char *argv[]);
STATIC int dbg_setShaftBrake(int argc, char *argv[]);
STATIC int dbg_setLed(int argc, char *argv[]);
#ifdef SUPPORT_OFFSET_MEASURE
STATIC int dbg_measureOffset(int argc, char *argv[]);
#endif
STATIC int dbg_showVersion(int argc, char *argv[]);

#ifdef SAMPLE_ADC_VALUE
STATIC int dbg_getAdcSample(int argc, char *argv[]);
#endif

STATIC int dbg_testCmdQueue(int argc, char *argv[]);
STATIC int dbg_testMotorParam(int argc, char *argv[]);
#ifdef UNIT_TEST_ENABLED
STATIC int dbg_UnitTest(int argc, char *argv[]);
#endif
STATIC int dbg_tmpTest(int argc, char *argv[]);

STATIC int EchoSetting(int argc, char*argv[]);

void dbg_logo(void);

/*******************************************************************************
 * GLOBAL VARIABLES
 */

//const tCmdLineEntry g_sCmdTable[DBG_CMD_ENUM_MAX] =
tCmdLineEntry g_sCmdTable[DBG_CMD_ENUM_MAX] =
{
#if 0 // reduce 5KB
	{"help", dbg_processHelp, " help : show command list"},
	{"echo", EchoSetting, " echo off/on"},
	//{"spd", dbg_setSpeed, " spd index(0-7) spd(30-180) : set spd for each step"},
	{"freq", dbg_setFreq, " frequency setting\n" \
			"   freq main freq(5-400) : set freq for each step\n"  \
			"   freq show : display freq settings"
	},
	{"maxf", dbg_setMaxFreq, " set Max frequency "},
	{"jmpf", dbg_setJumpFreq, " jmpf index(0-2) low(1-400) high(1-400) : set jump freq range, low < high"},
	{"time", dbg_setAccelTime, " Accel/Decel time setting\n" \
			"   time acc time(0-300): set accel time \n"  \
			"   time dec time(0-300): set decel time \n"  \
			"   time show : display time settings"
	},
	{"engy", dbg_setEnergySave, " engy 0,1 : energy save off/on"},
	//{"vst", dbg_setVoltVoost, " vst rate(0-1000) : v_boost"},
	{"drv", dbg_setDriveControl, " Main Drive Control setting\n" \
				"   drv pwm freq(0~3): set pwm frequency \n"  \
				"   drv show : display drive control settings"
	},

	{"start", dbg_runMotor, " start : run motor"},
	{"stop", dbg_stopMotor, " stop : stop motor"},
	{"end", dbg_endMotor, " end : end PWM"},
	{"dir", dbg_setDirection, " dir 0(forward) or 1(reverse) : set motor direction"},
	{"state", dbg_showMotorState, " state : display running status"},
	{"rprm", dbg_readParameter, " rprm : read parameter"},

	{"brk", dbg_setBrakeControl, " Brake control setting" \
			"   brk mth method(0-2) : select stop method\n" \
			"   brk thld threshold(0-50) : set start freq for brake\n" \
			"   brk show : display brake settings"
	},
	{"dcbrk", dbg_setDcInjBrake, " DC injection Brake setting\n" \
			"   dcbrk frq freq(0-50) : set start freq for dc brake\n" \
			"   dcbrk btime time(0-60) : set block time for dc brake\n" \
			"   dcbrk brake rate(0-200) time(0-60) : set dc rate and time for brake\n"  \
			"   dcbrk show : display DC brake settings"
	},

	{"mtrp", dbg_processMotorParam, "  mtrp : show motor parameters"},
	{"trip", dbg_processTripInfo, " trip : process trip info"},
	{"invs", dbg_showInverterStatus, " invs : display inverter status"},
	{"temp", dbg_showTempStatus, " temp : display temperature status"},
	{"fan", dbg_setFanControl, " fan : fan control"},

	{"ovl", dbg_setOverload, " overload protect settings\n" 			\
			"   ovl en flag(0, 1) : enable/disable overload trip\n"  		\
			"   ovl wl level(30-150) : set warning current level\n"  		\
			"   ovl wd dur(0-30) : set warning duration\n"  				\
			"   ovl tl level(30-200) : set trip current level\n"  		\
			"   ovl td dur(0-60) : set trip duration\n"  				\
			"   ovl show : display overload settings"
	},

	{"regen", dbg_setRegen, " Regen setting" \
			"   regen res ohm(150-500) power(10-65535) : set resister value and power\n"  \
			"   regen thml thermal(0-65535) reduce(0-150): set thermal and reduce rate\n" \
			"   regen show : display regen settings"
	},

	{"up", dbg_stepUpFreq, " up : Freq step up"},
	{"dn", dbg_stepDownFreq, " up : Freq step down"},
	{"step", dbg_setStepFreq, " step : set step freqquency"},
	{"irel", dbg_setInitRelay, " irel : set Init Relay"},

	{"irel", dbg_setInitRelay, " irel : set Init Relay"},
	{"sbrk", dbg_setShaftBrake, " sbrk (0/1) : set Shaft Brake off/on"},
	{"led", dbg_setLed, " led (0~3) 0/1 : set Led R,G,R2,G2 off/on"},
#ifdef SUPPORT_OFFSET_MEASURE
	{"ofs", dbg_measureOffset, " Offset measurement during 3 sec" },
#endif
	{"ver", dbg_showVersion, " ver : show HW, FW version"},

#ifdef SAMPLE_ADC_VALUE
	{"iadc", dbg_getAdcSample, " iadc (0,1) read ADC sample"},
#endif
	{"que", dbg_testCmdQueue, " que : test cmd queue"},
#ifdef UNIT_TEST_ENABLED
	{"utest", dbg_UnitTest, " utest : unit test"},
#endif
	{"tmp", dbg_tmpTest, " tmp : test command"}
#else
	{"help", dbg_processHelp, " help : show command list"},
	{"echo", EchoSetting, " echo off/on"},
	{"freq", dbg_setFreq, " frequency setting"},
	{"maxf", dbg_setMaxFreq, " set Max frequency "},
	{"jmpf", dbg_setJumpFreq, " jump freq setting"},
	{"time", dbg_setAccelTime, " Accel/Decel time"},
	{"engy", dbg_setEnergySave, " energy save off/on"},
	//{"vst", dbg_setVoltVoost, " v_boost"},
	{"drv", dbg_setDriveControl, " Main Drive"},

	{"start", dbg_runMotor, " start"},
	{"stop", dbg_stopMotor, " stop"},
	{"end", dbg_endMotor, " end"},
	{"dir", dbg_setDirection, " dir 0/1"},
	{"state", dbg_showMotorState, " state"},
	{"rprm", dbg_readParameter, " read parameter"},

	{"brk", dbg_setBrakeControl, " brk Setting"},
	{"dcbrk", dbg_setDcInjBrake, " DCI Brake setting"},

	{"mtrp", dbg_processMotorParam, "  motor show motor parameters"},
	{"trip", dbg_processTripInfo, " process trip info"},
	{"invs", dbg_showInverterStatus, " inverter status"},
	{"temp", dbg_showTempStatus, " temperature status"},
	{"fan", dbg_setFanControl, " Fan control"},

	{"ovl", dbg_setOverload, " overload protect"},
	{"regen", dbg_setRegen, " Regen setting" },

	{"up", dbg_stepUpFreq, " up"},
	{"dn", dbg_stepDownFreq, " dn"},
	{"step", dbg_setStepFreq, " step"},
	{"irel", dbg_setInitRelay, " irel"},
	{"sbrk", dbg_setShaftBrake, " sbrk"},
	{"led", dbg_setLed, " led"},
#ifdef SUPPORT_OFFSET_MEASURE
	{"ofs", dbg_measureOffset, " Offset" },
#endif
	{"ver", dbg_showVersion, " ver"},

#ifdef SAMPLE_ADC_VALUE
	{"iadc", dbg_getAdcSample, " read ADC I"},
#endif
	{"que", dbg_testCmdQueue, " que "},
#ifdef UNIT_TEST_ENABLED
	{"utest", dbg_UnitTest, " utest "},
#endif
	{"tmp", dbg_tmpTest, " tmp"}
#endif
};


/*
 *  ======== function ========
 */

void printLog(void)
{
	UARTprintf("Speed: %f krpm, Torque: %f Nm \n", _IQtoF(gMotorVars.Speed_krpm), _IQtoF(gMotorVars.Torque_Nm));
	UARTprintf("IdRef: %f A, Id_rated: %f A,  \n", _IQtoF(gMotorVars.IdRef_A), _IQtoF(gMotorVars.MagnCurr_A));
	//UARTprintf("IdRef: %f A, Id_A: %f A \n", _IQtoF(gMotorVars.IdRef_A), _IQtoF(gMotorVars.Id_A));
}

STATIC void dbg_setQueCommand(uint16_t index, union32_st data)
{
	cmd_type_st que_data;

	que_data.cmd = SPICMD_PARAM_W;
	que_data.index = index;
	que_data.data = data;
	if(!QUE_isFull())
	{
		QUE_putCmd(que_data);
	}
	else
	{
		UARTprintf("QUE_full !!\n");
	}
}

// command function lists
STATIC int dbg_processHelp(int argc, char *argv[])
{
    int i;
//    UARTprintf("cmd <arg1> <arg2>\n");
    dbg_logo();
    for(i=0;i<DBG_CMD_ENUM_MAX;i++)
    {
        UARTprintf("%s\n", g_sCmdTable[i].pcHelp);
        //UARTFlushTx(0);
    }

    return 0;
}

STATIC void dbg_showAccelTimeSetting(void)
{
	UARTprintf(" Accel/Decel time Settings\n");
#ifdef SUPPORT_ACCEL_TIME_BASE
	UARTprintf("\t base=%d, accel time %f, decel time %f\n",
			(int)iparam[ACCEL_BASE_INDEX].value.l, iparam[ACCEL_TIME_INDEX].value.f, iparam[DECEL_TIME_INDEX].value.f);
#else
	UARTprintf("\t accel time %f decel time %f\n", iparam[ACCEL_TIME_INDEX].value.f, iparam[DECEL_TIME_INDEX].value.f);
#endif
}


STATIC void dbg_showBrakeControlParam(void)
{
	UARTprintf(" Brake Control Settings\n");
	UARTprintf("\t method %d, brk_freq %f\n", (int)iparam[BRK_TYPE_INDEX].value.l, iparam[BRK_FREQ_INDEX].value.f);
}

STATIC void dbg_showDciBrakeParam(void)
{
	UARTprintf(" DC injection Brake Settings const=%f, state=%d\n", dev_const.dci_pwm_rate, DCIB_getState());
	UARTprintf("\t brake mode %d, start freq %f, block time %f\n", (int)iparam[BRK_TYPE_INDEX].value.l, iparam[BRK_DCI_START_FREQ_INDEX].value.f, iparam[BRK_DCI_BLOCK_TIME_INDEX].value.f);
	UARTprintf("\t inject rate %f %%, time %f to brake\n", iparam[BRK_DCI_BRAKING_RATE_INDEX].value.f, iparam[BRK_DCI_BRAKING_TIME_INDEX].value.f);
}

STATIC void dbg_showMotorParam(void)
{
	UARTprintf(" Motor Parameter Settings\n");
#ifdef SUPPORT_MOTOR_PARAM
	UARTprintf("\t input_volt: %d, poles %d, rated_freq: %d\n", mtr_param.voltage_in, mtr_param.pole_pairs, mtr_param.rated_freq);
	UARTprintf("\t no load current: %f, max_current: %f\n", mtr_param.noload_current, mtr_param.max_current);
	UARTprintf("\t Rs: %f, Rr: %f, Ls: %f\n", mtr_param.Rs, mtr_param.Rr, mtr_param.Ls);
#else
	UARTprintf("\t input_volt: %d, poles %d, rated_freq: %d\n", USER_MOTOR_VOLTAGE_IN, USER_MOTOR_NUM_POLE_PAIRS, USER_MOTOR_RATED_FREQUENCY);
	UARTprintf("\t no load current: %f, max_current: %f\n", USER_MOTOR_NO_LOAD_CURRENT, USER_MOTOR_MAX_CURRENT);
	UARTprintf("\t Rs: %f, Rr: %f, Ls: %f\n", USER_MOTOR_Rs, USER_MOTOR_Rr, USER_MOTOR_Ls_d);
#endif

	UARTprintf("\t gUser.max_current: %f\n", gUserParams.maxCurrent);
}

STATIC void dbg_showTripData(void)
{
	const char *state_str[5] = { "START", "STOP", "ACCEL", "DECEL", "RUN" };
	UARTprintf(" Error info display\n");
	//for(i=0; i<FAULT_HISTORY_NUM; i++)
	{
		UARTprintf("\t errCode: %d, state=%s, cur: %f, freq: %f \n", \
				err_info[ERR_CODE_INDEX].value.arr[0], state_str[err_info[ERR_CODE_INDEX].value.arr[1]], \
				err_info[ERR_CURRENT_INDEX].value.f, err_info[ERR_FREQ_INDEX].value.f);
	}

	ERR_printTripInfo();
}

extern _iq gVbus_lpf;
STATIC void dbg_showMonitorParam(void)
{
	UARTprintf(" Inverter Status display\n");

	float_t gOver = _IQtoF(_IQdiv(_IQ(1.0),gVbus_lpf));
	UARTprintf("\t Iu: %f, Iv: %f, Iw: %f, DC voltage: %f\n", MAIN_getIu(), MAIN_getIv(), MAIN_getIw(), MAIN_getVdcBus());
	UARTprintf("\t RMS Iu: %f, Iv: %f, Iw: %f, Iave: %f \n", internal_status.Irms[0], internal_status.Irms[1], internal_status.Irms[2], m_status.current);
	UARTprintf("\t RMS Vu: %f, Vv: %f, Vw: %f\n", internal_status.Vrms[0], internal_status.Vrms[1], internal_status.Vrms[2]);
	UARTprintf("\t RMS Vppu: %f, Vppv: %f, Vppw: %f\n", (float)internal_status.Vpprms[0], (float)internal_status.Vpprms[1], (float)internal_status.Vpprms[2]);
//	UARTprintf("\t Volt: Vu: %f, Vv: %f, Vw: %f \n", internal_status.Vu_inst, internal_status.Vv_inst, internal_status.Vw_inst); //, MAIN_getDC_lfp());
//	UARTprintf("\t Volt: U-V: %f, V-W: %f, W-U: %f \n", (internal_status.Vu_inst - internal_status.Vv_inst), (internal_status.Vv_inst-internal_status.Vw_inst), (internal_status.Vw_inst-internal_status.Vu_inst));
//	UARTprintf("\t input status: 0x%x, out status: 0x%x\n", (int)((mnt.dio_status>>16)&0x0F), (int)(mnt.dio_status&0x0F));
	UARTprintf("\t Motor RPM: %f  Freq: %f,  target %f, dir=%d \n", STA_getCurSpeed(), m_status.cur_freq, m_status.target_freq, (int)m_status.direction);
	UARTprintf("\t Motor status %d, accel: %f  decel: %f gOver=%f \n", m_status.status, m_status.acc_res, m_status.dec_res, gOver);
}

STATIC void dbg_showOverloadParam(void)
{
	UARTprintf(" Overload Setting\n");
	UARTprintf("\t enable: %d\n", (int)iparam[OVL_ENABLE_INDEX].value.l);
	UARTprintf("\t Warning level: %d, duration: %d\n", (int)iparam[OVL_WARN_LIMIT_INDEX].value.l, (int)iparam[OVL_WR_DURATION_INDEX].value.l);
	UARTprintf("\t Trip level: %d, duration: %d\n", (int)iparam[OVL_TR_LIMIT_INDEX].value.l, (int)iparam[OVL_TR_DURATION_INDEX].value.l);
	UARTprintf("\t warn: %f, trip: %f OVC: %f\n", dev_const.warn_level, dev_const.trip_level, dev_const.ovc_level);
}

STATIC void dbg_showRegenParam(void)
{
	//float_t value = sqrtf(0.9*150.0*50.0);
//	UARTprintf("\t resistance ohm %f, power %d \n", iparam[REGEN_RESISTANCE_INDEX].value.f, (int)iparam[REGEN_POWER_INDEX].value.l);
//	UARTprintf("\t thermal %f, band %d \n", iparam[REGEN_THERMAL_INDEX].value.f, (int)iparam[REGEN_BAND_INDEX].value.l);
//	UARTprintf("\t V_max %f, %f, regen_duty %d \n", dev_const.regen_max_V, value, REGEN_getDuty());
	UARTprintf("\t duty %d, band %d \n", iparam[REGEN_DUTY_INDEX].value.l, (int)iparam[REGEN_BAND_INDEX].value.l);
	UARTprintf("\t V_max %f, regen_duty %d \n", dev_const.regen_max_V, REGEN_getDuty());
}


#if 1
STATIC int dbg_setFreq(int argc, char *argv[])
{
	//int result;
	uint16_t value;
	float_t f_value, max_freq = MAX_FREQ_VALUE;
	union32_st data;

    if(argc != 1 && argc != 2) goto freq_err;

    if(argc == 1)
    {
    	UARTprintf("Command frequency %f\n", iparam[FREQ_VALUE_INDEX].value.f);
    	return 0;
    }

	value = (uint16_t)atoi(argv[1]);
	f_value = (float_t)(value/FREQ_INPUT_RESOLUTION);
#ifdef SUPPORT_ACCEL_TIME_BASE
	max_freq = FREQ_getMaxFreqValue();
#endif
	if(f_value > MIN_FREQ_VALUE && f_value <= max_freq)
	{
		data.f = f_value;
		dbg_setQueCommand(FREQ_VALUE_INDEX, data);
	}
	else
	{
		goto freq_err;
	}

    return 0;

freq_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_FREQUENCY].pcHelp);
	return 1;
}

STATIC int dbg_setMaxFreq(int argc, char *argv[])
{
	//int result;
	uint16_t value;
	float_t f_value;
	union32_st data;

    if(argc != 1 && argc != 2) goto maxf_err;

    if(argc == 1)
    {
    	UARTprintf("Max frequency %f\n", iparam[MAX_FREQ_INDEX].value.f);
    	return 0;
    }

	value = (uint16_t)atoi(argv[1]);
	f_value = (float_t)(value/FREQ_INPUT_RESOLUTION);
	if(f_value > MIN_FREQ_VALUE && f_value < MAX_FREQ_VALUE)
	{
		data.f = f_value;
		dbg_setQueCommand(MAX_FREQ_INDEX, data);
	}
	else
	{
		goto maxf_err;
	}

    return 0;

    maxf_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_FREQUENCY].pcHelp);
	return 1;
}

STATIC int dbg_setJumpFreq(int argc, char *argv[])
{
	int i, index;
	uint16_t low, high;
	float_t f_low, f_high;
	union32_st data;

    if(argc != 1 && argc != 4) goto jmp_err;

    if(argc == 1) // show settings
    {
    	UARTprintf("Jump frequency setting\n");
    	for(i=0; i<MAX_JUMP_FREQ_NUM; i++)
    	{
    		if(iparam[JUMP_ENABLE0_INDEX + i].value.l)
    			UARTprintf("  Jump freq[%d]: %f - %f\n", i, iparam[JUMP_LOW0_INDEX+i].value.f, iparam[JUMP_HIGH0_INDEX+i].value.f);
    	}

    	return 0;
    }

    index = atoi(argv[1]);
    low = (uint16_t)atoi(argv[2]);
    high = (uint16_t)atoi(argv[3]);

    if(low > high) goto jmp_err;

    if(index==0 || index==1 || index==2)
    {
    	// enable
    	data.l = 1;
    	dbg_setQueCommand(JUMP_ENABLE0_INDEX+index, data);

		// low
    	f_low = (float_t)(low/FREQ_INPUT_RESOLUTION);
    	data.f = f_low;
    	dbg_setQueCommand(JUMP_LOW0_INDEX+index, data);

		// high
    	f_high = (float_t)(high/FREQ_INPUT_RESOLUTION);
    	data.f = f_high;
    	dbg_setQueCommand(JUMP_HIGH0_INDEX+index, data);
    }
    else
    	goto jmp_err;


    return 0;

jmp_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_JUMP_FREQ].pcHelp);
    return 1;
}

#else
STATIC int dbg_setSpeed(int argc, char *argv[])
{
	int index, value, result;

	if(argc == 1)
	{
		dbg_showSpeedSettings();
		return 0;
	}
	else if(argc == 3)
	{
		index = atoi(argv[1]);
		value = atoi(argv[2]);
		result = DRV_setSpeedValue(index, value);
		UARTprintf("set speed=%d at index=%d, result=%s\n", value, index, res_str[result]);
	}
	else
	{
		goto spd_err;
	}

    return 0;

spd_err:
	UARTprintf("speed input range : %d ~ %d\n",  (int)dev_const.spd_rpm_min, (int)dev_const.spd_rpm_max);
	return 1;
}
#endif

STATIC int dbg_setAccelTime(int argc, char *argv[])
{
	int value;
	float_t f_val;
	union32_st data;

    if(argc != 2 && argc != 3) goto acc_err;

    if(argc == 2 && strcmp("show", argv[1]) == 0)
    {
    	dbg_showAccelTimeSetting();
		return 0;
    }

    if(argc == 3)
    {
    	value = atoi(argv[2]);
    	f_val = (float_t)value/10.0;
    	data.f = f_val;
    	if(strcmp("acc", argv[1]) == 0)
    	{
    		dbg_setQueCommand(ACCEL_TIME_INDEX, data);
    	}
    	else if(strcmp("dec", argv[1]) == 0)
    	{
    		dbg_setQueCommand(DECEL_TIME_INDEX, data);
    	}
    	else
    		goto acc_err;


		return 0;
    }

acc_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_ACCEL_TIME].pcHelp);
	return 1;
}

STATIC int dbg_setEnergySave(int argc, char *argv[])
{
	int on_off;
	union32_st data;

	if(argc != 1 && argc != 2) goto save_err;

	if(argc == 1)
	{
		UARTprintf("energy save is %d\n", (int)iparam[ENERGY_SAVE_INDEX].value.l);
		return 0;
	}

	on_off = atoi(argv[1]);
	if(on_off > ESAVE_BOTH) goto save_err;

	data.l = (uint32_t)on_off;
	dbg_setQueCommand(ENERGY_SAVE_INDEX, data);

	return 0;

save_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_ENERGY_SAVE].pcHelp);
	return 1;
}

#if 0
STATIC int dbg_setVoltVoost(int argc, char *argv[])
{
	int value;
	float_t f_value;
	union32_st data;

	if(argc != 1 && argc != 2) goto boost_err;

	if(argc == 1)
	{
		UARTprintf("v_boost is %f\n", iparam[V_BOOST_INDEX].value.f);
		return 0;
	}

	value = atoi(argv[1]);
	if(value < 0 || value > 150) goto boost_err;

	f_value = (float_t)(value/10.0);
	data.f = f_value;
	dbg_setQueCommand(V_BOOST_INDEX, data);

	return 0;

boost_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_V_BOOST].pcHelp);
	return 1;
}
#endif

STATIC int dbg_setDriveControl(int argc, char *argv[])
{
	int value;
	union32_st data;

    if(argc != 2 && argc != 3) goto drv_err;

#if 1
    if(argc == 2)
    {
    	if(strcmp("vf", argv[1]) == 0)
    	{
//    		data.l = (uint32_t)VF_CONTROL;
//    		dbg_setQueCommand(VF_FOC_SEL_INDEX, data);
    		DRV_enableVfControl();
    		UARTprintf("set VF control\n");
    	}
    	else if(strcmp("foc", argv[1]) == 0)
    	{
//    		data.l = (uint32_t)FOC_CONTROL;
//    		dbg_setQueCommand(VF_FOC_SEL_INDEX, data);
    		DRV_enableFocControl();
    		UARTprintf("set FOC control\n");
    	}
    	else
    		goto drv_err;
    }
#endif

    if(argc == 3)
    {
    	value = atoi(argv[2]);
    	if(strcmp("pwm", argv[1]) == 0)
    	{
    		data.l = (uint32_t)value;
    		dbg_setQueCommand(PWM_FREQ_INDEX, data);
    	}
    	else
    		goto drv_err;
    }
    return 0;

drv_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_MAIN_CONTROL].pcHelp);
	return 1;
}


STATIC int dbg_runMotor(int argc, char *argv[])
{
	cmd_type_st que_data;

    if(argc >= 2) goto run_err;

#if 1
	que_data.cmd = SPICMD_CTRL_RUN;
	que_data.index = INV_RUN_STOP_CMD_INDEX;
	que_data.data.l = 0;
	if(!QUE_isFull())
	{
		QUE_putCmd(que_data);
	}
	else
	{
		UARTprintf("QUE_full !!\n");
	}

#else
	MAIN_enableSystem();
	STA_calcResolution();
	UARTprintf("start running motor\n");

    STA_printInvState();
#endif

    return 0;

run_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_START_MOTOR].pcHelp);
    return 1;
}

STATIC int dbg_stopMotor(int argc, char *argv[])
{
	cmd_type_st que_data;

    if(argc >= 2) goto stop_err;

#if 1
	que_data.cmd = SPICMD_CTRL_STOP;
	que_data.index = INV_RUN_STOP_CMD_INDEX;
	que_data.data.l = 0;
	if(!QUE_isFull())
	{
		QUE_putCmd(que_data);
		UARTprintf("reduce speed cur_freq=%f to stop\n", m_status.cur_freq);
		UARTprintf("resolution acc_res=%f, dec_res=%f\n", m_status.acc_res, m_status.dec_res);
	}
	else
	{
		UARTprintf("QUE_full !!\n");
	}

#else

	STA_setStopCondition();
	UARTprintf("reduce speed cur_freq=%f to stop\n", m_status.cur_freq);
	UARTprintf("resolution acc_res=%f, dec_res=%f\n", m_status.acc_res, m_status.dec_res);
#endif

    return 0;

stop_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_STOP_MOTOR].pcHelp);
    return 1;
}

STATIC int dbg_endMotor(int argc, char *argv[])
{
    if(argc >= 2) goto end_err;

	MAIN_disableSystem();
	UARTprintf("stop running motor\n");

    STA_printInvState();

    return 0;

end_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_END_MOTOR].pcHelp);
    return 1;
}

STATIC int dbg_setDirection(int argc, char *argv[])
{
	int dir;
	union32_st data;

    if(argc != 2) goto dir_err;

	dir = atoi(argv[1]);
	if(dir != 0 && dir != 1) goto dir_err;

	data.l = (uint32_t)dir;
	dbg_setQueCommand(DIRECTION_INDEX, data);

    return 0;

dir_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_DIRECTION].pcHelp);
    return 1;
}

STATIC int dbg_showMotorState(int argc, char *argv[])
{
	const char *state_str[5] = { "START", "STOP", "ACCEL", "DECEL", "RUN" };

    if(argc != 1) goto sta_err;

    UARTprintf(" Running speed %d, current %f\n", MAIN_getCurrentSpeed(), STA_getCurrent());

    UARTprintf(" Motor status %s\n", state_str[m_status.status]);

    return 0;

sta_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_STATUS].pcHelp);
    return 1;
}

STATIC int dbg_readParameter(int argc, char *argv[])
{
	uint16_t index, type;
	union32_st val;
	uint16_t i, buf[4];

    if(argc != 2) goto param_err;

    index = (uint16_t)atoi(argv[1]);
	if(index >= INV_PARAM_INDEX_MAX)
	{
		UARTprintf(" Error index bigger than %d\n", INV_PARAM_INDEX_MAX);
		goto param_err;
	}

	for(i=0; i<4; i++) buf[i] = 0;
	PARAM_getValue(index, &buf[0]);

	type = iparam[index].type;
	val.arr[0] = buf[0];
	val.arr[1] = buf[1];
	if(type == PARAMETER_TYPE_LONG)
	{
		UARTprintf(" param[%d] is long, value = %d\n", index, (int)val.l);
	}
	else
	{
		UARTprintf(" param[%d] is float, value = %f\n", index, val.f);
	}

    return 0;

param_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_READ_PARAMETER].pcHelp);
    return 1;
}

STATIC int dbg_setBrakeControl(int argc, char *argv[])
{
	int value;
	float_t f_value;
	union32_st data;

	if(argc < 2 || argc > 3) goto brk_err;

	if(argc == 2)
	{
		if(strcmp(argv[1], "show") == 0)
		{
			dbg_showBrakeControlParam();
			return 0;
		}
		else
			goto brk_err;
	}

	if(argc == 3)
	{
		value = atoi(argv[2]);
		if(strcmp(argv[1], "mth")==0)
		{
			data.l = (uint32_t)value;
			dbg_setQueCommand(BRK_TYPE_INDEX, data);
		}
		else if(strcmp(argv[1], "freq")==0)
		{
			f_value = (float_t)(value/FREQ_INPUT_RESOLUTION);
			data.f = f_value;
			dbg_setQueCommand(BRK_FREQ_INDEX, data);
		}
		else
			goto brk_err;
	}

	return 0;

brk_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_BRAKE_CONTROL].pcHelp);
	return 1;
}

STATIC int dbg_setDcInjBrake(int argc, char *argv[])
{
	int value;
	float_t f_value;
	union32_st data;

	if(argc < 2 && argc > 3) goto dcib_err;

	if(argc == 2)
	{
		if(strcmp(argv[1], "show") == 0)
		{
			dbg_showDciBrakeParam();
			return 0;
		}
		else if(strcmp(argv[1], "on") == 0)
		{
			data.l = (uint32_t)DC_INJECT_BRAKE;
			dbg_setQueCommand(BRK_TYPE_INDEX, data);
		}
		else if(strcmp(argv[1], "off") == 0)
		{
			data.l = (uint32_t)REDUCE_SPEED_BRAKE;
			dbg_setQueCommand(BRK_TYPE_INDEX, data);
		}
		else
			goto dcib_err;
	}

	if(argc == 3)
	{
		value = atoi(argv[2]);
		f_value = (float_t)(value/FREQ_INPUT_RESOLUTION);
		data.f = f_value;
		if(strcmp(argv[1], "frq")==0)
		{
			dbg_setQueCommand(BRK_DCI_START_FREQ_INDEX, data);
		}
		else if(strcmp(argv[1], "btime")==0)
		{
			dbg_setQueCommand(BRK_DCI_BLOCK_TIME_INDEX, data);
		}
		else if(strcmp(argv[1], "rate")==0)
		{
			dbg_setQueCommand(BRK_DCI_BRAKING_RATE_INDEX, data);
		}
		else if(strcmp(argv[1], "time")==0)
		{
			dbg_setQueCommand(BRK_DCI_BRAKING_TIME_INDEX, data);
		}
		else
			goto dcib_err;

	}

	return 0;

dcib_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_DCI_BRAKE].pcHelp);
	return 1;
}

#if 1
STATIC int dbg_processMotorParam(int argc, char *argv[])
{
	if(argc == 1)
	{
		dbg_showMotorParam();
	}
	else if(argc == 2)
	{
#ifdef SUPPORT_MOTOR_PARAM
		uint16_t mtr_set;

		mtr_set = (uint16_t)atoi(argv[1]);
		if(mtr_set >= MOTOR_TYPE_MAX) goto mtr_err;

		MPARAM_init(MOTOR_SY_1_5K_TYPE);
		MPARAM_setMotorParam(&gUserParams);

		CTRL_setParams(ctrlHandle,&gUserParams);
		CTRL_setUserMotorParams(ctrlHandle);
		UARTprintf(" update motor param in running %d\n");
#else
		UARTprintf(" not supported\n");
#endif
	}
	else
		goto mtr_err;

	return 0;

mtr_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_MTR_PARAM].pcHelp);
	return 1;
}
#else
STATIC int dbg_processMotorParam(int argc, char *argv[])
{
	int value;
	int f_val;

	if(argc != 2 && argc != 3) goto mtr_err;

	if(argc == 2)
	{
		if(strcmp(argv[1], "show") == 0)
		{
			dbg_showMotorParam();
			return 0;
		}
		else
			goto mtr_err;
	}

	if(strcmp(argv[1], "slip")==0)
	{
		value = atoi(argv[2]);
		if(value < 0 || value > 10) goto mtr_err;

		mtr.slip_offset = value;
		UARTprintf(" set slip freq %d\n", value);
	}
	else if(strcmp(argv[1], "curr")==0)
	{
		f_val = atoi(argv[2]);
		if(value < 0 || value > 150) goto mtr_err;

		mtr.rated_current = f_val;
		UARTprintf(" set rated current %d\n", (int)f_val);
	}
	else if(strcmp(argv[1], "noload")==0)
	{
		f_val = atoi(argv[2]);
		if(value < 0 || value > 100) goto mtr_err;

		mtr.noload_current = f_val;
		UARTprintf(" set noload current %d\n", (int)f_val);
	}
	else if(strcmp(argv[1], "poles")==0)
	{
		value = atoi(argv[2]);
		if(value < 0 || value > 10) goto mtr_err;

		mtr.poles = value;
		UARTprintf(" set poles %d\n", value);
	}
	else if(strcmp(argv[1], "capa")==0)
	{
		value = atoi(argv[2]);
		if(value < 0 || value > 10) goto mtr_err;

		mtr.capacity = value;
		UARTprintf(" set capacity %d\n", value);
	}
	else if(strcmp(argv[1], "eff")==0)
	{
		value = atoi(argv[2]);
		if(value < 50 || value > 100) goto mtr_err;

		mtr.effectiveness = value;
		UARTprintf(" set effectiveness %d\n", value);
	}
	else if(strcmp(argv[1], "rs")==0)
	{
		f_val = atoi(argv[2]);
		if(value < 0 || value > 28) goto mtr_err;

		mtr.Rs = f_val;
		UARTprintf(" set stator resistance %d\n", (int)f_val);
	}
	else if(strcmp(argv[1], "ls")==0)
	{
		f_val = atoi(argv[2]);
		if(value < 0 || value > 300) goto mtr_err;

		mtr.Ls = f_val;
		UARTprintf(" set stator inductance %d\n", (int)f_val);
	}
	else
		goto mtr_err;

	return 0;

mtr_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_MTR_PARAM].pcHelp);
	return 1;
}
#endif

STATIC int dbg_processTripInfo(int argc, char *argv[])
{
	int cmd;

    if(argc != 2) goto tr_err;

    cmd = argv[1][0];

    if(cmd != 'r' && cmd != 'w' && cmd != 'd') goto tr_err;

    switch(cmd)
    {
    	case 'r':
    		dbg_showTripData();
    		break;

    	case 'd':
    		PARAM_initErrInfo();
    		UARTprintf("clear trip info\n");
    		break;

    	case 'w':
    		ERR_setTripInfo();
    		ERR_setTripFlag(TRIP_REASON_TEST_ERR);
    		UARTprintf("test trip %d happened\n", TRIP_REASON_TEST_ERR);
    		break;
    }

	return 0;

tr_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_TRIP].pcHelp);
	return 1;
}

STATIC int dbg_showInverterStatus(int argc, char *argv[])
{
	if(argc > 1) goto mnt_err;

	dbg_showMonitorParam();

	return 0;

mnt_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_MONITOR].pcHelp);
	return 1;
}

STATIC int dbg_showTempStatus(int argc, char *argv[])
{
	float_t ipm_temp, mtr_temp;
	uint16_t mtr_status;
	if(argc > 1) goto temp_err;

	ipm_temp = UTIL_readIpmTemperature();
	mtr_temp = UTIL_readMotorTemperature();
	mtr_status = UTIL_readMotorTemperatureStatus();
	UARTprintf("IPM temp= %f, adc= %d, Motor Temp= %f, mtr_status= %d\n", ipm_temp, internal_status.ipm_temp, mtr_temp, mtr_status);

	return 0;

temp_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_TEMP].pcHelp);
	return 1;
}

STATIC int dbg_setFanControl(int argc, char *argv[])
{
	int on_off;
	int result=0;
	union32_st data;

    if(argc != 2) goto fan_err;

    on_off = atoi(argv[1]);
	if(on_off != 0 && on_off != 1) goto fan_err;

	data.l = (uint32_t)on_off;
	dbg_setQueCommand(FAN_COMMAND_INDEX, data);
	UARTprintf("set fan control %d\n", on_off);

    return result;

fan_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_FAN].pcHelp);
    return 1;
}


STATIC int dbg_setOverload(int argc, char *argv[])
{
	int value;
	union32_st data;

	if(argc < 2 && argc > 3) goto ovl_err;

	if(argc == 2)
	{
		if(strcmp(argv[1], "show") == 0)
		{
			dbg_showOverloadParam();
			return 0;
		}
		else
			goto ovl_err;
	}

	if(argc == 3)
	{
		value = atoi(argv[2]);
		data.l = (uint32_t)value;
		if(strcmp(argv[1], "en")==0)
		{
			if(value != 0 && value != 1) goto ovl_err;

			dbg_setQueCommand(OVL_ENABLE_INDEX, data);
		}
		else if(strcmp(argv[1], "wl")==0)
		{
			dbg_setQueCommand(OVL_WARN_LIMIT_INDEX, data);
		}
		else if(strcmp(argv[1], "wd")==0)
		{
			dbg_setQueCommand(OVL_WR_DURATION_INDEX, data);
		}
		else if(strcmp(argv[1], "tl")==0)
		{
			dbg_setQueCommand(OVL_TR_LIMIT_INDEX, data);
		}
		else if(strcmp(argv[1], "td")==0)
		{
			dbg_setQueCommand(OVL_TR_DURATION_INDEX, data);
		}
		else
			goto ovl_err;
	}

	return 0;

ovl_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_PROT_OVERLOAD].pcHelp);
	return 1;
}

STATIC int dbg_setRegen(int argc, char *argv[])
{
	uint16_t value;
	union32_st data;

	if(argc < 2 && argc > 3) goto regen_err;

	if(argc == 2)
	{
		if(strcmp(argv[1], "show") == 0)
		{
			dbg_showRegenParam();
			return 0;
		}
		else
			goto regen_err;
	}

	if(argc == 3)
	{
		value = (uint16_t)atoi(argv[2]);
		if(strcmp(argv[1], "duty")==0)
		{
			data.l = (uint32_t)value;
			dbg_setQueCommand(REGEN_DUTY_INDEX, data);
		}
		else if(strcmp(argv[1], "band")==0)
		{
			data.l = (uint32_t)value;
			dbg_setQueCommand(REGEN_BAND_INDEX, data);
		}
		else
			goto regen_err;
	}

	return 0;

regen_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_PROT_REGEN].pcHelp);
	return 1;
}

STATIC int dbg_stepUpFreq(int argc, char *argv[])
{
	union32_st data;
	float_t f_value = iparam[FREQ_VALUE_INDEX].value.f;

    if(argc > 2) goto up_err;

    f_value = f_value + (float_t)freq_step;
    data.f = f_value;
    dbg_setQueCommand(FREQ_VALUE_INDEX, data);
    UARTprintf("set frequency=%f rpm=%d \n", f_value, 30*(int)f_value);
    UARTprintf("resolution acc_res=%f, dec_res=%f\n", m_status.acc_res, m_status.dec_res);

    return 0;

up_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_STEP_UP].pcHelp);
	return 1;
}

STATIC int dbg_stepDownFreq(int argc, char *argv[])
{
	union32_st data;
	float_t f_value = iparam[FREQ_VALUE_INDEX].value.f;

    if(argc > 2) goto down_err;

    f_value = f_value - (float_t)freq_step;
    if(f_value < 3.0)
    {
    	UARTprintf("no more step down freq=%f step=%d\n", f_value, freq_step);
    	goto down_err;
    }
    data.f = f_value;
    dbg_setQueCommand(FREQ_VALUE_INDEX, data);
    UARTprintf("set frequency=%f rpm=%d \n", f_value, 30*(int)f_value);
    UARTprintf("resolution acc_res=%f, dec_res=%f\n", m_status.acc_res, m_status.dec_res);

    return 0;

down_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_STEP_DOWN].pcHelp);
	return 1;
}

STATIC int dbg_setStepFreq(int argc, char *argv[])
{
	int step;

    if(argc > 3) goto step_err;

    if(argc == 1)
    {
    	UARTprintf(" current step is %d\n", freq_step);
    	return 0;
    }

    step = atoi(argv[1]);
	if(step < 1 || step > 10) goto step_err;

	freq_step = step;
	UARTprintf(" new step is %d\n", freq_step);

	return 0;

step_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_STEP].pcHelp);
	return 1;
}

STATIC int dbg_setInitRelay(int argc, char *argv[])
{
	int on_off;

    if(argc > 2) goto irel_err;

    if(argc == 1)
    {
    	UARTprintf(" Init Relay status %d\n", internal_status.relay_enabled);
    	return 0;
    }

    on_off = atoi(argv[1]);
	if(on_off != 0 && on_off != 1) goto irel_err;

	if(on_off == 0) //release shaft brake
	{
		UTIL_clearInitRelay();
		UARTprintf(" clear Init Relay Low %d\n", internal_status.relay_enabled);
	}
	else
	{
		UTIL_setInitRelay();
		UARTprintf(" set Init Relay High %d\n", internal_status.relay_enabled);
	}

    return 0;

irel_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_INIT_RELAY].pcHelp);
    return 1;
}

STATIC int dbg_setShaftBrake(int argc, char *argv[])
{
	int on_off;

    if(argc != 2) goto sbrk_err;

    on_off = atoi(argv[1]);
	if(on_off != 0 && on_off != 1) goto sbrk_err;

	if(on_off == 0) //release shaft brake
	{
		UTIL_releaseShaftBrake();
		UARTprintf("release shaft brake\n");
    }
    else
    {
		UTIL_setShaftBrake();
		UARTprintf("set shaft brake\n");
    }

    return 0;

sbrk_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_SHAFT_BRK].pcHelp);
    return 1;
}

STATIC int dbg_setLed(int argc, char *argv[])
{
	int type, on_off;
	int led_type[] = {HAL_Gpio_LED_R, HAL_Gpio_LED_G};
	char *led_str[] = {"LED_R", "LED_G"};

    if(argc != 3) goto led_err;

    type = atoi(argv[1]);
    on_off = atoi(argv[2]);

    if(type < 0 || type > 1) goto led_err;
	if(on_off != 0 && on_off != 1) goto led_err;

	if(on_off == 1) //LED on
	{
		UTIL_controlLed(led_type[type], on_off);
		UARTprintf(" %s on\n", led_str[type]);
    }
    else
    {
		UTIL_controlLed(led_type[type], on_off);
		UARTprintf(" %s off\n", led_str[type]);
    }

    return 0;

led_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_LED].pcHelp);
    return 1;
}

#ifdef SUPPORT_OFFSET_MEASURE
extern uint16_t gOffsetMeasureFlag;
extern uint16_t ofs_done;
extern float_t ofs_total[6], ofs_value[5];
//extern int OFS_getOffsetState(void);
//extern void OFS_initOffsetState(void);
//extern int OFS_readyOffset(void);
//extern void OFS_startOffset(void);
//extern int OFS_isOffsetDone(void);
extern void OFS_initOffset(void);

STATIC int dbg_measureOffset(int argc, char *argv[])
{
	uint16_t flag=0;

    if(argc != 2) goto ofs_err;

    flag = atoi(argv[1]);

#if 1
    switch(flag)
    {
    case 1:
    	OFS_initOffset();
    	gOffsetMeasureFlag = 1;
    	ofs_done=0;
    	UARTprintf("start offset measure\n");
    	break;

    case 2:
		UARTprintf(" offset total V:%f, %f, %f, I: %f, %f\n", ofs_total[0],ofs_total[1],ofs_total[2],ofs_total[3],ofs_total[4]);
		UARTprintf("V offset %f, %f, %f\n", ofs_value[0],ofs_value[1],ofs_value[2]);
		UARTprintf("I offset %f, %f\n", ofs_value[3],ofs_value[4]);
		break;

    case 0:
    default:
    	UARTprintf("gOffsetMeasureFlag=%d, ofs_done=%d\n", gOffsetMeasureFlag, ofs_done);
    	break;
    }
#else
    if(flag == 0) // status display
    {
    	UARTprintf("OFS status=%d, ofs_done=%d\n", OFS_getOffsetState(), OFS_isOffsetDone());
    }
    else if(flag == 1)
    {
    	if(OFS_readyOffset())
    	{
    		OFS_initOffsetState();
    		OFS_startOffset();
    	}
    	else
    		goto ofs_err;
    }
    else if(flag == 2) // show measured value
    {
    	if(OFS_isOffsetDone())
    	{
			UARTprintf("V offset %f, %f, %f\n", );
			UARTprintf("I offset %f, %f\n", );
    	}
    	else
    	{
    		UARTprintf("OFS not done\n");
    	}
    }
    else
    	goto ofs_err;
#endif


    return 0;

ofs_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_CALC_OFFSET].pcHelp);
    return 1;
}
#endif

STATIC int dbg_showVersion(int argc, char *argv[])
{
    if(argc >= 2) goto ver_err;

    UARTprintf("HW ver %d.%d, FW ver %d.%d\n", HW_VER_MAJ, HW_VER_MIN, DSP_FW_VER_MAJ, DSP_FW_VER_MIN);

    return 0;

ver_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_VERSION].pcHelp);
    return 1;
}


#ifdef SAMPLE_ADC_VALUE
#define V_SAMPLE_COUNT	240

#define I_CURR_SAMPLE_TYPE  	0
#define V_UVW_SAMPLE_TYPE		1
#define V_DC_SAMPLE_TYPE		2
#define V_AB_SAMPLE_TYPE		3
#define PWM_SAMPLE_TYPE			4
#define PWM2_SAMPLE_TYPE		5
#define PHASOR_SAMPLE_TYPE		6
#define V_AB_PHASOR_SAMPLE_TYPE	7
#define PWM_ERR_TYPE			8
#define PWM_PERIOD_COUNT		9
#define I_RMS_SAMPLE_TYPE		10

int sample_type=I_CURR_SAMPLE_TYPE;
int v_count = V_SAMPLE_COUNT;
int sampling_flag=0, stop_sampling_flag=1;
float_t smpl_buff[3][V_SAMPLE_COUNT];
uint16_t smpl_buff_idx=0;
long pwm_err_cnt=0;
float_t pwm_value=0.0, pwm_value_neg=0.0, pwm_value_sat=0.0;

void initSampleBuffer(void)
{
	int i;

	smpl_buff_idx=0;
	for(i=0; i<V_SAMPLE_COUNT; i++)
	{
		smpl_buff[0][i] = 0.0;
		smpl_buff[1][i] = 0.0;
		smpl_buff[2][i] = 0.0;
	}

}

void dbg_getSample(float_t val1, float_t val2, float_t val3)
{
	// for reading V
	if(sampling_flag)
	{
	  smpl_buff[0][smpl_buff_idx] = val1;
	  smpl_buff[1][smpl_buff_idx] = val2;
	  smpl_buff[2][smpl_buff_idx] = val3;

	  smpl_buff_idx++;
	  if(smpl_buff_idx == V_SAMPLE_COUNT)
	  {
		  if(stop_sampling_flag)
		  {
			  sampling_flag=0;
			  UARTprintf(" sampling done\n");
		  }
		  else
			  smpl_buff_idx=0;
	  }
	}
}

STATIC int dbg_getAdcSample(int argc, char *argv[])
{
	uint16_t i, delay;
	uint16_t cmd=0;
	//float_t f_result[3];

    if(argc != 2) goto iadc_err;

    cmd = (int)atoi(argv[1]);

    switch(cmd)
    {
    case 0: // start sampling
    	initSampleBuffer();
    	stop_sampling_flag = 0;
    	sampling_flag=1;
    	UARTprintf(" start sampling type=%d\n", sample_type);
    	break;
    case 1: // stop sampling
    	stop_sampling_flag = 1;
    	UARTprintf(" stop sampling %d\n", smpl_buff_idx);
    	break;
    case 2: // display sample
    	UARTprintf(" sample type %d\n", sample_type);
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    	{
			for(i=0; i<V_SAMPLE_COUNT; i++)
			{
				for(delay=0; delay<1000; delay++);
				UARTprintf(" %f, %f, %f \n", smpl_buff[0][i], smpl_buff[1][i], smpl_buff[2][i]);
			}
			//UARTprintf(" %f, %f\n", f_result[0], f_result[1]);
    	}
    	else
    		UARTprintf(" sampling data not ready  %d\n", smpl_buff_idx);
    	break;

    case 3:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = I_CURR_SAMPLE_TYPE;
    	UARTprintf("set I samples\n");

    	break;

    case 4:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = V_UVW_SAMPLE_TYPE;
   		UARTprintf("set Vuvw samples\n");

    	break;

    case 5:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = V_DC_SAMPLE_TYPE;
   		UARTprintf("set Vdc sample\n");

    	break;

    case 6:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = V_AB_SAMPLE_TYPE;
    	UARTprintf("set Vab samples\n");

    	break;

    case 7:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = PWM_SAMPLE_TYPE;
    	UARTprintf("set PWM samples\n");

    	break;

    case 8:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = PWM2_SAMPLE_TYPE;
    	UARTprintf("set PWM2 samples\n");

    	break;

    case 9:
    	UARTprintf(" sample type %d\n", sample_type);

    	break;

    default:
    	UARTprintf("wrong parameter %d\n", cmd);
       	break;
    }

    return 0;

iadc_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_GET_ADC_I_SAMPLE].pcHelp);
    return 1;
}
#endif


STATIC int dbg_testCmdQueue(int argc, char *argv[])
{
	static uint16_t index=0;
	static uint32_t command=0;
	int type;
	cmd_type_st cmd_data;

    if(argc != 2) goto que_err;

    type = argv[1][0];

    if(type != 'r' && type != 'w') goto que_err;

	switch(type)
	{
	case 'r':
		if(!QUE_isEmpty())
		{
			cmd_data = QUE_getCmd();
			UARTprintf(" QUE read cmd=%d, index=%d, cmd=%d\n", cmd_data.cmd, cmd_data.index, (int)cmd_data.data.l);
		}
		else
			UARTprintf(" QUE empty!\n");

		break;

	case 'w':
		if(!QUE_isFull())
		{
			cmd_data.cmd = index;
			cmd_data.index=index++;
			cmd_data.data.l=command++;
			QUE_putCmd(cmd_data);
			UARTprintf(" QUE write cmd=%d, index=%d cmd=%d\n", cmd_data.cmd, cmd_data.index, (int)cmd_data.data.l);
		}
		else
			UARTprintf(" QUE full!\n");

		break;
	}

    return 0;

que_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_QUE_TEST].pcHelp);
    return 1;
}

#ifdef UNIT_TEST_ENABLED
STATIC int dbg_UnitTest(int argc, char *argv[])
{
	if(MAIN_isSystemEnabled())
	{
		UARTprintf("Unit Test is not available during running motor! \n");
		return 0;
	}

    UARTprintf("--Unit Test Running\n");

	UNITY_BEGIN();

	unit_test_running = 1;

	// for speed setting, not used
	//RUN_TEST(test_setSpeedParam);
	//RUN_TEST(test_setAccelTime); //test_speed.c
	//RUN_TEST(test_processSpeedScaling); //test_resolution.c
	//RUN_TEST(test_processResolution);

	RUN_TEST(test_setFreqParam); //test_freq.c

	RUN_TEST(test_processConvertFreq); //test_resolution.c

	RUN_TEST(test_processResolutionTargetFreq);

	RUN_TEST(test_setDciBrakeParam); //test_dci_brake.c

	RUN_TEST(test_setOverload); // test_protect.c
	//RUN_TEST(test_processDcVoltage); not ready

	RUN_TEST(test_controlState); //test_state.c

	RUN_TEST(test_controlDrive); //test_drive.c


//	RUN_TEST(test_errorTrip); // test_trip.c

	UNITY_END();

	UARTprintf(" End of Unit Test!\n Please Reset Device!!\n");

	//UARTFlushTx(0);
	while(1);

	return 0;
}
#endif

//extern float MAIN_convert2InternalSpeedRef(int freq);
//extern int MAIN_convert2Speed(float speed);
//extern void initParam(void);

#ifdef PWM_DUTY_TEST
extern uint16_t gFlag_PwmTest;
extern _iq gPwmData_Value;
uint16_t gFlag_isFocPwm=0;
uint16_t gFlag_isOffserPwm=0;
extern uint16_t gFlagDCIBrake;
extern void dbg_enableSystem(void);
extern void dbg_disableSystem(void);
#endif

STATIC int dbg_tmpTest(int argc, char *argv[])
{
	int i, s_value, index;
//	int value;
//	_iq iq_val;
//	float f_val;

    if(argc > 3) goto tmp_err;

    index = atoi(argv[1]);
    if(index == 0)
    	index = argv[1][0];

    if(index == '0')
    {
        for(i=0; i<400; i+=30)
        {
        	s_value = FREQ_convertToSpeed(i);
        	UARTprintf("value = freq=%d -> rpm=%d\n", i, s_value);
        }
    }
    else if(index == 1)
    {
    	iparam[OVL_TR_LIMIT_INDEX].value.l = 110;
    	iparam[OVL_TR_DURATION_INDEX].value.l = 10;
    	OVL_enbleOverloadTrip(1);
    	UARTprintf("OVL start = %d\n", (int)secCnt);
    }
    else if(index == 2)
    {
    	iparam[BRK_DCI_START_FREQ_INDEX].value.f = 20.0;
    	iparam[BRK_DCI_BLOCK_TIME_INDEX].value.f = 2.0;
    	iparam[BRK_DCI_BRAKING_TIME_INDEX].value.f = 10.0;
    	// enable DCI
    	UARTprintf("DCIB start = %d\n", (int)secCnt);
    }
    else if(index == 3) // check sizeof(int) -> 2byte
    {
//    	unsigned long b = 0xFFFFFFFE;
//    	unsigned int a=0xFFFE;
//    	UARTprintf("a=%d b=%d \n", (unsigned int)(a+1), (unsigned int)(a+2));

    	// float, long -> word array : using union
    	uint32_t i_data=0x10001;
    	float_t f_data=3.14;
    	union_fdata fdata, fdata2;
    	union_ldata ldata, ldata2;


    	ldata.l_data = i_data;
    	ldata2.i_word[0] = ldata.i_word[0];
    	ldata2.i_word[1] = ldata.i_word[1];

    	fdata.f_data = f_data;
    	fdata2.i_word[0] = fdata.i_word[0];
    	fdata2.i_word[1] = fdata.i_word[1];
    	UARTprintf("ldata %l, fdata %f\n", (uint32_t)ldata2.l_data, fdata2.f_data);
    }
    else if(index == 4) //
    {
    	//initParam();
    	PARAM_init();
    	UARTprintf(" Initialize parameters! \n");

    }
    else if(index == 5)
    {
//    	float_t f_val=3.14;
//    	UARTprintf("float test %f, sizeof(float)=%d\n", f_val, (int)sizeof(float));
//    	UARTprintf("sizeof(long)=%d sizeof(int)=%d, sizeof(char)=%d\n", (int)sizeof(long), (int)sizeof(int), (int)sizeof(char));
//    	uint16_t period_cycles = (uint16_t)(90.0*1000.0); // overflow
//    	UARTprintf("period = %d\n", period_cycles);
#ifdef SUPPORT_MOTOR_PARAM
    	UARTprintf("input_voltage = %d, trip=%d\n", mtr_param.voltage_in, internal_status.trip_happened);
#else
    	UARTprintf("input_voltage = %d, trip=%d\n", USER_MOTOR_VOLTAGE_IN, internal_status.trip_happened);
#endif
    }
    else if(index == 6)
    {
    	uint16_t size;

    	size = PARAM_getInvStatus(&invbuf[0]);
    	UARTprintf(" INV Status : 0x%x, 0x%x, 0x%x, 0x%x: 0x%x, 0x%x, 0x%x, 0x%x: 0x%x, 0x%x, 0x%x, 0x%x \n", \
    	           invbuf[0], invbuf[1], invbuf[2], invbuf[3], invbuf[4], invbuf[5], invbuf[6], invbuf[7], invbuf[8], invbuf[9], invbuf[10], invbuf[11]);

    	size = PARAM_getErrorInfo(&errbuf[0]);
    	UARTprintf(" Err Status : 0x%x, 0x%x, 0x%x, 0x%x: 0x%x, 0x%x\n",\
    	           errbuf[0], errbuf[1], errbuf[2], errbuf[3], errbuf[4], errbuf[5]);

    	size = PARAM_getValue(0x100, (uint16_t *)&rparam[0]);
    	UARTprintf(" Err Status : 0x%x, 0x%x\n", rparam[0], rparam[1]);
    }
    else if(index == 7)
    {
//    	uint16_t   pwm_4k = (uint16_t)(gUserParams.systemFreq_MHz*(1000.0/4.0)) >> 1;
//    	uint16_t   pwm_8k = (uint16_t)(gUserParams.systemFreq_MHz*(1000.0/8.0)) >> 1;
//    	uint16_t   pwm_12k = (uint16_t)(gUserParams.systemFreq_MHz*(1000.0/12.0)) >> 1;
//    	uint16_t   pwm_16k = (uint16_t)(gUserParams.systemFreq_MHz*(1000.0/16.0)) >> 1;

//    	UARTprintf("current control %d, pwm=%d kHz \n", (int)iparam[VF_FOC_SEL_INDEX].value.l, (int)DRV_getPwmFrequency());
    	//UARTprintf(" PWM period 4k:%d, 8k:%d 12k:%d, 16k:%d\n", pwm_4k, pwm_8k, pwm_12k, pwm_16k);
    	UARTprintf(" pwm=%d kHz \n", (int)DRV_getPwmFrequency());
    	UARTprintf(" I offset %f, %f, %f \n", _IQtoF(i_offset[0]), _IQtoF(i_offset[1]), _IQtoF(i_offset[2]));
    	UARTprintf(" V offset %f, %f, %f \n", _IQtoF(v_offset[0]), _IQtoF(v_offset[1]), _IQtoF(v_offset[2]));
    	UARTprintf(" Vs %f A, VsRef=%f A \n", _IQtoF(gMotorVars.Vs), _IQtoF(gMotorVars.VsRef));

    }
    else if(index == 8)
    {
    	//UTIL_controlLed(HAL_Gpio_LED_R2, 1);
    	//UTIL_controlLed(HAL_Gpio_LED_G, 1);
    	UTIL_setFanOff();
    	UARTprintf(" Fan off %d, %d\n", internal_status.fan_enabled, (int)iparam[FAN_COMMAND_INDEX].value.l);
    }
    else if(index == 9)
    {
    	UTIL_setFanOn();
    	UARTprintf(" Fan on %d, %d\n", internal_status.fan_enabled, (int)iparam[FAN_COMMAND_INDEX].value.l);
    }
    else if(index == 'g')
    {
    	UARTprintf(" Trip happened %d, Mcu_comm %d \n", internal_status.trip_happened, (int)UTIL_getCommStatus());
    }
    else if(index == 'i')
    {
    	int i;
    	extern float_t array_Iu[];
    	UARTprintf(" Irms =%f \n", MAIN_getIave());
    	for(i=0; i<I_RMS_SAMPLE_COUNT; i++)
    		UARTprintf(" int_Iu %f \n", array_Iu[i]);
    }
    else if(index == 'r')
    {
    	regen_duty = atoi(argv[2]);
    	UARTprintf(" set regen duty = %d \n", regen_duty);
    }
    else if(index == 'f')
    {
    	int duty;
    	duty = atoi(argv[2]);
    	UTIL_setRegenPwmDuty(duty);

    	UARTprintf(" set REGEN pwm duty=%d\n", duty);
    }
#ifdef SUPPORT_INIT_MAGNETIZE
    else if(index == 'q')
    {
    	int rate;
    	rate = (uint16_t)atoi(argv[2]);

    	switch(rate)
    	{
    	case 2: magnetize_rate = 0.01; break;
    	case 3: magnetize_rate = 0.02; break;
    	case 4: magnetize_rate = 0.03; break;
    	case 5: magnetize_rate = 0.04; break;
    	case 1:
    	default: magnetize_rate = 0.01; break;
    	}

    	UARTprintf(" set magnetize rate=%f\n", magnetize_rate);
    }
#endif
#ifdef SUPPORT_MISS_PHASE_DETECT
	else if(index == 'a')
	{
		int i, delay;

		// print measured I value for missing I phase
		for(i=0; i<I_MISS_SAMPLE_COUNT; i++)
		{
			for(delay=0; delay<1000; delay++);
			UARTprintf(" %f, %f, %f \n", i_buff[0][i], i_buff[1][i], i_buff[2][i]);
		}
		UARTprintf("low cnt: %d, %d, %d \n", u_low_cnt, v_low_cnt, w_low_cnt);
		UARTprintf("hi  cnt: %d, %d, %d \n", u_high_cnt, v_high_cnt, w_high_cnt);
	}
#endif
#ifdef SUPPORT_SPI_ACCELEROMETER
    else if(index == 'a')
    {
    	int ret=0;
    	uint16_t data[2]= {0,0};

    	ret = SPI_readSensor(0x20, data);
    	UARTprintf(" accel ret=%d, data0=%d, data1=%d \n", ret, data[0], data[1]);
    }
    else if(index == 'b')
    {
    	int ret=0;
    	uint16_t data[2]= {0x20,0xA5};

    	ret = SPI_writeSesnsor(data);
    	UARTprintf(" accel write, ret=%d, data0=%d, data1=%d \n", ret, data[0], data[1]);
    }
    else if(index == 'c') // read who am i register -> 51
    {
    	int ret=0;
    	uint16_t data[2]= {0,0};

    	ret = SPI_readSensor(0xF, data);
    	UARTprintf(" accel read 51, ret=%d, data0=%d, data1=%d \n", ret, data[0], data[1]);
    }
#endif
    else if(index == 'k')
    {
    	//_iq Kp, Ki;
  	    _iq spd_Kp, spd_Ki, spd_Kd;
  	    _iq id_Kp, id_Ki, id_Kd;
    	_iq iq_Kp, iq_Ki, iq_Kd;

    	MAIN_showPidGain();
//   	    Kp = _IQ(0.035*gUserParams.maxCurrent*gUserParams.iqFullScaleFreq_Hz/gUserParams.iqFullScaleCurrent_A);
//    	Ki = _IQ(2.5*gUserParams.maxCurrent*gUserParams.iqFullScaleFreq_Hz*gUserParams.ctrlPeriod_sec/gUserParams.iqFullScaleCurrent_A);
//    	UARTprintf(" Ki=%f, Kp=%f \n", (float_t)_IQtoF(Ki), (float_t)_IQtoF(Kp));

    	CTRL_getGains(ctrlHandle,CTRL_Type_PID_spd, &spd_Kp, &spd_Ki, &spd_Kd);
    	UARTprintf("SPD Kp=%f, Ki=%f, Kd=%f\n", (float_t)_IQtoF(spd_Kp), (float_t)_IQtoF(spd_Ki), (float_t)_IQtoF(spd_Kd));

   	    CTRL_getGains(ctrlHandle,CTRL_Type_PID_Id, &id_Kp, &id_Ki, &id_Kd);
   	    UARTprintf("Id Kp=%f, Ki=%f, Kd=%f\n", (float_t)_IQtoF(id_Kp), (float_t)_IQtoF(id_Ki), (float_t)_IQtoF(id_Kd));

   	    CTRL_getGains(ctrlHandle,CTRL_Type_PID_Iq, &iq_Kp, &iq_Ki, &iq_Kd);
   	    UARTprintf("Iq Kp=%f, Ki=%f, Kd=%f\n", (float_t)_IQtoF(iq_Kp), (float_t)_IQtoF(iq_Ki), (float_t)_IQtoF(iq_Kd));

    }
    else if(index == 'v')
    {
    	//UARTprintf("OVL warn enable=%d\n", ovl_alarm_enable);
    	UARTprintf(" ADC Vu=%d Vv=%d, Vw=%d\n", Vinst[0], Vinst[1], Vinst[2]);
    	UARTprintf(" V value Vu=%f Vv=%f, Vw=%f\n", internal_status.Vu_inst, internal_status.Vv_inst, internal_status.Vw_inst);
    }
    else if(index == 't')
    {
    	int ms_dur;
    	float_t f_dur;

    	ms_dur = atoi(argv[2]);
    	f_dur = (float_t)ms_dur;
    	TMR_startTimerSig(TIMER_TEST_TSIG, f_dur);
    	UARTprintf(" Test Timer %f sec start at %d \n", f_dur, (int)secCnt);
    }
    else if(index == 'x')
    {
    	uint16_t on_off;

    	on_off = (uint16_t)atoi(argv[2]);

    	MAIN_setSpeedGain(on_off);
    	UARTprintf(" set Ki_spd gain for %d\n", on_off);
    }
    else if(index == 'u')
    {
    	uint16_t value;

    	value = (uint16_t)atoi(argv[2]);

    	UTIL_setNotifyFlagMcu(value);
    	UARTprintf(" set MCU notify %d\n", value);
    }
    else if(index == 'l') // enable/disable log
    {
    	int enable=0;

    	if(argc != 3)
    	{
    		UARTprintf(" enable/disable log %d\n", gFlag_LogEnabled);
    		return 0;
    	}

    	enable = atoi(argv[2]);
    	if(enable == 1)
    	{
    		UARTprintf(" enable log\n");
    		gFlag_LogEnabled = 1;
    	}
    	else
    	{
    		UARTprintf(" disable log\n");
    		gFlag_LogEnabled = 0;
    	}
    }
#ifdef SUPPORT_AUTO_LOAD_TEST_
    else if(index == 'l')
    {
        bool sw_state=0;
        int btn_state=0;
        sw_state = UTIL_readSwGpio();
        btn_state = TEST_readSwitch();
        UARTprintf(" test SW %d btn %d\n", (int)sw_state, (int)btn_state);
    }
    else if(index == 'i')
    {
    	int enable;

    	enable = atoi(argv[2]);
    	if(enable)
    		ipm_disp_on = 1;
    	else
    		ipm_disp_on = 0;

    	UARTprintf(" IPM temperature periodic display enable=%d\n", ipm_disp_on);
    }
#endif

#ifdef PWM_DUTY_TEST
    else if(index == 'p')
    {
    	int enable=0;

    	if(argc != 3)
    	{
    		UARTprintf(" PWM test %d\n", gFlag_PwmTest);
    		return 0;
    	}

    	enable = atoi(argv[2]);
    	if(enable == 1)
    	{
			dbg_enableSystem();
			if(DRV_isFocControl()) // need VF for constant PWM duty test
			{
				gFlag_isFocPwm=1;
				DRV_enableVfControl();
			}
			if(gMotorVars.Flag_enableOffsetcalc) // need disable offset recalibration for PWM duty test
			{
				gFlag_isOffserPwm=1;
				gMotorVars.Flag_enableOffsetcalc=false;
			}
			gFlag_PwmTest = true;
			UARTprintf(" enable PWM test \n");
    	}
    	else
    	{
			dbg_disableSystem();
			gFlag_PwmTest = false;

			if(gFlag_isFocPwm) { DRV_enableFocControl(); gFlag_isFocPwm=0;}

			if(gFlag_isOffserPwm) {gMotorVars.Flag_enableOffsetcalc=true; gFlag_isOffserPwm=0;}

			UARTprintf(" disable PWM test \n");
    	}
    }
    else if(index == 'm')
    {
    	int pwm_data=50;
    	float_t pwm_f=0.0;

    	if(argc != 3)
    	{
    		UARTprintf(" Pwm data %d -> %f \n", pwm_data, pwm_f);
    		return 0;
    	}

    	pwm_data = atoi(argv[2]);
    	if(pwm_data >= 0 && pwm_data <= 100) // input duty as 0 ~ 100%
    	{
    		pwm_f = (float_t)(pwm_data-50)/100.0; // change -0.5 ~ 0.5
    		gPwmData_Value = _IQ(pwm_f);
    		UARTprintf(" Pwm data %d -> %f \n", pwm_data, pwm_f);
    	}
    	else
    		UARTprintf(" Pwm data error %d\n", pwm_data);

    }
#endif
    else if(index == 'j')
    {
    	int enable=0;

    	if(argc != 3)
    	{
    		UARTprintf(" PWM DC inject test %d, need >tmp p 1\n", gFlagDCIBrake);
    		return 0;
    	}

    	enable = atoi(argv[2]);
    	if(enable == 1)
    	{
			gFlagDCIBrake = true;
			dbg_enableSystem();
			UARTprintf(" enable PWM DC inject test \n");
    	}
    	else
    	{
    		dbg_disableSystem();
			gFlagDCIBrake = false;
			UARTprintf(" disable PWM DC inject test \n");
    	}
    }
    else if(index == 'd')
    {
    	int16_t cur_rate=0;
    	float_t V_duty=0.0; //pwm_f=0.0,

    	if(argc != 3)
    	{
    		UARTprintf(" set DCI Pwm data, need Current rate \n");
    		return 0;
    	}

    	cur_rate = (int16_t)atoi(argv[2]);
    	if(cur_rate >= 0 && cur_rate <= 200) // input duty as 0 ~ 200%
    	{
    		MPARAM_setDciPwmRate((float_t)cur_rate); //dev_const.dci_pwm_rate
    		V_duty = dev_const.dci_pwm_rate*100.0/MAIN_getVdcBus();
//    		pwm_f = (float_t)(cur_rate)/100.0 * USER_MOTOR_MAX_CURRENT*USER_MOTOR_Rs*2;
//    		V_duty = pwm_f*100.0 / MAIN_getVdcBus();
    		gPwmData_Value = _IQ(V_duty/100.0);
    		UARTprintf(" DCI Pwm data %d -> V_percent=%f\n", cur_rate, V_duty/100.0);
    		MPARAM_setDciPwmRate(iparam[BRK_DCI_BRAKING_RATE_INDEX].value.f); // restore dev_const.dci_pwm_rate
    	}
    	else
    		UARTprintf(" DCI Pwm a error %d\n", cur_rate);
    }

#ifdef SAMPLE_ADC_VALUE
    else if(index == 'z')
    {
    	UARTprintf(" PWM zero cnt=%d, val=%f neg=%f sat=%f \n", pwm_err_cnt, pwm_value, pwm_value_neg, pwm_value_sat);
    }
#endif



    return 0;

tmp_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_TEST].pcHelp);
    return 1;
}
#endif


void ProcessDebugCommand(void)
{
    //int size = 0;
    //int cnt = 0, cmdCnt;
    int ret;

    if(g_bNewCmd) {
        UARTgets((char*)cmdString, (unsigned long)NUM_OF_DEBUGCHAR);
#ifdef SUPPORT_DEBUG_TERMINAL
        {
            ret = CmdLineProcess((char*)cmdString);
            if(ret==CMDLINE_BAD_CMD)
            	UARTprintf("Invalid command.\n");

            UARTprintf("debug>");
        }
#endif
        g_bNewCmd = false;
    }
}

void dbg_logo(void)
{
	//UARTprintf("\n*****************************************************************");
	//UARTprintf("\n**%14sCompiled :    %4d/%02d/%02d   %10s %9s**", " ", BUILD_YEAR, BUILD_MONTH, BUILD_DAY, __TIME__, " ");
	//UARTprintf("\n**%15sCopyright(C) Nara Control Co., Ltd.%11s**", " ", " ");
	//UARTprintf("\n**        Motor control debug program for TMS320F28069M        **\n");
	UARTprintf("\n** %s Compiled :    %4d/%02d/%02d   %10s **", " ", BUILD_YEAR, BUILD_MONTH, BUILD_DAY, __TIME__);
	UARTprintf("\n");
}

int EchoSetting(int argc, char *argv[])
{
    if(strcmp("on", argv[1]) == 0)
        UARTEchoSet(true);
    else if(strcmp("off", argv[1]) == 0)
        UARTEchoSet(false);
    else
        UARTprintf("echo <on/off>\n");

    return 0;
}


//#endif
