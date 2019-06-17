/*
 * err_trip.c
 *
 *  Created on: 2017. 7. 27.
 *      Author: hrjung
 */
#include "stdint.h"

#include "uartstdio.h"
//#include "nv_param.h"
#include "parameters.h"
#include "inv_param.h"
//#include "drive.h"
#include "state_func.h"
#include "common_tools.h"
#include "err_trip.h"


#ifdef FLASH
#pragma CODE_SECTION(ERR_setTripInfo,"ramfuncs");
#pragma CODE_SECTION(ERR_setTripFlag,"ramfuncs");
#endif
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


/*******************************************************************************
 * LOCAL FUNCTIONS
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * EXTERNS
 */
extern float_t temp_spd_ref;
extern MOTOR_working_st m_status;

extern float_t MAIN_getVdcBus(void);
extern float_t MAIN_getIave(void);
extern float_t UTIL_readIpmTemperature(void);
extern uint16_t UTIL_readMotorTemperatureStatus(void);
/*
 *  ======== local function ========
 */


/*
 *  ======== public function ========
 */
trip_info_st trip_info;


void ERR_printTripInfo(void)
{
	UARTprintf("Iu_inst: %f, Iv_inst: %f, Iw_inst: %f \n", trip_info.Iu_inst,trip_info.Iv_inst,trip_info.Iw_inst);
	UARTprintf("Irms: %f, Vdc: %f, Vin: %d \n", trip_info.Irms,trip_info.Vdc_inst,trip_info.V_input);
	UARTprintf("ipm_temp: %f, mtr_temp: %d, temp_ref: %f, ov_accel: %f, ov_freq: %f \n", \
			trip_info.ipm_temp,(int)trip_info.motor_temp,trip_info.temp_spd_ref,trip_info.over_accel,trip_info.over_freq);
}

void ERR_initTripInfo(void)
{
	trip_info.Iu_inst = 0.0;
	trip_info.Iv_inst = 0.0;
	trip_info.Iw_inst = 0.0;

	trip_info.Irms = 0.0;
	trip_info.Vdc_inst = 0.0;
	trip_info.V_input = 0;

	trip_info.ipm_temp = 0.0;
	trip_info.motor_temp = 0;

	trip_info.temp_spd_ref = 0.0;
	trip_info.over_accel = 0.0;
	trip_info.over_freq = 0.0;
}

void ERR_setTripInfo(void)
{
	trip_info.Iu_inst = internal_status.Iu_inst;
	trip_info.Iv_inst = internal_status.Iv_inst;
	trip_info.Iw_inst = internal_status.Iw_inst;

	trip_info.Irms = MAIN_getIave();

	trip_info.Vdc_inst = MAIN_getVdcBus();
	trip_info.V_input = (int)USER_MOTOR_VOLTAGE_IN;

	trip_info.ipm_temp = UTIL_readIpmTemperature();
	trip_info.motor_temp = UTIL_readMotorTemperatureStatus();

	trip_info.temp_spd_ref = temp_spd_ref;
	trip_info.over_accel = 0.0;
	trip_info.over_freq = m_status.cur_freq;
}

void ERR_setTripFlag(uint16_t cause)
{
	if(internal_status.trip_happened != cause) // avoid duplicated
	{
		internal_status.trip_happened = cause;
		PARAM_setErrInfo(cause, (uint16_t)m_status.status, m_status.current, m_status.cur_freq);
		UTIL_setNotifyFlagMcu(MCU_COMM_ERROR_NOTI);
	}
}

