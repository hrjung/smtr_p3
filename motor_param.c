//###########################################################################
//
// FILE:   motor_param.c
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
#include "parameters.h"
#include "motor_param.h"


//*****************************************************************************
//
//! \addtogroup
//! @{
//
//*****************************************************************************
#ifdef SUPPORT_MOTOR_PARAM
#define PWM_DEADBAND_LIMITATION  (0.93)


const motor_param_st SY_0_8k = {
    2,      //pole_pairs;
    60,     //rated_freq;
    380,    //voltage_in;

    1.2,    //noload_current;
    1.9,    // rated current
    6.5,    //max_current;
    7.7107, //Rs;
    4.0901, //Rr;
    0.062048, //Ls;

    (0.89*1.4142),   //USER_MOTOR_MAGNETIZING_CURRENT;
    4.94542551,   //USER_MOTOR_RATED_FLUX;
};

const motor_param_st SY_1_5k = {
	2,		//pole_pairs;
	60, 	//rated_freq;
	380, 	//voltage_in;

	2.0, 	//noload_current;
	3.4,    // rated current
	10.0, 	//max_current;
	2.5, 	//Rs;
	2.14568, //Rr;
	0.013955, //Ls;

	(2.0*104142), //USER_MOTOR_MAGNETIZING_CURRENT;
	4.923143, 	//USER_MOTOR_RATED_FLUX;
};

const motor_param_st SY_1_5k_ie3 = {
	2,		//pole_pairs;
	60, 	//rated_freq;
	380, 	//voltage_in;

	2.0, 	//noload_current;
	3.6, 	//rated current;
	10.0,   //max_current;
	2.1693, 	//Rs;
	2.1915, //Rr;
	0.013229, //Ls;

	2.828427, 	//magnetize_current;
	4.936045, 	//rated_flux;
};

const motor_param_st SY_2_2k = {
	2,		//pole_pairs;
	60, 	//rated_freq;
	380, 	//voltage_in;

	3.235, 	//noload_current;
	5.3, 	// rated current
	18.7,   //max_current;
	2.86, 	//Rs;
	1.14793, //Rr;
	0.01092, //Ls;

	4.575, 	//magnetize_current;
	4.857245, 	//rated_flux;
};


motor_param_st mtr_param;
#endif

extern float foc_end_rpm;
//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************

void MPARAM_setDciPwmRate(float_t rate)
{
#ifdef SUPPORT_MOTOR_PARAM
	dev_const.dci_pwm_rate = rate/100.0 * mtr_param.rated_current*mtr_param.Rs;
#else
	//dev_const.dci_pwm_rate = rate/100.0 * USER_MOTOR_MAX_CURRENT/sqrtf(2.0)*USER_MOTOR_Rs*2; //*2 for Y connection
	dev_const.dci_pwm_rate = rate/100.0 * USER_MOTOR_RATED_CURRENT*USER_MOTOR_Rs*2;
#endif
}

void MPARAM_setOvlTripLevel(uint32_t level)
{
#ifdef SUPPORT_MOTOR_PARAM
	dev_const.trip_level = mtr_param.rated_current*(float_t)level/100.0;
#else
	dev_const.trip_level = USER_MOTOR_RATED_CURRENT*(float_t)level/100.0;
#endif
}

void MPARAM_setOvlWarnLevel(uint32_t level)
{
#ifdef SUPPORT_MOTOR_PARAM
	dev_const.warn_level = mtr_param.rated_current*(float_t)level/100.0;
#else
	dev_const.warn_level = USER_MOTOR_RATED_CURRENT*(float_t)level/100.0;
#endif
}

#ifdef SUPPORT_MOTOR_PARAM
void MPARAM_init(uint16_t type)
{
	switch(type)
	{
	case MOTOR_SY_0_8K_TYPE: mtr_param = SY_0_8k; foc_end_rpm = 0.015; break;
	case MOTOR_SY_1_5K_TYPE: mtr_param = SY_1_5k; foc_end_rpm = 0.015; break;
	case MOTOR_SY_2_2K_TYPE: mtr_param = SY_2_2k; foc_end_rpm = 0.03; break;

	case MOTOR_SY_1_5K_IE3_TYPE:
	default:
		mtr_param = SY_1_5k_ie3; foc_end_rpm = 0.015;
		break;
	}
}


void MPARAM_setMotorParam(USER_Params *pUserParams)
{
	pUserParams->motor_type = USER_MOTOR_TYPE;
	pUserParams->motor_numPolePairs = mtr_param.pole_pairs;
	pUserParams->motor_ratedFlux = mtr_param.rated_flux;
	pUserParams->motor_Rr = mtr_param.Rr;
	pUserParams->motor_Rs = mtr_param.Rs;
	pUserParams->motor_Ls_d = mtr_param.Ls;
	pUserParams->motor_Ls_q = mtr_param.Ls;

	pUserParams->maxCurrent = mtr_param.max_current;
	pUserParams->IdRated = mtr_param.magnetize_current;

	pUserParams->maxNegativeIdCurrent_a = (-0.5 * pUserParams->maxCurrent);

	pUserParams->VF_volt_max = ((mtr_param.voltage_in*1.414)/1.732051)*PWM_DEADBAND_LIMITATION;
	pUserParams->VF_volt_min = (pUserParams->VF_volt_max*(USER_MOTOR_FREQ_LOW/USER_MOTOR_FREQ_HIGH));
}
#endif

void MPARAM_updateDevConst(void)
{
	MPARAM_setOvlTripLevel(iparam[OVL_TR_LIMIT_INDEX].value.l);
	MPARAM_setOvlWarnLevel(iparam[OVL_WARN_LIMIT_INDEX].value.l);
#ifdef SUPPORT_MOTOR_PARAM
	dev_const.ovc_level = mtr_param.rated_current*3.5;
#else
	dev_const.ovc_level = USER_MOTOR_RATED_CURRENT*3.5;
	//dev_const.ovc_level = 2.1; //USER_MOTOR_RATED_CURRENT*0.5; // test only
#endif
	MPARAM_setDciPwmRate(iparam[BRK_DCI_BRAKING_RATE_INDEX].value.f);

}

float_t FREQ_convertToSpeed(float_t freq)
{
#ifdef SUPPORT_MOTOR_PARAM
    float_t spd_rpm = (freq*mtr_param.rated_freq / mtr_param.pole_pairs);
#else
	float_t spd_rpm = (freq*60.0 /USER_MOTOR_NUM_POLE_PAIRS);
#endif

    return spd_rpm;
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


