/*
 * motor_param.h
 *
 *  Created on: 2017. 8. 22.
 *      Author: hrjung
 */

#ifndef MOTOR_PARAM_H
#define MOTOR_PARAM_H

#include "stdint.h"
#include "userParams.h"

/*******************************************************************************
 * CONSTANTS
 */

// motor type
#define	MOTOR_TEST_0_25k_TYPE		0
#define MOTOR_SY_1_5K_TYPE        	1
#define MOTOR_SY_1_5K_IE3_TYPE     	2
#define MOTOR_SY_2_2K_TYPE        	3

#define MOTOR_TYPE_MAX				4


typedef struct
{
	uint16_t	pole_pairs;
	uint16_t 	rated_freq;
	uint16_t 	voltage_in;

	float_t		noload_current;
	float_t		max_current;
	float_t 	Rs;
	float_t		Rr;
	float_t		Ls;

	float_t 	magnetize_current;
	float_t		rated_flux;
} motor_param_st ;


extern void MPARAM_init(uint16_t type);
extern void MPARAM_setMotorParam(USER_Params *pUserParams);
extern void MPARAM_updateDevConst(void);

extern void MPARAM_setDciPwmRate(float_t rate);
extern void MPARAM_setOvlTripLevel(uint32_t level);
extern void MPARAM_setOvlWarnLevel(uint32_t level);

extern float_t FREQ_convertToSpeed(float_t freq);


#endif /* MOTOR_PARAM_H */
