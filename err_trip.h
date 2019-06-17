/*
 * err_trip.h
 *
 *  Created on: 2017. 7. 27.
 *      Author: hrjung
 */

#ifndef ERR_TRIP_H_
#define ERR_TRIP_H_


/*******************************************************************************
 * CONSTANTS
 */

typedef struct
{
	float_t 	Iu_inst;
	float_t 	Iv_inst;
	float_t 	Iw_inst;

	float_t		Irms;
	float_t 	Vdc_inst;
	uint16_t	V_input;

	float_t 	ipm_temp;
	uint16_t 	motor_temp;

	float_t 	temp_spd_ref;
	float_t 	over_accel;
	float_t		over_freq;

} trip_info_st;

/*******************************************************************************
 * EXTERNS
 */
extern trip_info_st trip_info;


extern void ERR_initTripInfo(void);
extern void ERR_setTripInfo(void);
extern void ERR_setTripFlag(uint16_t cause);


#endif /* ERR_TRIP_H_ */
