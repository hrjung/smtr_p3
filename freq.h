/*
 * freq.h
 *
 *  Created on: 2017. 3. 11.
 *      Author: hrjung
 */

#ifndef FREQ_H_
#define FREQ_H_


/*******************************************************************************
 * CONSTANTS
 */
#define 	MAX_FREQ_VALUE	(200.0)
#define		MIN_FREQ_VALUE	(0.9)

#define 	ACCEL	0
#define		DECEL	1

#define FREQ_INPUT_RESOLUTION	(10.0)
//#define FREQ_INPUT_RESOLUTION	(100.0)

//#define FREQ_isJumpFreqUsed(index)	(param.ctrl.jump[index].enable == 1)
/*******************************************************************************
 * EXTERNS
 */
extern int FREQ_isValid(float_t value);
extern int Freq_isInWorkingFreqRange(float_t value);

extern int FREQ_setFreqValue(float_t value);
#ifdef SUPPORT_ACCEL_TIME_BASE
extern int FREQ_setMaxFreqValue(float_t value);
extern float_t FREQ_getMaxFreqValue(void);
#endif

extern void FREQ_updateJumpSpeed(void);
extern int FREQ_setJumpFreqEnable(uint16_t index, uint16_t enable);
extern int FREQ_setJumpFreqLow(uint16_t index, float_t low);
extern int FREQ_setJumpFreqHigh(uint16_t index, float_t high);
extern int FREQ_clearJumpFreq(uint16_t index);
extern int FREQ_setJumpFreqRange(int index, float_t low, float_t high);

// valid frequency to set at last after checking range, jump freq.
extern float_t FREQ_getVarifiedFreq(float_t current, float_t target);

extern float_t FREQ_convertToSpeed(float_t freq);
//extern int FREQ_showUserSpeed(float_t rpm);

//extern int FREQ_convertAinToFreq(int type, int adc_value);

#endif /* FREQ_H_ */
