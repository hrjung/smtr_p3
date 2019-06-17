/*
 * test_freq.c
 *
 *  Created on: 2017. 3. 10.
 *      Author: hrjung
 */

#ifdef UNIT_TEST_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "unity.h"
#include "../inv_param.h"
#include "../parameters.h"
#include "../freq.h"


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
extern dev_const_st	dev_const;

/*******************************************************************************
 * EXTERNS
 */
extern void PARAM_init(void);

/*
 *  ======== function ========
 */

void setUp(void) {
// set stuff up here
	PARAM_init();
}
void tearDown(void) {
// clean stuff up here
	PARAM_init();
}

/*
 * test procedure
 *
 * 	1. check valid freq range (min ~ max or low ~ high)
 * 	2. avoid jump freq range
 * 	3. default freq range should be applied for no range setting ( 1 ~ 400 )
 * 	4. default freq range should be applied for max, min, low, high limit value
 * 	5. when jump freq range set, check all range conditions.
 * 	6. clear jump range, then clear low, high as well.
 *
 */

void test_setFreqParam(void)
{
	int result=0;
	int exp=0;
	float_t value, cur, low, high;
//	int i=0;

	// set normal working frequency
	value=10.0;
	exp=0; // OK
	result = FREQ_setFreqValue(value);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(value, iparam[FREQ_VALUE_INDEX].value.f);

	value=450.0;
	exp=1; // NOK
	result = FREQ_setFreqValue(value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//UARTprintf("min=%f max=%f\n", param.ctrl.freq_min, param.ctrl.freq_max);
	// check min ~ max range
	// current min ~ max : 100 ~ 300
	// set OK in min ~ max range
	value = 250.0;
	exp = 0;
	result = FREQ_setFreqValue(value);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(value, iparam[FREQ_VALUE_INDEX].value.f);

	//set jump freq
	low=-50.0; // out of limit
	high=200.0;
	exp = 1; //NOK
	result = FREQ_setJumpFreqRange(0, low, high);
	TEST_ASSERT_EQUAL_INT(result, exp);

	low=170;
	high=410;// out of limit
	exp = 1; //NOK
	result = FREQ_setJumpFreqRange(0, low, high);
	TEST_ASSERT_EQUAL_INT(exp, result);

	low=170.0;
	high=180.0;
	exp = 1; //NOK
	result = FREQ_setJumpFreqRange(3, low, high); // out of index
	TEST_ASSERT_EQUAL_INT(exp, result);

	low=170.0;
	high=180.0;
	exp = 0; //OK
	result = FREQ_setJumpFreqRange(0, low, high);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(iparam[JUMP_ENABLE0_INDEX].value.l, 1);
	TEST_ASSERT_EQUAL_FLOAT(iparam[JUMP_LOW0_INDEX].value.f, low);
	TEST_ASSERT_EQUAL_FLOAT(iparam[JUMP_HIGH0_INDEX].value.f, high);

	TEST_ASSERT_EQUAL_INT(dev_const.spd_jmp[0].enable, 1);
	TEST_ASSERT_EQUAL_FLOAT(dev_const.spd_jmp[0].low, low/USER_IQ_FULL_SCALE_FREQ_Hz);
	TEST_ASSERT_EQUAL_FLOAT(dev_const.spd_jmp[0].high, high/USER_IQ_FULL_SCALE_FREQ_Hz);

	cur = 150.0;
	result = FREQ_setFreqValue(cur);
	value = 175.0;
	exp = 170; // low jump
	result = FREQ_getVarifiedFreq(cur, value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	cur = 190.0;
	result = FREQ_setFreqValue(cur);
	value = 175.0;
	exp = 180; // high jump
	result = FREQ_getVarifiedFreq(cur, value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//clear jump range, then OK
	result = FREQ_clearJumpFreq(0);
	value = 175.0;
	exp = 175; //no jump check
	result = FREQ_getVarifiedFreq(cur, value);
	TEST_ASSERT_EQUAL_INT(exp, result);

}

#endif
