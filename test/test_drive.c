/*
 * test_state.c
 *
 *  Created on: 2017. 3. 20.
 *      Author: hrjung
 */


#ifdef UNIT_TEST_ENABLED

#include "unity.h"
#include "../inv_param.h"
#include "../parameters.h"
#include "../drive.h"
#include "../protect.h"

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

/*
 *  ======== function ========
 */



/*
 * test procedure
 *
 * 1. set drive control, check input range
 * 2. according to drive control, then block command from other control input
 * 3. set ban direction, check input range
 * 4. according to ban direction, ignore dir command
 * 5. set multi-func in, check input range
 * 6. for multi-step freq setting, process freq step input
 */
void test_controlDrive(void)
{
	int result=0;
	int exp=0;

	// set energy saving off
	exp = 0;
	result = DRV_setEnergySave(ESAVE_UNUSED);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(0, (int)iparam[ENERGY_SAVE_INDEX].value.l);

	// set energy saving startup
	exp = 0;
	result = DRV_setEnergySave(ESAVE_STARTUP);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(1, (int)iparam[ENERGY_SAVE_INDEX].value.l);

	// set energy saving wrong value
	exp = 1;
	result = DRV_setEnergySave(ESAVE_BOTH+1);
	TEST_ASSERT_EQUAL_INT(exp, result);

#if 0
	// set Vmin boost correct value
	exp = 0;
	result = DRV_setVoltageBoost(0.0);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(0, iparam[V_BOOST_INDEX].value.f);

	// set Vmin boost correct value 0 ~ 15%
	exp = 0;
	result = DRV_setVoltageBoost(10.0);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(10.0, iparam[V_BOOST_INDEX].value.f);

	// set Vmin boost wrong value > 15%
	exp = 1;
	result = DRV_setVoltageBoost(20.1);
	TEST_ASSERT_EQUAL_INT(exp, result);
#endif

	// set PWM freq correct value
	exp = 0;
	result = DRV_setPwmFrequency(PWM_4KHz);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(PWM_4KHz, iparam[PWM_FREQ_INDEX].value.l);

	// set PWM freq correct value
	exp = 0;
	result = DRV_setPwmFrequency(PWM_16KHz);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(PWM_16KHz, iparam[PWM_FREQ_INDEX].value.l);

	// set PWM freq wrong value
	exp = 1;
	result = DRV_setPwmFrequency(PWM_16KHz+1);
	TEST_ASSERT_EQUAL_INT(exp, result);

}

#endif
