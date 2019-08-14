/*
 * production_test.c
 *
 *  Created on: 2019. 8. 13.
 *      Author: hrjung
 */

#include <stdint.h>

#include "hal.h"

#include "uartstdio.h"

#include "inv_param.h"
#include "parameters.h"

#include "drv_spi.h"
#include "cmd_queue.h"

#ifdef FLASH
#pragma CODE_SECTION(test_startRun,"ramfuncs");
#pragma CODE_SECTION(test_stopRun,"ramfuncs");
//#pragma CODE_SECTION(test_startRun,"ramfuncs");
#pragma CODE_SECTION(processProductionMotorTest,"ramfuncs");
#endif

#ifdef SUPPORT_PRODUCTION_TEST_MODE

/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */


/*******************************************************************************
 * TYPEDEFS
 */

enum
{
    P_TEST_MOTOR_INIT,
    P_TEST_MOTOR_START,
    P_TEST_MOTOR_RUN,
    P_TEST_MOTOR_STOP,
    P_TEST_MOTOR_END,
};

/*******************************************************************************
 * LOCAL VARIABLES
 */



/*******************************************************************************
 * LOCAL FUNCTIONS
 */

uint32_t p_start_time=0;

/*******************************************************************************
 * GLOBAL VARIABLES
 */
uint16_t production_test_mode_f = 0;

/*******************************************************************************
 * EXTERNS
 */


extern uint32_t secCnt;

extern int FREQ_setFreqValue(float_t value);
extern void STA_calcResolution(void);

/*
 *  ======== local function ========
 */



/*
 *  ======== public function ========
 */



void test_startRun(void)
{
    cmd_type_st que_data;

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
}


void test_stopRun(void)
{
    cmd_type_st que_data;

    que_data.cmd = SPICMD_CTRL_STOP;
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
}


/*
 *      basic test function in production
 *
 *      1. read IPM, motor temperature with default V value
 *      2. set relay on/off -> JIG
 *      3. set constant PWM duty, read Vrms
 *      4. check SPI communication with MCU
 *      5. check trip notification to MCU, from MCU
 *
 */
void processProductionBasicTest(void)
{

    // read input
}

/*
 *      Motor test function in production
 *
 *      1. set motor parameter to test motor
 *      2. run VF, with predetermined freq
 *      3. run FOC, with predetermined freq
 *      4. restore motor parameter as
 *
 */
void processProductionMotorTest(void)
{
    static int mp_state = P_TEST_MOTOR_INIT;

    switch(mp_state)
    {
    case P_TEST_MOTOR_INIT:
        // TODO : set motor parameter as test motor

        mp_state = P_TEST_MOTOR_START;
        break;

    case P_TEST_MOTOR_START:
        p_start_time = secCnt; // set start time

        FREQ_setFreqValue(60.0); // test freq
        STA_calcResolution();
        test_startRun();
        mp_state = P_TEST_MOTOR_RUN;
        break;

    case P_TEST_MOTOR_RUN:
        // wait acceleration and hold, check speed
        if(secCnt- p_start_time > 300) // wait 30 sec, check speed
        {

            mp_state = P_TEST_MOTOR_STOP;
        }
        break;

    case P_TEST_MOTOR_STOP:
        test_stopRun();
        mp_state = P_TEST_MOTOR_END;
        break;

    case P_TEST_MOTOR_END:
        //TODO: restore motor parameter

        break;
    }
}


#endif


