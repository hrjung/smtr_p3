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
#include "user.h"

#include "drv_spi.h"
#include "cmd_queue.h"

#ifdef SUPPORT_PRODUCTION_TEST_MODE

#ifdef FLASH
#pragma CODE_SECTION(test_startRun,"ramfuncs");
#pragma CODE_SECTION(test_stopRun,"ramfuncs");
//#pragma CODE_SECTION(test_startRun,"ramfuncs");
#pragma CODE_SECTION(processProductionMotorTest,"ramfuncs");
#endif

/*******************************************************************************
 * MACROS
 */

#define AVE_SAMPLE_CNT    16

/*******************************************************************************
 * CONSTANTS
 */


/*******************************************************************************
 * TYPEDEFS
 */

// basic test
enum
{
    P_TEST_BASIC_INIT,
    P_TEST_READ_TEMPERATURE,
    P_TEST_BASIC_RELAY,
    P_TEST_CONSTANT_PWM,
    P_TEST_BASIC_END,
};

enum
{
    P_TEST_DUTY_INIT,
    P_TEST_DUTY_RUN,
    P_TEST_DUTY_END,
};

// motor test
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

uint32_t p_start_time=0, p_end_time=0;

/*******************************************************************************
 * GLOBAL VARIABLES
 */
uint16_t production_test_mode_f = 0;

uint16_t p_test_i_ave_f=1;
uint16_t p_test_v_ave_f=1; // 0: fail, 1: pass

#ifdef PWM_DUTY_TEST
extern uint16_t gFlag_PwmTest;
#endif
/*******************************************************************************
 * EXTERNS
 */


extern uint32_t secCnt;

extern float_t MAIN_getIave(void);
extern float_t MAIN_getVave(void);

extern void dbg_enableSystem(void);
extern void dbg_disableSystem(void);

extern void UTIL_setTestPwmDuty(void);

extern int STA_isConstantRun(void);

extern void DRV_enableVfControl(void);
extern void DRV_enableFocControl(void);

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

int I_ave_test_index=0;
float_t i_rms_val[AVE_SAMPLE_CNT];
float_t i_ave_val=0.0;
/*
 *      read Irms value for noload motor drive (average value of 16 samples)
 */
uint16_t TEST_readIave(int i_ave_idx)
{
    int ave_idx=0;
    int i;
    float ave_sum=0;

    ave_idx = i_ave_idx%AVE_SAMPLE_CNT;
    i_rms_val[ave_idx] = MAIN_getIave();
    ave_idx++;

    if(i_ave_idx > AVE_SAMPLE_CNT)
    {
        ave_sum=0;
        for(i=0; i<AVE_SAMPLE_CNT; i++) ave_sum += i_rms_val[i];

        i_ave_val = ave_sum/(float_t)AVE_SAMPLE_CNT;

        //TODO : VF noload current is smaller than FOC, around 1.8A, need to fix ?
        if(fabsf(i_ave_val-USER_MOTOR_NO_LOAD_CURRENT) < 0.2)
            return 1;
        else
            return 0;
    }
    else
        return 1;

}

int V_ave_test_index=0; // read Vrms sample count
float_t v_rms_val[AVE_SAMPLE_CNT];
float_t v_ave_val=0.0;
/*
 *      read Vrms value for constant PWN duty test (average value of 16 samples)
 */
uint16_t TEST_readVave(int v_ave_idx)
{
    int ave_idx=0;
    int i;
    float ave_sum=0;

    ave_idx = v_ave_idx%AVE_SAMPLE_CNT;
    v_rms_val[ave_idx] = MAIN_getVave();
    ave_idx++;

    if(v_ave_idx > AVE_SAMPLE_CNT)
    {
        ave_sum=0;
        for(i=0; i<AVE_SAMPLE_CNT; i++) ave_sum += v_rms_val[i];

        v_ave_val = ave_sum/(float_t)AVE_SAMPLE_CNT;

        if(fabsf(v_ave_val-160.0) < 10.0) // 80% duty -> around 160V
            return 1;
        else
            return 0;
    }
    else
        return 1;

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
void processProductionConstantDutyTest(void)
{
    static int pt_state = P_TEST_DUTY_INIT;
    uint32_t elapse=0;

    switch(pt_state)
    {
    case P_TEST_DUTY_INIT:
        p_start_time = secCnt; // set start time

        // set VF mode
        DRV_enableVfControl();

#ifdef PWM_DUTY_TEST
        // set duty
        UTIL_setTestPwmDuty(); // set 80% -> Vrms ~= 160

        gFlag_PwmTest = true;
#endif
        dbg_enableSystem();
        V_ave_test_index=0;
        pt_state = P_TEST_DUTY_RUN;
        break;

    case P_TEST_DUTY_RUN:
        elapse = secCnt- p_start_time;
        if(elapse > 50 &&  elapse < 150) // check during 5 ~ 15 sec
        {
            // read Vrms
            if(TEST_readVave(V_ave_test_index) == 0) p_test_v_ave_f = 0;

            V_ave_test_index++;
        }
        else if(secCnt- p_start_time > 150) // 15 sec end test
        {
            dbg_disableSystem();
            pt_state = P_TEST_DUTY_END;
        }
        break;

    case P_TEST_DUTY_END:

        break;
    }
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
    static int drv_mode = 0; // 0: VF, 1: FOC

    switch(mp_state)
    {
    case P_TEST_MOTOR_INIT:
        // TODO : set motor parameter as test motor

        mp_state = P_TEST_MOTOR_START;
        break;

    case P_TEST_MOTOR_START:
        p_start_time = secCnt; // set start time

        if(drv_mode == 0) DRV_enableVfControl();
        else if(drv_mode == 1) DRV_enableFocControl();
        else
        {
            mp_state = P_TEST_MOTOR_END;
            break;
        }

        FREQ_setFreqValue(60.0); // test freq
        STA_calcResolution();
        test_startRun();
        mp_state = P_TEST_MOTOR_RUN;
        break;

    case P_TEST_MOTOR_RUN:
        // wait acceleration and hold, check speed
        if(secCnt- p_start_time > 300) // wait 30 sec running, goto stop
        {
            p_end_time = secCnt;
            I_ave_test_index = 0;
            test_stopRun();
            mp_state = P_TEST_MOTOR_STOP;
        }
        else // till 30 sec, check noload current
        {
            if(STA_isConstantRun())
            {
                if(TEST_readIave(I_ave_test_index) == 0) p_test_i_ave_f = 0;

                I_ave_test_index++;
            }

        }
        break;

    case P_TEST_MOTOR_STOP:
        if(secCnt- p_end_time > 150) // wait 30 sec, check speed
        {
            drv_mode++;
            if(drv_mode > 1) // after FOC drive, end
                mp_state = P_TEST_MOTOR_END;
            else
                mp_state = P_TEST_MOTOR_START; // goto next drive(FOC)
        }
        break;

    case P_TEST_MOTOR_END:
        //TODO: restore motor parameter

        break;
    }
}


#endif


