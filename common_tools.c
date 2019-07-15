//###########################################################################
//
// FILE:   common_tools.c
//
// TITLE:
//
//###########################################################################

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <Assert.h>

//#include "main.h"
//#include "inv_param.h"
#include "common_tools.h"
#include "uartstdio.h"

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

extern HAL_Handle halHandle;
#ifdef SUPPORT_COMM_MCU_STATE
extern uint16_t comm_mcu_status;
#endif

//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************
// for SUPPORT_P3_HW GPIO changed to active Low
// set active than shaft brake released
// set inactive than shaft brake locked
void UTIL_setShaftBrake(void)
{
	internal_status.shaft_brake_locked = 1;
	HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_Brake);
}

void UTIL_releaseShaftBrake(void)
{
	internal_status.shaft_brake_locked = 0;
	HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_Brake);
}

void UTIL_setFanOn(void)
{
	internal_status.fan_enabled = 1;
	HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_Fan);
}

void UTIL_setFanOff(void)
{
	internal_status.fan_enabled = 0;
	HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_Fan);
}

#ifdef SUPPORT_COMM_MCU_STATE
int UTIL_isCommStatusReady(void)
{
	return (int)(comm_mcu_status == MCU_COMM_READY_NOTI);
}

uint16_t UTIL_getCommStatus(void)
{
	return comm_mcu_status;
}

void UTIL_setNotifyFlagMcu(uint16_t status)
{
	switch(status)
	{
	case MCU_COMM_READY_NOTI:
		comm_mcu_status = MCU_COMM_READY_NOTI;
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_MCU_NOTI0);
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_MCU_NOTI1);
		break;

	case MCU_COMM_IN_PROGRESS:
		comm_mcu_status = MCU_COMM_IN_PROGRESS;
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_MCU_NOTI0);
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_MCU_NOTI1);
		break;

	case MCU_COMM_STATUS_NOTI:
		comm_mcu_status = MCU_COMM_STATUS_NOTI;
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_MCU_NOTI0);
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_MCU_NOTI1);
		break;

	case MCU_COMM_ERROR_NOTI:
		comm_mcu_status = MCU_COMM_ERROR_NOTI;
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_MCU_NOTI0);
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_MCU_NOTI1);
		break;

	default:

		break;
	}
}
#endif

int UTIL_controlLed(int on_off)
{
    UTIL_testLED(on_off);

    return 1;
}

// TODO : debug purpose only, using LED_R2 as test bit
void UTIL_testLED(int on_off)
{
	if(on_off == 1)
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
	else
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
}

void UTIL_testbit0(int on_off) // test bit 0
{
    if(on_off == 1)
        HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_Test0);
    else
        HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_Test0);
}

void UTIL_testbit1(int on_off) // test bit 1
{
    if(on_off == 1)
        HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_Test1);
    else
        HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_Test1);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


