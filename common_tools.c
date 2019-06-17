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
void UTIL_setShaftBrake(void)
{
	internal_status.shaft_brake_enabled = 1;
	HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_Brake);
}

void UTIL_releaseShaftBrake(void)
{
	internal_status.shaft_brake_enabled = 0;
	HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_Brake);
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

int UTIL_controlLed(int type, int on_off)
{
	int result = 0;

	if(type == HAL_Gpio_LED_R || type == HAL_Gpio_LED_G)
	{
		if(on_off == 1)
			HAL_setGpioHigh(halHandle,(GPIO_Number_e)type);
		else
			HAL_setGpioLow(halHandle,(GPIO_Number_e)type);
	}
	else
	{
		UARTprintf("Error : no LED type=%d \n", type);
		result = 1;
	}

	return result;
}

// TODO : debug purpose only, using LED_R2 as test bit
void UTIL_testbit(int on_off) // LD2
{
	if(on_off == 1)
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED_R);
	else
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED_R);
}

void UTIL_testbitG(int on_off) // LD1
{
	if(on_off == 1)
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
	else
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


