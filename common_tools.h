//###########################################################################
//
// FILE:   common_tools.h
//
// TITLE:
//
//###########################################################################

#ifndef __COMMON_TOOLS_H__
#define __COMMON_TOOLS_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_obj.h"
#include "hal.h"


/*******************************************************************************
 * MACROS
 */

#define MCU_COMM_READY_NOTI		0
#define MCU_COMM_IN_PROGRESS	1
#define MCU_COMM_STATUS_NOTI	2
#define MCU_COMM_ERROR_NOTI		3

//*****************************************************************************




//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void UTIL_setShaftBrake(void);
extern void UTIL_releaseShaftBrake(void);

extern void UTIL_setFanOn(void);
extern void UTIL_setFanOff(void);

#ifdef SUPPORT_COMM_MCU_STATE
extern int UTIL_isCommStatusReady(void);
extern uint16_t UTIL_getCommStatus(void);
extern void UTIL_setNotifyFlagMcu(uint16_t status);
#endif

extern int UTIL_controlLed(int type, int on_off);
extern void UTIL_testbit(int on_off);
extern void UTIL_testbitG(int on_off);
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __COMMON_TOOLS_H__


