//*****************************************************************************
//
// events.h - Events that control software tasks.
//
// Copyright (c) 2013 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************


#ifndef __EVENTS_H__
#define __EVENTS_H__


//*****************************************************************************
//
// Number of SysTick Timer interrupts per second.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     100

//*****************************************************************************
//
// Number of timer perihperal ticks needed to get an active PWM width for the
// servo to go to its neutral position
//
//*****************************************************************************
#define SERVO_NEUTRAL_POSITION	3750

//*****************************************************************************
//
// Global system tick counter.  incremented by SysTickIntHandler.
//
//*****************************************************************************
extern volatile uint_fast32_t g_ui32SysTickCount;


//*****************************************************************************
//
// Hold the state of the buttons on the board.
//
//*****************************************************************************
extern volatile uint_fast8_t g_ui8Buttons;

//*****************************************************************************
//
// Holds command bits used to signal the main loop to perform various tasks.
//
//*****************************************************************************
extern volatile uint_fast32_t g_ui32Events;

#define USB_TICK_EVENT          0
#define MOTION_EVENT            1
#define MOTION_ERROR_EVENT      2
#define LPRF_EVENT              3
#define LPRF_TICK_EVENT         4


#endif // __EVENTS_H__
