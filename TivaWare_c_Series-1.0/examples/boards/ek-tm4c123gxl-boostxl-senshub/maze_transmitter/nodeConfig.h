//*****************************************************************************
//
// nodeConfig.h - Main routines for SensHub Air Mouse Demo.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************

#ifndef __MAZE_NODE_H__
#define __MAZE_NODE_H__

//
// If this is the receiver, we refer to it as the target, as this is what the
// CC2000 group calls the device that is being talked to.
//
#define DEV_IS_TARGET

//
// Uncomment/comment the following lines to switch between using the sensor hub
// booster pack or the em header booster pack on the receiver.  The EM header
// booster pack is much cheaper, but has the reset line routed different than
// the sensorhub booster pack.  Of course, this is moot if you're using the
// transmitter.
//
#ifdef DEV_IS_TARGET
#define USE_SENSHUB
//#define USE_EM_ADAPTER

#if defined(USE_SENSHUB)
#define EM_RESET_SYSCTL_PERIPH	SYSCTL_PERIPH_GPIOC
#define EM_RESET_GPIO_PORT		GPIO_PORTC_BASE
#define EM_RESET_GPIO_PIN		GPIO_PIN_7
#elif defined(USE_EM_ADAPTER)
#define EM_RESET_SYSCTL_PERIPH	SYSCTL_PERIPH_GPIOA
#define EM_RESET_GPIO_PORT		GPIO_PORTA_BASE
#define EM_RESET_GPIO_PIN		GPIO_PIN_4
#else
#error "must define whether using em header or sens hub to talk to RF board"
#endif //defined(USE_SENSHUB or USE_EM_ADAPTER)
#endif //def DEV_IS_TARGET

#endif
