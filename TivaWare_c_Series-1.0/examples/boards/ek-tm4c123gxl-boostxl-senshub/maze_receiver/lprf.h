//*****************************************************************************
//
// lprf.h - Prototypes for application LPRF (Low Power Radio Frequency)
// interface and stack.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************

#ifndef __LPRF_H__
#define __LPRF_H__

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


extern void LPRFInit(void);
extern void LPRFMain(void);
extern uint16_t getXPWMWidth(void);
extern uint16_t getYPWMWidth(void);
extern float getDeltaX(void);
extern float getDeltaY(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __LPRF_H__

