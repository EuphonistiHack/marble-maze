//*****************************************************************************
//
// remoti_uart.h - RemoTI UART API defines and declarations.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************

#ifndef __REMOTI_UART_H__
#define __REMOTI_UART_H__

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

//*****************************************************************************
//
// Defines the size of the UART transmit and receive buffers
//
//*****************************************************************************
#define REMOTI_UART_TX_BUF_SIZE 512
#define REMOTI_UART_RX_BUF_SIZE 512

//*****************************************************************************
//
// RemoTI UART driver error conditions.
//
//*****************************************************************************
#define REMOTI_UART_RX_LENGTH_ERR                                             \
                                1
#define REMOTI_UART_RX_FCS_ERR                                                \
                                2
#define REMOTI_UART_UNEXPECTED_SOF_ERR                                        \
                                3

//*****************************************************************************
//
// Remote Procedure Call constants
//
//*****************************************************************************
#define RPC_CMD_POLL            0x00
#define RPC_CMD_SREQ            0x20
#define RPC_CMD_AREQ            0x40
#define RPC_CMD_SRSP            0x60
#define RPC_CMD_RES4            0x80
#define RPC_CMD_RES5            0xA0
#define RPC_CMD_RES6            0xC0
#define RPC_CMD_RES7            0xE0
    
#define RPC_SYS_RES0            0
#define RPC_SYS_SYS             1
#define RPC_SYS_MAC             2
#define RPC_SYS_NWK             3
#define RPC_SYS_AF              4
#define RPC_SYS_ZDO             5
#define RPC_SYS_SAPI            6
#define RPC_SYS_UTIL            7
#define RPC_SYS_DBG             8
#define RPC_SYS_APP             9
#define RPC_SYS_RCAF            10
#define RPC_SYS_RCN             11
#define RPC_SYS_RCN_CLIENT      12
#define RPC_SYS_BOOT            13
#define RPC_SYS_MAX             14
    
#define RPC_CMD_TYPE_MASK       0xE0
#define RPC_SUBSYSTEM_MASK      0x1F

#define RPC_UART_SOF            0xFE

//*****************************************************************************
//
// Typedef the callback function prototype used by RemoTI UART driver to alert
// higher layers that an event has occurred.
//
//*****************************************************************************
typedef void (tRemoTICallback)(uint32_t);

//*****************************************************************************
//
// RemoTI UART API declarations.
//
//*****************************************************************************
extern void RemoTIUARTInit(uint32_t ui32Base);
extern void RemoTIUARTPutMsg(uint8_t* pui8Msg, uint_fast16_t ui16Length);
extern void RemoTIUARTGetMsg(uint8_t* pui8Msg, uint_fast16_t ui16Length);
extern uint_fast16_t RemoTIUARTGetRxMsgCount(void);
extern void RemoTIUARTRegisterMsgRxCallback(tRemoTICallback *pfnCallback);
extern void RemoTIUARTRegisterErrCallback(tRemoTICallback *pfnCallback);
extern void RemoTIUARTRegisterTxCompleteCallback(tRemoTICallback *pfnCallback);
//##### INTERNAL BEGIN #####
extern void RemoTIUARTWake(void);
//##### INTERNAL END #####

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __REMOTI_RTIS_H__
