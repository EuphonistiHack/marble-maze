//*****************************************************************************
//
// lprf.c - Implementation of the applications low power radio frequency
// interface.
//
// Copyright (c) 2012-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 1.0 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "drivers/rgb.h"
#include "remoti_uart.h"
#include "remoti_npi.h"
#include "remoti_rti.h"
#include "remoti_rtis.h"
#include "remoti_zid.h"
#include "usblib/usblib.h"
#include "usblib/usbhid.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcomp.h"
#include "usblib/device/usbdhid.h"
#include "usblib/device/usbdhidmouse.h"
#include "usblib/device/usbdhidkeyb.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "events.h"
#include "nodeConfig.h"

#if  !defined (DEV_IS_CONTROLLER)  && !defined (DEV_IS_TARGET)
#error
#endif

//*****************************************************************************
//
// Global array that contains the colors of the RGB.  Low power RF is assigned
// green.
//
// Slow blinking means trying to pair.
// Solid means paired.
// Quick blinks occur when data is transmitted.  (Light off on transmit attempt
// light on when transmit attempt is complete).
// off not paired.
//
//*****************************************************************************
extern volatile uint32_t g_pui32RGBColors[3];


//*****************************************************************************
//
// Global storage to count blink ticks.  This sets blink timing after error.
//
//*****************************************************************************
uint32_t g_ui32RGBLPRFBlinkCounter;

//*****************************************************************************
//
// Global storage for buttons state of previous LPRF packet
//
//*****************************************************************************
uint_fast8_t g_ui8LPRFButtonsPrev;

//*****************************************************************************
//
// Current state of the LPRF network link.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8LinkState;

//*****************************************************************************
//
// The index into our pairing table that contains the current active link.
//
//*****************************************************************************
uint8_t g_ui8LinkDestIndex;

uint8_t g_ui8Sending;

#ifdef DEV_IS_TARGET
uint16_t g_ui16YWidth = SERVO_NEUTRAL_POSITION;
uint16_t g_ui16XWidth = SERVO_NEUTRAL_POSITION;

float g_fDeltaX;
float g_fDeltaY;
#endif

//*****************************************************************************
//
// Link States.
//
//*****************************************************************************
enum
{
  LINK_STATE_INIT,
  LINK_STATE_READY,
  LINK_STATE_PAIR,
  LINK_STATE_NDATA,
  LINK_STATE_UNPAIR,
  LINK_STATE_TEST,
  LINK_STATE_TEST_COMPLETED,
  LINK_STATE_OAD
};

//*****************************************************************************
//
// Global variables to hold key presses and hold information.
//
//*****************************************************************************
tZIDKeyboardData g_sZIDKeys;
bool g_bModifierHold;
bool g_bKeyHold;
bool g_bPressed;

//*****************************************************************************
//
// List of implemented device types.
//
//*****************************************************************************
const uint8_t pui8DevList[RTI_MAX_NUM_DEV_TYPES] =
{
#ifdef DEV_IS_CONTROLLER
	RTI_DEVICE_REMOTE_CONTROL,
#else
	RTI_DEVICE_TELEVISION,
#endif
	RTI_DEVICE_RESERVED_INVALID,
	RTI_DEVICE_RESERVED_INVALID
};

//*****************************************************************************
//
// List of implemented profiles.
//
//*****************************************************************************
const uint8_t pui8ProfileList[RTI_MAX_NUM_PROFILE_IDS] =
{
    RTI_PROFILE_ZRC, 0, 0, 0, 0, 0, 0
};

//*****************************************************************************
//
// List of possible target types.
//
//*****************************************************************************
const unsigned char pucTargetList[RTI_MAX_NUM_SUPPORTED_TGT_TYPES] =
{
#ifndef DEV_IS_CONTROLLER
	RTI_DEVICE_REMOTE_CONTROL,
#endif
	RTI_DEVICE_TELEVISION,
    RTI_DEVICE_SET_TOP_BOX,
    RTI_DEVICE_MEDIA_CENTER_PC,
    RTI_DEVICE_GAME_CONSOLE,
    RTI_DEVICE_MONITOR
#ifdef DEV_IS_CONTROLLER
    ,RTI_DEVICE_RESERVED_INVALID
#endif
};

//*****************************************************************************
//
// String version of Vendor Name.
//
//*****************************************************************************
const unsigned char pui8VendorName[]="TI-LPRF";

//*****************************************************************************
//
// Determine type of reset to issue based on user buttons presses.
// No buttons will cause a restore of all previous state values.
// Left button press will clear state information but not configuration.
// Right button will clear state information and configuration.
//
// Reset the RNP by toggling the RF modules external reset line.
//
//*****************************************************************************
void
ZIDResetRNP(void)
{
    //
    // Assert reset to the RNP.
    //
    ROM_GPIOPinWrite(EM_RESET_GPIO_PORT, EM_RESET_GPIO_PIN, 0);

    //
    // Hold reset low for about 8 milliseconds to verify reset is detected
    //
    ROM_SysCtlDelay(ROM_SysCtlClockGet() / (125 * 3));

    //
    //Release reset to the RNP
    //
    ROM_GPIOPinWrite(EM_RESET_GPIO_PORT, EM_RESET_GPIO_PIN, EM_RESET_GPIO_PIN);

    //
    // Delay to allow RNP to do its internal boot.
    //
    ROM_SysCtlDelay(ROM_SysCtlClockGet() / (2 * 3));

}

//*****************************************************************************
//
// Based on desired restoration setting, configure the RNP parameters for this
// application.
//
//*****************************************************************************
void
ZIDConfigParams(void)
{
    uint8_t pui8Value[MAX_AVAIL_DEVICE_TYPES];
    volatile uint8_t ui8Status;
    uint8_t ui8Tmp;

    g_ui8Sending = 0;
    UARTprintf("start config\n");


    pui8Value[0] = eRTI_CLEAR_STATE;

    //
    // Execute the desired startup control setting. Clears or restore based on
    // button status.
    //
    ui8Status = RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, pui8Value);
    if(ui8Status != RTI_SUCCESS)
    {
    	UARTprintf("Err init: wr startup ctrl\n");
    }

    //
    // If we successfully read the startup control value and it is not set to
    // restore the previous state then we need to re-configure the RNP.
    // If we are set to RESTORE then we can skip this configuration section.
    //
    if((ui8Status == RTI_SUCCESS)  && (pui8Value[0] != eRTI_RESTORE_STATE))
    {
        ui8Status = RTI_WriteItem(RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES,
                                 RTI_MAX_NUM_SUPPORTED_TGT_TYPES,
                                 pucTargetList);
        if(ui8Status != RTI_SUCCESS)
        {
        	UARTprintf("Err init: wr supp tgt types\n");
        }

        //
        // Application capabilities is a bit field that the application must
        // configure. It defines number of devices and profiles that this
        // node will be presenting to the network.
        //
        ui8Tmp = 0x12;
        ui8Status = RTI_WriteItem( RTI_CP_ITEM_APPL_CAPABILITIES, 1, &ui8Tmp);
        if (ui8Status != RTI_SUCCESS)
        {
        	UARTprintf("Err init: wr appl cap\n");
        }

        //
        // Write the list of supported device types to the RNP
        //
        ui8Status = RTI_WriteItem(RTI_CP_ITEM_APPL_DEV_TYPE_LIST,
                                 RTI_MAX_NUM_DEV_TYPES, pui8DevList);
        if(ui8Status != RTI_SUCCESS)
        {
        	UARTprintf("Err init: wr dev type list\n");
        }

        //
        // Write the list of supported profiles to the RNP.
        //
        ui8Status = RTI_WriteItem(RTI_CP_ITEM_APPL_PROFILE_ID_LIST,
                                 RTI_MAX_NUM_PROFILE_IDS, pui8ProfileList);
        if(ui8Status != RTI_SUCCESS)
        {
        	UARTprintf("Err init: wr supp prof types\n");
        }

        //
        // Node capabilities is a bit field that controls security, power
        // and other node characteristics.
        //
#ifdef DEV_IS_CONTROLLER
        ui8Tmp = 0x06;
#else
        ui8Tmp = 0x07;
#endif
        ui8Status = RTI_WriteItem(RTI_CP_ITEM_NODE_CAPABILITIES, 1, &ui8Tmp);
        if(ui8Status != RTI_SUCCESS)
        {
        	UARTprintf("Err init: wr node caps\n");
        }

        //
        // Write the Vendor ID number to the RNP.
        //
        pui8Value[0] = (uint8_t) (RTI_VENDOR_TEXAS_INSTRUMENTS & 0xFF);
        pui8Value[1] = (uint8_t) ((RTI_VENDOR_TEXAS_INSTRUMENTS >> 8) & 0xFF);
        ui8Status = RTI_WriteItem(RTI_CP_ITEM_VENDOR_ID, 2, pui8Value);
        if(ui8Status != RTI_SUCCESS)
        {
        	UARTprintf("Err init: wr vendor ID\n");
        }

        ui8Status = RTI_ReadItem(RTI_CP_ITEM_VENDOR_ID, 2, pui8Value);
        if(ui8Status == RTI_SUCCESS )
        {
            UARTprintf("Vendor ID: %d,%d\n", pui8Value[0], pui8Value[1]);
        }
        else
        {
        	UARTprintf("Err reading vendor ID!\n");
        }

        //
        // Write the string version of vendor name to the RNP.
        //
        ui8Status = RTI_WriteItem(RTI_CP_ITEM_VENDOR_NAME,
                                 sizeof(pui8VendorName), pui8VendorName);
        if(ui8Status != RTI_SUCCESS)
        {
        	UARTprintf("Err init: wr vendor name\n");
        }

        ui8Status = RTI_ReadItem(RTI_CP_ITEM_VENDOR_ID, 2, pui8Value);
		if(ui8Status == RTI_SUCCESS )
		{
			UARTprintf("aVendor ID: %d,%d\n", pui8Value[0], pui8Value[1]);
		}
		else
		{
			UARTprintf("Err reading vendor ID!\n");
		}

        //
        // Write the desired standby duty cycle to the RNP.
        //
        pui8Value[0] = (uint8_t) (1000 & 0xFF);
        pui8Value[1] = (uint8_t) (1000 >> 8);
        ui8Status = RTI_WriteItem(RTI_CP_ITEM_STDBY_DEFAULT_DUTY_CYCLE, 2,
                                 pui8Value);
        if(ui8Status != RTI_SUCCESS)
        {
        	UARTprintf("Err init: wr stdby duty cycle\n");
        }
        ui8Status = RTI_ReadItem(RTI_CP_ITEM_VENDOR_ID, 2, pui8Value);
		if(ui8Status == RTI_SUCCESS )
		{
			UARTprintf("bVendor ID: %d,%d\n", pui8Value[0], pui8Value[1]);
		}
		else
		{
			UARTprintf("Err reading vendor ID!\n");
		}
        UARTprintf("All conf written\n");
    }

    UARTprintf("end config\n");
}

//*****************************************************************************
//
// RTI Confirm function. Confirms the receipt of an Init Request (RTI_InitReq).
//
// Called by RTI_AsynchMsgProcess which we have placed in the main application
// context.
//
//*****************************************************************************
void
RTI_InitCnf(uint8_t ui8Status)
{
    uint8_t ui8MaxEntries, ui8Value;
	uint8_t pui8Value[MAX_AVAIL_DEVICE_TYPES];

    ui8Status = RTI_ReadItem(RTI_CP_ITEM_VENDOR_ID, 2, pui8Value);
	if(ui8Status == RTI_SUCCESS )
	{
		UARTprintf("1Vendor ID: %d,%d\n", pui8Value[0], pui8Value[1]);
	}
	else
	{
		UARTprintf("Err reading vendor ID!\n");
	}

    UARTprintf("init hit cnf\n");
    //
    // Verify return status.
    //
    if (ui8Status == RTI_SUCCESS)
    {
        //
        // Make sure startup control is now set back to RESTORE mode.
        //
        ui8Value = eRTI_RESTORE_STATE;
        ui8Status = RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &ui8Value);
        if(ui8Status != RTI_SUCCESS)
        {
        	UARTprintf("write startupctrl fail\n");
        }

        ui8Status = RTI_ReadItem(RTI_CP_ITEM_VENDOR_ID, 2, pui8Value);
		if(ui8Status == RTI_SUCCESS )
		{
			UARTprintf("2Vendor ID: %d,%d\n", pui8Value[0], pui8Value[1]);
		}
		else
		{
			UARTprintf("Err reading vendor ID!\n");
		}
        //
        // Determine the maximum number of pairing table entries.
        //
        ui8Status = RTI_ReadItemEx(RTI_PROFILE_RTI,
                                  RTI_CONST_ITEM_MAX_PAIRING_TABLE_ENTRIES,
                                  1, &ui8MaxEntries);
        if(ui8Status != RTI_SUCCESS)
        {
        	UARTprintf("read max table entries fail\n");
        }

        //
        // Verify that read was successful.
        //
        if(ui8Status == RTI_SUCCESS)
        {
			//
			// Delay briefly so we don't overrun the RPI buffers.
			//
			ROM_SysCtlDelay(ROM_SysCtlClockGet() / (100*3));

			//
			// Send a Disable Sleep Request.  This will wake up the RPI
			// and then start a pairing sequence.
			//
			RTI_DisableSleepReq();

			//
			// Set the link state to pairing in process
			//
			g_vui8LinkState = LINK_STATE_PAIR;
			g_ui32RGBLPRFBlinkCounter = g_ui32SysTickCount;
        }
    }
    else
    {
    	UARTprintf("Err!  initcnf called w/o succes: 0x%02x\n", ui8Status);
    }

}

//*****************************************************************************
//
// RTI confirm function. Called by RTI_AsynchMsgProcess when a pairing
// request is confirmed.  Contains the status of the pairing attempt and
// if successful the destination index.
//
//*****************************************************************************
void
RTI_PairCnf(uint8_t ui8Status, uint8_t ui8DestIndex,
            uint8_t ui8DevType)
{
    //
    // The maze side uses AllowPairReq, not InitPairReq, as it is the target in
    // this case.  As such, PairReq shouldn't be called, so paircnf does nada
    //
#ifdef DEV_IS_CONTROLLER

    static uint8_t sui8Tx[8];
    uint8_t ui8TXOptions = 0;
    //
    // Determine if the Pair attempt was successful.
    //
    if(ui8Status == RTI_SUCCESS)
    {
    	UARTprintf("paired!\n");
        //
        // Save the destination index we got back from our successful pairing.
        // Write that value back to the RNP so it knows this is the current
        // link to be used.
        //
        g_ui8LinkDestIndex = ui8DestIndex;
        ui8Status = RTI_WriteItemEx(RTI_PROFILE_RTI,
                                    RTI_SA_ITEM_PT_CURRENT_ENTRY_INDEX, 1,
                                    &g_ui8LinkDestIndex);

        //
        // Turn on the LED to show we paired and ready.
        //
        g_pui32RGBColors[GREEN] = 0x0;
        RGBColorSet(g_pui32RGBColors);

        sui8Tx[0] = 'h';

        RTI_SendDataReq(g_ui8LinkDestIndex, RTI_PROFILE_ZRC,
						RTI_VENDOR_TEXAS_INSTRUMENTS, ui8TXOptions,
						1, sui8Tx);

    }

    else
    {
    	UARTprintf("tried to pair, failed: 0x%02x\n", ui8Status);
        //
        // Turn off the LED to show pairing failed.
        //
        g_pui32RGBColors[GREEN] = 0;
        RGBColorSet(g_pui32RGBColors);
    }

    g_vui8LinkState = LINK_STATE_READY;
#endif
}

//*****************************************************************************
//
// RTI Confirm function. Called by RTI_AsynchMsgProcess if pairing was aborted.
// Currently not expected to get this call so just set the link state back to
// ready.
//
//*****************************************************************************
void
RTI_PairAbortCnf(uint8_t ui8Status)
{
    (void) ui8Status;

    //
    // Reset the link state.
    //
    if (LINK_STATE_PAIR == g_vui8LinkState)
    {
        g_vui8LinkState = LINK_STATE_READY;
    }
}

//*****************************************************************************
//
// RTI Confirm function. Called by RTI_AsynchMsgProcess when an allow pair
// request is recieved by the application processor.  This is not expected for
// this application.
//
//*****************************************************************************
void
RTI_AllowPairCnf(uint8_t ui8Status, uint8_t ui8DestIndex,
                 uint8_t ui8DevType)
{
    //
    // Do nothing. Controller does not trigger AllowPairReq() and hence is not
    // expecting this callback.
    //
    (void) ui8Status;


#ifdef DEV_IS_TARGET
    //
    // For maze_receiver, we are the target, so our pairing callback is
    // AllowPairCnf, not PairCnf.
    //

	//
	// Determine if the Pair attempt was successful.
	//
	if(ui8Status == RTI_SUCCESS)
	{
	UARTprintf("paired!\n");
	   //
	   // Save the destination index we got back from our successful pairing.
	   // Write that value back to the RNP so it knows this is the current
	   // link to be used.
	   //
	   g_ui8LinkDestIndex = ui8DestIndex;
	   ui8Status = RTI_WriteItemEx(RTI_PROFILE_RTI,
								   RTI_SA_ITEM_PT_CURRENT_ENTRY_INDEX, 1,
								   &g_ui8LinkDestIndex);

	   //
	   // Turn on the LED to show we paired and ready.
	   //
	   g_pui32RGBColors[GREEN] = 0x4000;
	   RGBColorSet(g_pui32RGBColors);
	}

	else
	{
	UARTprintf("tried to pair, failed: 0x%02x\n", ui8Status);
	   //
	   // Turn off the LED to show pairing failed.
	   //
	   g_pui32RGBColors[GREEN] = 0;
	   RGBColorSet(g_pui32RGBColors);
	}

	g_vui8LinkState = LINK_STATE_READY;
#endif
}

//*****************************************************************************
//
// RTI Confirm function.  Called by RTI_AsyncMsgProcess when a unpair request
// is confirmed by the RNP. Currently not implemented.
//
//*****************************************************************************
void
RTI_UnpairCnf(uint8_t ui8Status, uint8_t ui8DestIndex)
{
    //
    // unused arguments
    //
    (void) ui8Status;
    (void) ui8DestIndex;
}

//*****************************************************************************
//
// RTI indication function. Called by RTI_AsynchMsgProcess when the far side of
// the link is requesting to unpair. Currently not implemented.
//
//
//*****************************************************************************
void
RTI_UnpairInd(uint8_t ui8DestIndex)
{

    //
    // unused arguments
    //
    (void) ui8DestIndex;
}

//*****************************************************************************
//
// RTI Confirm function. Called by RTI_AsynchMsgProcess when a data send is
// confirmed. It is now clear to queue the next data packet.
//
//*****************************************************************************
void
RTI_SendDataCnf(uint8_t ui8Status)
{

	if(ui8Status != RTI_SUCCESS)
	{
		UARTprintf("senddatacnf error! 0x%02x\n", ui8Status);
	}

    //
    // Set the link state to ready.
    //
    if (g_vui8LinkState == LINK_STATE_NDATA)
    {
        g_vui8LinkState = LINK_STATE_READY;
    }

    //
    // Turn on the LED to show we are back to ready state.
    //
    g_pui32RGBColors[GREEN] = 0;
    RGBColorSet(g_pui32RGBColors);
    g_ui8Sending = 0;
}

//*****************************************************************************
//
// RTI Confirm function. Called by RTI_AsynchMsgProcess when the RxEnable
// request has been processed by the RNP. Not implemented.
//
//*****************************************************************************
void
RTI_RxEnableCnf(uint8_t ui8Status)
{
    //
    // Do nothing
    //
    (void) ui8Status;

}

//*****************************************************************************
//
// RTI Confirm function. Called by RTI_AsynchMsgProcess when the enable sleep
// request has been proccessed by the RNP.
//
//*****************************************************************************
void
RTI_EnableSleepCnf(uint8_t ui8Status)
{
    //
    // Do nothing
    //
    (void) ui8Status;
}

//*****************************************************************************
//
// RTI Confirm function. Called by RTI_AsynchMsgProcess when disable sleep
// request has been processed by the RNP. This is used during the init sequence
// as a trigger to start a pairing sequence if needed.
//
//*****************************************************************************
void
RTI_DisableSleepCnf(uint8_t ui8Status)
{
	uint8_t pui8Value[MAX_AVAIL_DEVICE_TYPES];
    (void) ui8Status;

    ui8Status = RTI_ReadItem(RTI_CP_ITEM_VENDOR_ID, 2, pui8Value);
	if(ui8Status == RTI_SUCCESS )
	{
		UARTprintf("3Vendor ID: %d,%d\n", pui8Value[0], pui8Value[1]);
	}
	else
	{
		UARTprintf("Err reading vendor ID!\n");
	}

    //
    // RNP is now awake, if we don't have a pairing link then start the pair
    // process.
    //
    if(g_ui8LinkDestIndex == RTI_INVALID_PAIRING_REF)
    {
#ifdef DEV_IS_CONTROLLER
    	UARTprintf("hitting PairReq\n");
		RTI_PairReq();
#else
    	UARTprintf("hitting AllowPairReq\n");
        RTI_AllowPairReq();
#endif
    }
}

//*****************************************************************************
//
// RTI Confirm function. Called by RTI_AsynchMsgProcess when the RNP would
// like to get the latest report data from the application processor.
//
//*****************************************************************************
void
RTI_ReceiveDataInd(uint8_t ui8SrcIndex, uint8_t ui8ProfileId,
                   uint16_t ui16VendorID, uint8_t ui8RXLinkQuality,
                   uint8_t ui8RXFlags, uint8_t ui8Length, uint8_t *pui8Data)
{

#ifdef DEV_IS_TARGET
    int16_t i16Roll, i16Pitch;

    if(ui8Length < 17)
    {
    	UARTprintf("len error!\n");
    }
    i16Roll = strtol((const char*)pui8Data, NULL, 10);
    i16Pitch = strtol((const char*)pui8Data+6, NULL, 10);

    g_fDeltaX = ((float)i16Roll/(float)1200);
    g_fDeltaY = ((float)i16Pitch/(float)1800);

    g_ui16XWidth = SERVO_NEUTRAL_POSITION + (g_fDeltaX * 2250);
    g_ui16YWidth = SERVO_NEUTRAL_POSITION + (-1 * g_fDeltaY * 2250);

    UARTprintf("RX: %s", pui8Data);
    UARTprintf("Roll: %5d, Pitch: %5d\r", i16Roll/10, i16Pitch/10);

#else
    int i;
	UARTprintf("rxd len %d: ", ui8Length);
	for(i=0;i<ui8Length;i++)
	{
		UARTprintf("0x%02x ", pui8Data[i]);
	}
#endif
}

//*****************************************************************************
//
// RTI Confirm function. Called by RTI_AsynchMsgProcess when a standby request
// has been processed by the RNP.
//
//*****************************************************************************
void
RTI_StandbyCnf(uint8_t ui8Status)
{

    (void) ui8Status;

}

//*****************************************************************************
//
// RTI Callback function. The lower level UART and NPI layers have verified
// received an asynchronous message. Set the flag to indicate it needs
// processed.
//
//*****************************************************************************
void
RTI_AsynchMsgCallback(uint32_t ui32Data)
{
    (void) ui32Data;

    //
    // Set the flag to tell LPRF Main that we need to process a message.
    //
    HWREGBITW(&g_ui32Events, LPRF_EVENT) = 1;
}


//*****************************************************************************
//
//
//*****************************************************************************
void
RTI_AsynchMsgProcess(void)
{
    tRemoTIMsg sMsg;

    //
    // Get the msg from the UART low level driver
    //
    RemoTIUARTGetMsg((uint8_t *) &sMsg, NPI_MAX_DATA_LEN);
    if ((sMsg.ui8SubSystem & 0x1F) == RPC_SYS_RCAF)
    {
        switch((unsigned long) sMsg.ui8CommandID)
        {
            case RTIS_CMD_ID_RTI_INIT_CNF:
                RTI_InitCnf(sMsg.pui8Data[0]);
                break;

            case RTIS_CMD_ID_RTI_PAIR_CNF:
                RTI_PairCnf(sMsg.pui8Data[0], sMsg.pui8Data[1], sMsg.pui8Data[2]);
                break;

            case RTIS_CMD_ID_RTI_PAIR_ABORT_CNF:
                RTI_PairAbortCnf(sMsg.pui8Data[0]);
                break;

            case RTIS_CMD_ID_RTI_ALLOW_PAIR_CNF:
                RTI_AllowPairCnf(sMsg.pui8Data[0], sMsg.pui8Data[1],
                                 sMsg.pui8Data[2]);
                break;

            case RTIS_CMD_ID_RTI_SEND_DATA_CNF:
                RTI_SendDataCnf(sMsg.pui8Data[0]);
                break;

            case RTIS_CMD_ID_RTI_REC_DATA_IND:
                RTI_ReceiveDataInd(sMsg.pui8Data[0], sMsg.pui8Data[1],
                                   sMsg.pui8Data[2] | (sMsg.pui8Data[3] << 8),
                                   sMsg.pui8Data[4], sMsg.pui8Data[5],
                                   sMsg.pui8Data[6], &sMsg.pui8Data[7]);
                break;

            case RTIS_CMD_ID_RTI_STANDBY_CNF:
                RTI_StandbyCnf(sMsg.pui8Data[0] );
                break;

            case RTIS_CMD_ID_RTI_ENABLE_SLEEP_CNF:
                RTI_EnableSleepCnf(sMsg.pui8Data[0] );
                break;

            case RTIS_CMD_ID_RTI_DISABLE_SLEEP_CNF:
                RTI_DisableSleepCnf(sMsg.pui8Data[0] );
                break;

            case RTIS_CMD_ID_RTI_RX_ENABLE_CNF:
                RTI_RxEnableCnf(sMsg.pui8Data[0] );
                break;

            case RTIS_CMD_ID_RTI_UNPAIR_CNF:
                RTI_UnpairCnf(sMsg.pui8Data[0], sMsg.pui8Data[1]);
                break;

            case RTIS_CMD_ID_RTI_UNPAIR_IND:
                RTI_UnpairInd(sMsg.pui8Data[0]);
                break;

            default:
                // nothing can be done here!
                break;
        }
    }
}

//*****************************************************************************
//
// We need a clean way for the timer interrupt handler (in maze_receiver) to
// get access to the needed PWM width that we calculated based on what we
// received (in this file).  The following is hacky and stupid, which is what
// happens when you don't want to do proper architecture practices before
// hacking away at a problem.  I am ashamed.
//
//*****************************************************************************
#ifdef DEV_IS_TARGET
uint16_t
getXPWMWidth(void)
{
	return(g_ui16XWidth);
}

uint16_t
getYPWMWidth(void)
{
	return(g_ui16YWidth);
}

float
getDeltaX(void)
{
	return(g_fDeltaX);
}

float
getDeltaY(void)
{
	return(g_fDeltaY);
}
#endif

//*****************************************************************************
//
//
//*****************************************************************************
void
LPRFInit(void)
{
    //
    // Enable the RemoTI UART pin muxing.
    // UART init is done in the RemoTI/uart_drv functions
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Configure the CC2533 Reset pin
    //
    ROM_SysCtlPeripheralEnable(EM_RESET_SYSCTL_PERIPH);
    ROM_GPIOPinTypeGPIOOutput(EM_RESET_GPIO_PORT, EM_RESET_GPIO_PIN);

    g_ui8LinkDestIndex = RTI_INVALID_PAIRING_REF;

    RemoTIUARTInit(UART1_BASE);
    IntEnable(INT_UART1);

    NPI_Init(RTI_AsynchMsgCallback);
    RTI_Init();

    ZIDResetRNP();
    ZIDConfigParams();
    RTI_InitReq();
}

//*****************************************************************************
//
//
//*****************************************************************************
void
LPRFMain(void)
{
#ifdef DEV_IS_CONTROLLER
	uint8_t ui8TXOptions = 0;
	static char pucOutStr[64];
#endif

    //
    // First determine if we need to process a asynchronous message, such as
    // a send data confirmation or pairing confirmation.
    //
    while(HWREGBITW(&g_ui32Events, LPRF_EVENT) != 0)
    {
        //
        // Processes the message and calls the appropriate RTI callback.
        //
        RTI_AsynchMsgProcess();

        //
        // Clear the event flag.
        //
        if(RemoTIUARTGetRxMsgCount() == 0)
        {
            HWREGBITW(&g_ui32Events, LPRF_EVENT) = 0;
        }
    }

    if(g_vui8LinkState == LINK_STATE_PAIR)
    {
        //
        // Pairing in process so do a steady quick blink of the LED.
        //
        if(g_ui32SysTickCount > (g_ui32RGBLPRFBlinkCounter + 20))
        {
            //
            // 20 ticks have expired since we last toggled so turn off the
            // LED and reset the counter.
            //
            g_ui32RGBLPRFBlinkCounter = g_ui32SysTickCount;
            g_pui32RGBColors[GREEN] = 0;
            RGBColorSet(g_pui32RGBColors);
        }
        else if(g_ui32SysTickCount == (g_ui32RGBLPRFBlinkCounter + 10))
        {
            //
            // 10 ticks have expired since the last counter reset.  turn
            // on the green LED.
            //
            g_pui32RGBColors[GREEN] = 0x4000;
            RGBColorSet(g_pui32RGBColors);
        }
    }
    else if(g_vui8LinkState == LINK_STATE_READY)
    {
    	//
    	// We are connected, turn on the green LED, turn off the red.
    	//
    	g_pui32RGBColors[GREEN] = 0x4000;
    	g_pui32RGBColors[RED] = 0;
    	RGBColorSet(g_pui32RGBColors);

#ifdef DEV_IS_CONTROLLER
		static int16_t i16RPYData[3];

		if(!g_ui8Sending)
		{
			MotionGetRPY(i16RPYData, i16RPYData+1, i16RPYData+2);
			sprintf(pucOutStr, "%05d %05d %05d", i16RPYData[0], i16RPYData[1],
					i16RPYData[2]);
			UARTprintf("%s, len %d\n", pucOutStr, strlen(pucOutStr));
			RTI_SendDataReq(g_ui8LinkDestIndex, RTI_PROFILE_ZRC,
							RTI_VENDOR_TEXAS_INSTRUMENTS, ui8TXOptions,
							strlen(pucOutStr), (uint8_t *)pucOutStr);
			TimerLoadSet(TIMER4_BASE, TIMER_A, 40000000/10);
			TimerEnable(TIMER4_BASE, TIMER_A);
			g_ui8Sending = 1;
		}
#endif
    }
}
