//*****************************************************************************
//
// airmouse.c - Main routines for SensHub Air Mouse Demo.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#if LAUNCHPAD_TIVA
#include "driverlib/pwm.h"
#else
#include "driverlib/timer.h"
#endif
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "drivers/rgb.h"
#include "remoti_uart.h"
#include "remoti_npi.h"
#include "remoti_rti.h"
#include "remoti_rtis.h"
#include "drivers/buttons.h"
#include "usblib/usblib.h"
#include "usblib/usbhid.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcomp.h"
#include "usblib/device/usbdhid.h"
#include "usblib/device/usbdhidmouse.h"
#include "usblib/device/usbdhidkeyb.h"
#include "events.h"
#include "motion.h"
#include "lprf.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Motion Air Mouse (airmouse)</h1>
//!
//! This example demonstrates the use of the Sensor Library, TM4C123G LaunchPad
//! and the SensHub BoosterPack to fuse nine axis sensor measurements into
//! motion and gesture events.  These events are then transformed into mouse
//! and keyboard events to perform standard HID tasks.
//!
//! Connect the device USB port on the side of the LaunchPad to a standard
//! computer USB port.  The LaunchPad with SensHub BoosterPack enumerates on
//! the USB bus as a composite HID keyboard and mouse.
//!
//! Hold the LaunchPad with the buttons away from the user and toward the
//! computer with USB Device cable exiting the right and bottom corner of the
//! board.
//!
//! - Roll or tilt the LaunchPad to move the mouse cursor of the computer
//! up, down, left and right.
//!
//! - The buttons on the LaunchPad perform the left and right mouse click
//! actions.  The buttons on the SensHub BoosterPack are not currently used by
//! this example.
//!
//! - A quick spin of the LaunchPad generates a PAGE_UP or PAGE_DOWN
//! keyboard press and release depending on the direction of the spin.  This
//! motion simulates scrolling.
//!
//! - A quick horizontal jerk to the left or right  generates a CTRL+ or
//! CTRL- keyboard event, which creates the zoom effect used in many
//! applications, especially web browsers.
//!
//! - A quick vertical lift generates an ALT+TAB keyboard event, which
//! allows the computer user to select between currently open windows.
//!
//! - A quick twist to the left or right moves the window selector.
//!
//! - A quick jerk in the down direction selects the desired window and
//! closes the window selection dialog.
//!
//! This example also supports the RemoTI low power RF Zigbee&reg;&nbsp;human
//! interface device profile.  The wireless features of this example require the
//! CC2533EMK expansion card and the CC2531EMK USB Dongle.  For details and
//! instructions for wireless operations see the Wiki at
//! http://processors.wiki.ti.com/index.php/Tiva_C_Series_LaunchPad and 
//! http://processors.wiki.ti.com/index.php/Wireless_Air_Mouse_Guide.
//
//*****************************************************************************

//*****************************************************************************
//
// Holds command bits used to signal the main loop to perform various tasks.
//
//*****************************************************************************
volatile uint32_t g_pui32RGBColors[3];

//*****************************************************************************
//
// Holds command bits used to signal the main loop to perform various tasks.
//
//*****************************************************************************
volatile uint_fast32_t g_ui32Events;

//*****************************************************************************
//
// Hold the state of the buttons on the board.
//
//*****************************************************************************
volatile uint_fast8_t g_ui8Buttons;

//*****************************************************************************
//
// Global system tick counter holds elapsed time since the application started
// expressed in 100ths of a second.
//
//*****************************************************************************
volatile uint_fast32_t g_ui32SysTickCount;

//*****************************************************************************
//
// The memory allocated to hold the composite descriptor that is created by
// the call to USBDCompositeInit().
//
//*****************************************************************************
#define DESCRIPTOR_DATA_SIZE    (COMPOSITE_DHID_SIZE + COMPOSITE_DHID_SIZE)
uint8_t g_pui8DescriptorData[DESCRIPTOR_DATA_SIZE];


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// This is the interrupt handler for the SysTick interrupt.  It is called
// periodically and updates a global tick counter then sets a flag to tell the
// main loop to move the mouse.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    g_ui32SysTickCount++;
    HWREGBITW(&g_ui32Events, USB_TICK_EVENT) = 1;
    HWREGBITW(&g_ui32Events, LPRF_TICK_EVENT) = 1;
    g_ui8Buttons = ButtonsPoll(0, 0);
}

void
Timer5BIntHandler(void)
{
	static uint8_t ui8Ping = 0;
	uint16_t ui16ActiveWidth;
	float fDeltaY;

//	fDeltaY = getDeltaY();


	//ui16ActiveWidth = 3750 + (fDeltaY * 1250);
	ui16ActiveWidth = getYPWMWidth();

	if(ui8Ping)
	{
		TimerLoadSet(TIMER5_BASE, TIMER_B, 50000 - ui16ActiveWidth);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0);
		ui8Ping = 0;
	}
	else
	{
		TimerLoadSet(TIMER5_BASE, TIMER_B, ui16ActiveWidth);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
		ui8Ping = 1;
	}
	//
	// Clear the timer interrupt flag.
	//
	TimerIntClear(TIMER5_BASE, TIMER_TIMB_TIMEOUT);

	TimerEnable(TIMER5_BASE, TIMER_B);
}

//*****************************************************************************
//
// The interrupt handler for the for TIMER5B interrupt.
//
//*****************************************************************************
void
Timer5AIntHandler(void)
{
	static uint8_t ui8Ping = 0;
	uint16_t ui16ActiveWidth;
	float fDeltaX;

	//fDeltaX = -1 * getDeltaX();

#if 0
    if (g_ui8Buttons & RIGHT_BUTTON)
    {
    	ui16ActiveWidth = 5000;
    	UARTprintf("r");
    }
    else if(g_ui8Buttons & LEFT_BUTTON)
    {
    	ui16ActiveWidth = 2500;
    	UARTprintf("l");
    }
    else
    {
    	ui16ActiveWidth = 3750;
    }
#else
    //ui16ActiveWidth = 3750 + (fDeltaX * 1250);
    ui16ActiveWidth = getXPWMWidth();
#endif
    if(ui8Ping)
    {
    	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
    	TimerLoadSet(TIMER5_BASE, TIMER_A, 50000 - ui16ActiveWidth);
    	ui8Ping = 0;
    }
    else
    {
    	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
    	TimerLoadSet(TIMER5_BASE, TIMER_A, ui16ActiveWidth);
		ui8Ping = 1;
    }

    //
	// Clear the timer interrupt flag.
	//
	TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER5_BASE, TIMER_A);
}

#if LAUNCHPAD_TIVA
#define PWM_TICKS_PER_SEC			(40000000/64)
void
EnablePWMs(void)
{
	unsigned long ulPeriod;

	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	GPIOPinConfigure(GPIO_PD1_M1PWM1);

	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);

	//
	// Set the period to 20 ms
	//
	ulPeriod = PWM_TICKS_PER_SEC * 20;

	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ulPeriod);

	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ulPeriod/2);

	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);

	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}
#else

#define TICKS_PER_SEC		40000000
#define PWM_DIV				(16)
#define PWM_TICKS_PER_SEC 	(TICKS_PER_SEC / PWM_DIV)
#define PWM_TICKS_PER_MS	(PWM_TICKS_PER_SEC / 1000)
void
EnablePWMs(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);

    TimerConfigure(TIMER5_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT |
    			   TIMER_CFG_B_ONE_SHOT);
    TimerPrescaleSet(TIMER5_BASE, TIMER_A, 15);
    TimerLoadSet(TIMER5_BASE, TIMER_A, 50000);
    IntEnable(INT_TIMER5A);
	TimerPrescaleSet(TIMER5_BASE, TIMER_B, 15);
	TimerLoadSet(TIMER5_BASE, TIMER_B, 50000);
	IntEnable(INT_TIMER5B);
    IntMasterEnable();
    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);
    //TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_0);   // this port now set to output

    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_0, 0);


#if 0
    //
    // The Timer2 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //
    // For this example T2CCP0 and 1 are used with port D pins 0 and 1
    // GPIO port D needs to be enabled so these pins can be used.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Configure the GPIO pin muxing for the Timer/CCP function.
    //
    GPIOPinConfigure(GPIO_PB4_T1CCP0);

    //
    // Configure the ccp settings for CC pin.
    //
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_4);

    //
	// Configure Timer2B as a 16-bit periodic timer.
	//
	TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);

	//
	// Configure the prescaler to divide by 16
	//
	TimerPrescaleSet(TIMER1_BASE, TIMER_A, 15);

	//
	// Set the Timer1B load value to 20 ms.  For neutral servo position, a
	// pulse of 1.5 ms is used.  From the load value down to the match value,
	// the signal will be high.  From the match value ot 0, the signal will be
	// low.
	//
	TimerLoadSet(TIMER1_BASE, TIMER_A, 5000);//PWM_TICKS_PER_MS * 20);

	//
	// Set the Timer1B match value to load a value of 1.5 ms.
	//
	TimerMatchSet(TIMER1_BASE, TIMER_A, TimerLoadGet(TIMER1_BASE, TIMER_A) / 3);//50000/3);

	//
	// Enable Timer1B.
	//
	TimerEnable(TIMER1_BASE, TIMER_A);
#endif //0'd out pwm timer code
}
#endif


//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

// Pairing reference
uint8_t destIdx;

//*****************************************************************************
//
// This is the main loop that runs the application.
//
//*****************************************************************************
int
main(void)
{
    //
    // Turn on stacking of FPU registers if FPU is used in the ISR.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run from the PLL at 40MHz.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Set the system tick to fire 100 times per second.
    //
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();

    //
    // Enable the Debug UART.
    //
    ConfigureUART();

    //
    // Print the welcome message to the terminal.
    //
    UARTprintf("\033[2JHost side marble maze application\n");

    //
    // Configure desired interrupt priorities. This makes certain that the DCM
    // is fed data at a consistent rate. Lower numbers equal higher priority.
    //
    ROM_IntPrioritySet(INT_I2C3, 0x00);
    ROM_IntPrioritySet(INT_GPIOB, 0x10);
    ROM_IntPrioritySet(FAULT_SYSTICK, 0x20);
    ROM_IntPrioritySet(INT_UART1, 0x60);
    ROM_IntPrioritySet(INT_UART0, 0x70);
    ROM_IntPrioritySet(INT_WTIMER5B, 0x80);

    //
    // Enable the PWM hardware
    //
    EnablePWMs();
    TimerEnable(TIMER5_BASE, TIMER_A | TIMER_B);

    //
    // User Interface Init
    //
    ButtonsInit();
    RGBInit(0);
    RGBEnable();

    //
    // Initialize the motion sub system.
    //
    //MotionInit();

    //
    // Initialize the Radio Systems.
    //
    LPRFInit();

    //
    // Drop into the main loop.
    //
    while(1)
    {

        //
        // Check for LPRF tick events.  LPRF Ticks are slower since UART to
        // RNP is much slower data connection than the USB.
        //
        if(HWREGBITW(&g_ui32Events, LPRF_TICK_EVENT) == 1)
        {
            //
            // Clear the event flag.
            //
            HWREGBITW(&g_ui32Events, LPRF_TICK_EVENT) = 0;

            //
            // Perform the LPRF Main task handling
            //
            LPRFMain();

        }
    }
}
