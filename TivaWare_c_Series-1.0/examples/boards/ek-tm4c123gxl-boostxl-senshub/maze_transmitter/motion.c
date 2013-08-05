//*****************************************************************************
//
// motion.c - Control of calibration, gestures and MPU9150 data aggregation.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"
#include "usblib/usblib.h"
#include "usblib/usbhid.h"
#include "drivers/rgb.h"
#include "events.h"
#include "motion.h"

//*****************************************************************************
//
// Global array that contains the colors of the RGB.  Motion is assigned the
// RED LED and should only modify RED.
//
// fast steady blink means I2C bus error.  Power cycle to clear.  usually
// caused by a reset of the system during an I2C transaction causing the slave
// to hold the bus.
//
// quick and brief blinks also occur on each motion system update.
//
//*****************************************************************************
extern volatile uint32_t g_pui32RGBColors[3];

//*****************************************************************************
//
// Global storage to count blink ticks.  This sets blink timing after error.
//
//*****************************************************************************
uint32_t g_ui32RGBMotionBlinkCounter;

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the MPU9150 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

//*****************************************************************************
//
// Global state variable for the motion state machine.
//
//*****************************************************************************
uint_fast8_t g_ui8MotionState;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global storage for most recent Euler angles and Sensor Data
//
//*****************************************************************************
float g_pfEulers[3];
float g_pfAccel[3];
float g_pfGyro[3];
float g_pfMag[3];

//*****************************************************************************
//
// Global storage for previous net acceleration magnitude reading. Helps smooth
// the emit classify readings.
//
//*****************************************************************************
float g_fAccelMagnitudePrevious;

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void
MotionI2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context.
//
//*****************************************************************************
void MotionCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        //
        // Set the motion event flag to show that we have completed the
        // i2c transfer
        //
        HWREGBITW(&g_ui32Events, MOTION_EVENT) = 1;

        //
        // Turn on the LED to show we are ready to process motion date
        //
        g_pui32RGBColors[RED] = 0xFFFF;
        RGBColorSet(g_pui32RGBColors);

        if(g_ui8MotionState == MOTION_STATE_RUN);
        {
            //
            // Get local copies of the raw motion sensor data.
            //
            MPU9150DataAccelGetFloat(&g_sMPU9150Inst, g_pfAccel, g_pfAccel + 1,
                                     g_pfAccel + 2);

            MPU9150DataGyroGetFloat(&g_sMPU9150Inst, g_pfGyro, g_pfGyro + 1,
                                    g_pfGyro + 2);

            MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, g_pfMag, g_pfMag + 1,
                                       g_pfMag + 2);

            //
            // Update the DCM. Do this in the ISR so that timing between the
            // calls is consistent and accurate.
            //
            CompDCMMagnetoUpdate(&g_sCompDCMInst, g_pfMag[0], g_pfMag[1],
                                 g_pfMag[2]);
            CompDCMAccelUpdate(&g_sCompDCMInst, g_pfAccel[0], g_pfAccel[1],
                               g_pfAccel[2]);
            CompDCMGyroUpdate(&g_sCompDCMInst, -g_pfGyro[0], -g_pfGyro[1],
                              -g_pfGyro[2]);
            CompDCMUpdate(&g_sCompDCMInst);
        }
    }
    else
    {
        //
        // An Error occurred in the I2C transaction.
        //
        HWREGBITW(&g_ui32Events, MOTION_ERROR_EVENT) = 1;
        UARTprintf("callback err: 0x%02x\n", ui8Status);
        //g_ui8MotionState = MOTION_STATE_ERROR;
        //g_ui32RGBMotionBlinkCounter = g_ui32SysTickCount;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag = ui8Status;

}


//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//
//*****************************************************************************
void
IntGPIOb(void)
{
    uint32_t ui32Status;

    ui32Status = GPIOIntStatus(GPIO_PORTB_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTB_BASE, ui32Status);

    //
    // Check which GPIO caused the interrupt event.
    //
    if(ui32Status & GPIO_PIN_2)
    {
        //
        // The MPU9150 data ready pin was asserted so start an I2C transfer
        // to go get the latest data from the device.
        //
    	MPU9150DataRead(&g_sMPU9150Inst, MotionCallback, &g_sMPU9150Inst);
    }
}

//*****************************************************************************
//
// MPU9150 Application error handler.
//
//*****************************************************************************
void
MotionErrorHandler(char * pcFilename, uint_fast32_t ui32Line)
{
	//
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in utils\\i2cm_drv.h\n",
               g_vui8ErrorFlag, pcFilename, ui32Line);

    //
    // Return terminal color to normal
    //
    UARTprintf("\033[0m");
}

//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete.
//
//*****************************************************************************
void
MotionI2CWait(char* pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((HWREGBITW(&g_ui32Events, MOTION_EVENT) == 0) &&
          (g_vui8ErrorFlag == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // clear the event flag.
    //
    HWREGBITW(&g_ui32Events, MOTION_EVENT) = 0;

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        MotionErrorHandler(pcFilename, ui32Line);
        g_vui8ErrorFlag = 0;
    }


    return;
}

void
MotionRestart(void)
{
	//
	// Set the motion state to initializing.
	//
	g_ui8MotionState = MOTION_STATE_INIT;

	//
	// Initialize I2C3 peripheral.
	//
	I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
			 ROM_SysCtlClockGet());

	//
	// Initialize the MPU9150 Driver.
	//
	MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
				MotionCallback, &g_sMPU9150Inst);



	//
	// Wait for transaction to complete
	//
	MotionI2CWait(__FILE__, __LINE__);

	//
	// Write application specifice sensor configuration such as filter settings
	// and sensor range settings.
	//
	g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
	g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
	g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
								  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
				 MotionCallback, &g_sMPU9150Inst);

	//
	// Wait for transaction to complete
	//
	MotionI2CWait(__FILE__, __LINE__);

	//
	// Configure the data ready interrupt pin output of the MPU9150.
	//
	g_sMPU9150Inst.pui8Data[0] = (MPU9150_INT_PIN_CFG_INT_LEVEL |
								  MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
								  MPU9150_INT_PIN_CFG_LATCH_INT_EN);
	g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
				 g_sMPU9150Inst.pui8Data, 2, MotionCallback, &g_sMPU9150Inst);

	//
	// Wait for transaction to complete
	//
	MotionI2CWait(__FILE__, __LINE__);

	//
	// Initialize the DCM system.
	//
	CompDCMInit(&g_sCompDCMInst, 1.0f / ((float) MOTION_SAMPLE_FREQ_HZ),
				DCM_ACCEL_WEIGHT, DCM_GYRO_WEIGHT, DCM_MAG_WEIGHT);
}

//*****************************************************************************
//
// Initialize the I2C, MPU9150 and Gesture systems.
//
//*****************************************************************************
void
MotionInit(void)
{
	//
    // Enable port B used for motion interrupt.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // The I2C3 peripheral must be enabled before use.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    //
    ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure and Enable the GPIO interrupt. Used for INT signal from the
    // MPU9150
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    ROM_IntEnable(INT_GPIOB);

    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();

    //
    // Initialize I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
             ROM_SysCtlClockGet());

    //
    // Set the motion state to initializing.
    //
    g_ui8MotionState = MOTION_STATE_INIT;

    //
    // Initialize the MPU9150 Driver.
    //
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MotionCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MotionI2CWait(__FILE__, __LINE__);

    //
    // Write application specifice sensor configuration such as filter settings
    // and sensor range settings.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MotionCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MotionI2CWait(__FILE__, __LINE__);

    //
    // Configure the data ready interrupt pin output of the MPU9150.
    //
    g_sMPU9150Inst.pui8Data[0] = (MPU9150_INT_PIN_CFG_INT_LEVEL |
                                  MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                                  MPU9150_INT_PIN_CFG_LATCH_INT_EN);
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MotionCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MotionI2CWait(__FILE__, __LINE__);

	//
	// Initialize the DCM system.
	//
	CompDCMInit(&g_sCompDCMInst, 1.0f / ((float) MOTION_SAMPLE_FREQ_HZ),
				DCM_ACCEL_WEIGHT, DCM_GYRO_WEIGHT, DCM_MAG_WEIGHT);

	//
	// Initialize the gesture instance and establish a initial state estimate.
	//
    //GestureInit(&g_sGestureInst, g_pfInitProb, g_ppfPath, g_ppfTransitionProb,
    //            g_ppfEmitProb, GESTURE_PATH_LENGTH, GESTURE_NUM_STATES,
    //            GESTURE_STATE_IDLE);
}

void MotionGetRPY(int16_t* pi16Roll, int16_t* pi16Pitch, int16_t* pi16Yaw)
{

	*pi16Roll = (int16_t)(g_pfEulers[0] * 1000.f);
	*pi16Pitch = (int16_t)(g_pfEulers[1] * 1000.f);
	*pi16Yaw = (int16_t)(g_pfEulers[2] * 1000.f);
}

//*****************************************************************************
//
// Main function to handler motion events that are triggered by the MPU9150
// data ready interrupt.
//
//*****************************************************************************
void
MotionMain(void)
{
    switch(g_ui8MotionState)
    {
        //
        // This is our initial data set from the MPU9150, start the DCM.
        //
        case MOTION_STATE_INIT:
        {
            //
            // Check the read data buffer of the MPU9150 to see if the
            // Magnetometer data is ready and present. This may not be the case
            // for the first few data captures.
            //
            if(g_sMPU9150Inst.pui8Data[14] & AK8975_ST1_DRDY)
            {
                //
                // Get local copy of Accel and Mag data to feed to the DCM
                // start.
                //
                MPU9150DataAccelGetFloat(&g_sMPU9150Inst, g_pfAccel,
                                         g_pfAccel + 1, g_pfAccel + 2);
                MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, g_pfMag,
                                          g_pfMag + 1, g_pfMag + 2);
                MPU9150DataGyroGetFloat(&g_sMPU9150Inst, g_pfGyro,
                                        g_pfGyro + 1, g_pfGyro + 2);

                //
                // Feed the initial measurements to the DCM and start it.
                // Due to the structure of our MotionMagCallback function,
                // the floating point magneto data is already in the local
                // data buffer.
                //
                CompDCMMagnetoUpdate(&g_sCompDCMInst, g_pfMag[0], g_pfMag[1],
                                     g_pfMag[2]);
                CompDCMAccelUpdate(&g_sCompDCMInst, g_pfAccel[0], g_pfAccel[1],
                                   g_pfAccel[2]);
                CompDCMStart(&g_sCompDCMInst);

                //
                // Proceed to the run state.
                //
                g_ui8MotionState = MOTION_STATE_RUN;

            }

            //
            // Turn off the LED to show we are done processing motion data.
            //
            g_pui32RGBColors[RED] = 0;
            RGBColorSet(g_pui32RGBColors);

            //
            // Finished
            //
            break;
        }

        //
        // DCM has been started and we are ready for normal operations.
        //
        case MOTION_STATE_RUN:
        {
            //
            // Get the latest Euler data from the DCM. DCMUpdate is done
            // inside the interrupt routine to insure it is not skipped and
            // that the timing is consistent.
            //
            CompDCMComputeEulers(&g_sCompDCMInst, g_pfEulers,
                                 g_pfEulers + 1, g_pfEulers + 2);

            //
            // Update best guess state based on past history and current
            // estimate.
            //
            //UARTprintf("M: Roll %5d\t", (int16_t) (g_pfEulers[0] * 1000.f));
            //UARTprintf("Pitch %5d\t", (int16_t) (g_pfEulers[1] * 1000.f));
            //UARTprintf("Yaw %5d\t", (int16_t) (g_pfEulers[2] * 1000.f));
            //UARTprintf("\n");

            //
            // Turn off the LED to show we are done processing motion data.
            //
            g_pui32RGBColors[RED] = 0;
            RGBColorSet(g_pui32RGBColors);

            //
            // Finished
            //
            break;
        }

        //
        // An I2C error has occurred at some point. Usually these are due to
        // asynchronous resets of the main MCU and the I2C peripherals. This
        // can cause the slave to hold the bus and the MCU to think it cannot
        // send.  In practice there are ways to clear this condition.  They are
        // not implemented here.  To clear power cycle the board.
        //
        case MOTION_STATE_ERROR:
        {
        	UARTprintf("\n\n\nMOTION ERROR!\n\n\n");
        	SysCtlDelay(1000);
        	MotionRestart();
            break;
        }
    }
}
