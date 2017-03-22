/* Copyright 2017, Gerardo Puga
 *
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief CIAA UART Driver for LPC54102
 **
 ** Implements the UART Driver for LPC54102
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Drivers CIAA Drivers
 ** @{ */
/** \addtogroup UART UART Drivers
 ** @{ */

/*==================[inclusions]=============================================*/



#include "ciaaDriverPulse.h"
#include "ciaaDriverConfig.h"
#include "ciaaDriverCommon.h"
#include "ciaaPOSIX_stdlib.h"
#include "ciaaPOSIX_stdio.h"
#include "os.h"

#undef INLINE

#include "chip.h"



/*==================[macros and definitions]=================================*/



#define CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(n) (ciaaDriverPulseCaptureLpc54102PulseCaptureTimers[n])

#define CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL       (0)

#define CIAA_DRIVER_PULSE_CAPTURE_LPC54102_PREESCALER_INPUT_FREQ (1e6)

#define CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS         (sizeof(ciaaDriverPulseCaptureLpc54102PulseCaptureDeviceDescriptionTable) / sizeof(ciaaDriverPulseCaptureLpc54102DeviceDescriptionType))


typedef struct {

   int32_t timerIndex;              /* Timer index number, 0 to 3 */

   char const * posixName;          /* Name of the device on the POSIX device tree. */

   uint32_t lpcIoconPort;
   uint32_t lpcIoconPin;
   uint32_t lpcIoconMode;
   uint32_t lpcIoconFunc;

   IRQn_Type lpcNvicInterrupt;

} ciaaDriverPulseCaptureLpc54102DeviceDescriptionType;



/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



LPC_TIMER_T *ciaaDriverPulseCaptureLpc54102PulseCaptureTimers[] = { LPC_TIMER0, LPC_TIMER1, LPC_TIMER2, LPC_TIMER3 };


const ciaaDriverPulseCaptureLpc54102DeviceDescriptionType ciaaDriverPulseCaptureLpc54102PulseCaptureDeviceDescriptionTable[] =
      {
            {
                  1,                                              /* timerIndex        */
                  "pulse/0",                                      /* posixName         */
                  1,                                              /* lpcIoconPort      */
                  5,                                              /* lpcIoconPin       */
                  (IOCON_DIGITAL_EN),                             /* lpcIoconMode      */
                  (IOCON_FUNC3)                                   /* lpcIoconFunc      */
            },
            {
                  0,                                              /* timerIndex        */
                  "pulse/1",                                      /* posixName         */
                  1,                                              /* lpcIoconPort      */
                  16,                                             /* lpcIoconPin       */
                  (IOCON_DIGITAL_EN),                             /* lpcIoconMode      */
                  (IOCON_FUNC3)                                   /* lpcIoconFunc      */
            }
      };


ciaaDevices_deviceType ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS];

ciaaDriverCommonLpc54102InternalBufferType ciaaDriverPulseCaptureLpc54102RxBufferTable[CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS];

int32_t ciaaDriverPulseCaptureLpc54102Timer2DevLookup[CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS];


/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/



inline IRQn_Type ciaaDriverPulseCapture_getLpcIrqIdFromUsartIndex(int32_t timerIndex)
{
   IRQn_Type lpcDeviceIrqId;

   lpcDeviceIrqId = CT32B0_IRQn;

   switch (timerIndex) {

      case 0 :
         lpcDeviceIrqId = CT32B0_IRQn;
         break;

      case 1 :
         lpcDeviceIrqId = CT32B1_IRQn;
         break;

      case 2 :
         lpcDeviceIrqId = CT32B2_IRQn;
         break;

      case 3 :
         lpcDeviceIrqId = CT32B3_IRQn;
         break;

      default :
         lpcDeviceIrqId = CT32B0_IRQn;
         break;
   }

   return lpcDeviceIrqId;
}


inline void ciaaDriverPulseCapture_enableInterrupt(int32_t timerIndex)
{

   Chip_TIMER_CaptureEnableInt(
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL);
}


inline void ciaaDriverPulseCapture_disableInterrupt(int32_t timerIndex)
{

   Chip_TIMER_CaptureDisableInt(
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL);
}


void ciaaDriverPulseCaptureLpc54102_InitializeControlStructures()
{
   int32_t devIndex;
   int32_t timerIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS; devIndex++)
   {
      ciaaDriverPulseCaptureLpc54102Timer2DevLookup[devIndex] = 0;
   }

   for (devIndex = 0; devIndex < CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS; devIndex++)
   {
      timerIndex = ciaaDriverPulseCaptureLpc54102PulseCaptureDeviceDescriptionTable[devIndex].timerIndex;

      /*
       * Build the backwards map from usartIndex to devIndex
       * */

      ciaaDriverPulseCaptureLpc54102Timer2DevLookup[timerIndex] = devIndex;

      /*
       * POSIX device information information
       * */

      ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex].path    = ciaaDriverPulseCaptureLpc54102PulseCaptureDeviceDescriptionTable[devIndex].posixName;

      ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex].open    = ciaaDriverPulseCapture_open;
      ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex].close   = ciaaDriverPulseCapture_close;
      ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex].read    = ciaaDriverPulseCapture_read;
      ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex].write   = ciaaDriverPulseCapture_write;
      ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex].ioctl   = ciaaDriverPulseCapture_ioctl;
      ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex].lseek   = NULL;

      ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex].upLayer = NULL;
      ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex].layer   = (void *)&ciaaDriverPulseCaptureLpc54102PulseCaptureDeviceDescriptionTable[devIndex];
      ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex].loLayer = (void *)timerIndex;

      /*
       * Initialize the rx buffer
       * */

      ciaaDriverCommonLpc54102_internalFifoClear(&ciaaDriverPulseCaptureLpc54102RxBufferTable[devIndex]);
   }
}


void ciaaDriverPulseCaptureLpc54102_hardwareInit()
{
   int32_t devIndex;
   int32_t timerIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS; devIndex++)
   {

      timerIndex = ciaaDriverPulseCaptureLpc54102PulseCaptureDeviceDescriptionTable[devIndex].timerIndex;

      Chip_TIMER_Init(
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex));

      Chip_TIMER_PrescaleSet(
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
            (Chip_Clock_GetAsyncSyscon_ClockRate() / CIAA_DRIVER_PULSE_CAPTURE_LPC54102_PREESCALER_INPUT_FREQ) - 1);

      Chip_TIMER_TIMER_SetCountClockSrc(
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
            TIMER_CAPSRC_RISING_PCLK,
            0);

      Chip_TIMER_CaptureFallingEdgeEnable(
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL);

      /*
       * Set the timer so that the counter is reset on the rising edge of the capture
       * channel input pin.
       * */

      CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex)->CTCR = (CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex)->CTCR & TIMER_CTCR_MASK) |
            ((uint32_t) (1 << 4)) | /* Clear the timer and the preescaler on edge. */
            ((uint32_t) (0 << 5)) | /* Act on rising edge. */
            ((uint32_t) (CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL << 6));

      Chip_TIMER_Reset(
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex));

      Chip_TIMER_CaptureEnableInt(
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL);

      NVIC_EnableIRQ(
            ciaaDriverPulseCapture_getLpcIrqIdFromUsartIndex(timerIndex));

      Chip_IOCON_PinMux(
            LPC_IOCON,
            ciaaDriverPulseCaptureLpc54102PulseCaptureDeviceDescriptionTable[devIndex].lpcIoconPort,
            ciaaDriverPulseCaptureLpc54102PulseCaptureDeviceDescriptionTable[devIndex].lpcIoconPin,
            ciaaDriverPulseCaptureLpc54102PulseCaptureDeviceDescriptionTable[devIndex].lpcIoconMode,
            ciaaDriverPulseCaptureLpc54102PulseCaptureDeviceDescriptionTable[devIndex].lpcIoconFunc);

      Chip_TIMER_Enable(
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex));
   }
}


void ciaaDriverPulseCaptureLpc54102_registerDevices()
{
   int32_t devIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS; devIndex++)
   {
      ciaaSerialDevices_addDriver(&ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex]);
   }
}


static void ciaaDriverPulseCaptureLpc54102_rxIndication(ciaaDevices_deviceType const * const device)
{
   /* receive the data and forward to upper layer */
   ciaaSerialDevices_rxIndication(device->upLayer, 1);
}


void ciaaDriverPulseCaptureLpc54102_unifiedIRQn(int32_t timerIndex)
{
   ciaaDriverCommonLpc54102InternalBufferType *rxBufferPtr;
   uint8_t *capturedValuePtr;
   uint32_t capturedValue;
   int32_t devIndex;
   int32_t byteIndex;

   devIndex = ciaaDriverPulseCaptureLpc54102Timer2DevLookup[timerIndex];

   rxBufferPtr = &ciaaDriverPulseCaptureLpc54102RxBufferTable[devIndex];

   capturedValue = Chip_TIMER_ReadCapture(
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL);

   Chip_TIMER_ClearCapture(
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL);

   capturedValuePtr = (uint8_t *)&capturedValue;

   for (byteIndex = 0; byteIndex < 4; byteIndex++)
   {
      ciaaDriverCommonLpc54102_internalFifoPush(
            rxBufferPtr,
            capturedValuePtr[byteIndex]);
   }

   ciaaDriverPulseCapture_disableInterrupt(timerIndex);

   ciaaDriverPulseCaptureLpc54102_rxIndication(
         &ciaaDriverPulseCaptureLpc54102PosixRegistrationTable[devIndex]);
}



/*==================[external functions definition]==========================*/



extern ciaaDevices_deviceType * ciaaDriverPulseCapture_open(char const * path, ciaaDevices_deviceType * device, uint8_t const oflag)
{
   return device;
}


extern int32_t ciaaDriverPulseCapture_close(ciaaDevices_deviceType const * const device)
{
   return 0;
}


extern int32_t ciaaDriverPulseCapture_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param)
{
   int32_t timerIndex;
   ssize_t ret = -1;

   timerIndex = (int32_t)device->loLayer;

   switch(request)
   {
      case ciaaPOSIX_IOCTL_SET_ENABLE_RX_INTERRUPT:

         if((bool)(intptr_t)param == false)
         {
            /*
             * Do nothing. Rx interrupts are disabled upon
             * reception of the capture event interrupt.
             * */

         } else {

            /*
             * Enable RX interrupts.
             * */
            ciaaDriverPulseCapture_enableInterrupt(timerIndex);
         }
         ret = 0;
         break;

   }

   return ret;
}


extern ssize_t ciaaDriverPulseCapture_read(ciaaDevices_deviceType const * const device, uint8_t* buffer, size_t const size)
{
   ciaaDriverCommonLpc54102InternalBufferType *rxBufferPtr;
   int32_t timerIndex;
   int32_t devIndex;
   ssize_t ret = -1;

   timerIndex = (int32_t)device->loLayer;

   devIndex   = ciaaDriverPulseCaptureLpc54102Timer2DevLookup[timerIndex];

   rxBufferPtr = &ciaaDriverPulseCaptureLpc54102RxBufferTable[devIndex];

   for (ret = 0; ret < size; ret++)
   {
      if (ciaaDriverCommonLpc54102_internalFifoIsEmpty(rxBufferPtr) == 0)
      {
         buffer[ret] = ciaaDriverCommonLpc54102_internalFifoPop(rxBufferPtr);

      } else {

         break;
      }
   }

   return ret;
}


extern ssize_t ciaaDriverPulseCapture_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size)
{
   return -1;
}


void ciaaDriverPulseCapture_init(void)
{
   ciaaDriverPulseCaptureLpc54102_InitializeControlStructures();

   ciaaDriverPulseCaptureLpc54102_hardwareInit();

   ciaaDriverPulseCaptureLpc54102_registerDevices();
}



/*==================[interrupt handlers]=====================================*/



ISR(PULSECAPTURE0_IRQHandler)
{
   ciaaDriverPulseCaptureLpc54102_unifiedIRQn(0);
}


ISR(PULSECAPTURE1_IRQHandler)
{
   ciaaDriverPulseCaptureLpc54102_unifiedIRQn(1);
}


ISR(PULSECAPTURE2_IRQHandler)
{
   ciaaDriverPulseCaptureLpc54102_unifiedIRQn(2);
}


ISR(PULSECAPTURE3_IRQHandler)
{
   ciaaDriverPulseCaptureLpc54102_unifiedIRQn(3);
}



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

