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
#include "chip.h"



/*==================[macros and definitions]=================================*/



#define CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(n) (lpc54102PulseCaptureTimers[n])

#define CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL       (0)

#define CIAA_DRIVER_PULSE_CAPTURE_LPC54102_PREESCALER_INPUT_FREQ (1e6)

#define CIAA_DRIVER_PULSE_CAPTURE_LPC54102_INTERNAL_BUFFER_SIZE  (8)

#define CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS         (sizeof(lpc54102PulseCaptureDeviceDescriptionTable) / sizeof(lpc54102PulseCaptureDeviceDescriptionType))


typedef struct {

   int32_t timerIndex;

   char const * posixName;          /* Name of the device on the POSIX device tree. */

   uint32_t lpcIoconPort;
   uint32_t lpcIoconPin;
   uint32_t lpcIoconMode;
   uint32_t lpcIoconFunc;

   IRQn_Type lpcNvicInterrupt;

} lpc54102PulseCaptureDeviceDescriptionType;


typedef struct {

   uint32_t circularQueue[CIAA_DRIVER_PULSE_CAPTURE_LPC54102_INTERNAL_BUFFER_SIZE];

   uint32_t head;

   uint32_t tail;

} ciaaDriverPulseCaptureLpc54102InternalBufferType;



/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



const LPC_TIMER_T lpc54102PulseCaptureTimers[] = { LPC_TIMER0, LPC_TIMER1, LPC_TIMER2, LPC_TIMER3 };


const lpc54102PulseCaptureDeviceDescriptionType lpc54102PulseCaptureDeviceDescriptionTable[] =
      {
            {
                  0,                                              /* timerIndex        */
                  "pulse/0",                                      /* posixName         */
                  0,                                              /* lpcIoconPort      */
                  0,                                              /* lpcIoconPin       */
                  0,                                              /* lpcIoconMode      */
                  0,                                              /* lpcIoconFunc      */
                  CT32B0_IRQn                                     /* lpcNvicInterrupt  */
            }
      };

ciaaDevices_deviceType lpc54102PulseCapturePosixRegistrationTable[CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS];

ciaaDriverPulseCaptureLpc54102InternalBufferType lpc54102PulseCaptureRxBufferTable[CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS];

int32_t lpc54102PulseCaptureTimer2DevLookup[CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS];


/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/



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


inline uint32_t ciaaDriverPulseCaptureLpc54102_internalFifoNextIndex(uint32_t index)
{
   int32_t incrementedIndex;

   incrementedIndex = index + 1;

   if (incrementedIndex >= CIAA_DRIVER_PULSE_CAPTURE_LPC54102_INTERNAL_BUFFER_SIZE)
   {
      incrementedIndex = incrementedIndex - CIAA_DRIVER_PULSE_CAPTURE_LPC54102_INTERNAL_BUFFER_SIZE;
   }

   return incrementedIndex;
}


inline void ciaaDriverPulseCaptureLpc54102_internalFifoClear(ciaaDriverPulseCaptureLpc54102InternalRxBuffer *fifo)
{
   fifo->head = 0;

   fifo->tail = 0;
}


inline int32_t ciaaDriverPulseCaptureLpc54102_internalFifoIsEmpty(ciaaDriverPulseCaptureLpc54102InternalRxBuffer *fifo)
{
   return (fifo->head == fifo->tail) ? 1 : 0;
}


inline int32_t ciaaDriverPulseCaptureLpc54102_internalFifoIsFull(ciaaDriverPulseCaptureLpc54102InternalRxBuffer *fifo)
{
   uint32_t nextTail;

   nextTail = ciaaDriverPulseCaptureLpc54102_internalFifoNextIndex(fifo->tail);

   return (fifo->head == nextTail) ? 1 : 0;
}


void ciaaDriverPulseCaptureLpc54102_internalFifoPush(ciaaDriverPulseCaptureLpc54102InternalRxBuffer *fifo, uint32_t item)
{
   /*
    * You must check whether the FIFO is full before calling this function.
    * */

   fifo->circularQueue[fifo->tail] = item;

   fifo->tail = ciaaDriverPulseCaptureLpc54102_internalFifoNextIndex(fifo->tail);
}


int32_t ciaaDriverPulseCaptureLpc54102_internalFifoPop(ciaaDriverPulseCaptureLpc54102InternalRxBuffer *fifo)
{
   uint32_t oldHead;

   /*
    * You must check whether the FIFO is empty before calling this function.
    * */

   oldHead = fifo->head;

   fifo->head = ciaaDriverUartLpc54102_internalFifoNextIndex(fifo->head);

   return fifo->circularQueue[oldHead];
}


void ciaaDriverPulseCaptureLpc54102_InitializeControlStructures()
{
   int32_t devIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS; devIndex++)
   {
      lpc54102PulseCaptureTimer2DevLookup[usartIndex] = 0;
   }

   for (devIndex = 0; devIndex < CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS; devIndex++)
   {
      /*
       * Build the backwards map from usartIndex to devIndex
       * */

      lpc54102PulseCaptureTimer2DevLookup[lpc54102PulseCaptureDeviceDescriptionTable[devIndex].timerIndex] = devIndex;

      /*
       * POSIX device information information
       * */

      lpc54102PulseCapturePosixRegistrationTable[devIndex].path    = lpc54102PulseCaptureDeviceDescriptionTable[devIndex].posixName;

      lpc54102PulseCapturePosixRegistrationTable[devIndex].open    = ciaaDriverPulseCapture_open;
      lpc54102PulseCapturePosixRegistrationTable[devIndex].close   = ciaaDriverPulseCapture_close;
      lpc54102PulseCapturePosixRegistrationTable[devIndex].read    = ciaaDriverPulseCapture_read;
      lpc54102PulseCapturePosixRegistrationTable[devIndex].write   = ciaaDriverPulseCapture_write;
      lpc54102PulseCapturePosixRegistrationTable[devIndex].ioctl   = ciaaDriverPulseCapture_ioctl;
      lpc54102PulseCapturePosixRegistrationTable[devIndex].lseek   = NULL;

      lpc54102PulseCapturePosixRegistrationTable[devIndex].upLayer = NULL;
      lpc54102PulseCapturePosixRegistrationTable[devIndex].layer   = (void *)&lpc54102PulseCaptureDeviceDescriptionTable[devIndex];
      lpc54102PulseCapturePosixRegistrationTable[devIndex].loLayer = (void *)timerIndex;

      /*
       * Initialize the rx buffer
       * */

      ciaaDriverPulseCaptureLpc54102_internalFifoClear(&lpc54102PulseCaptureRxBufferTable[devIndex]);
   }
}


void ciaaDriverPulseCaptureLpc54102_hardwareInit()
{
   int32_t devIndex;
   int32_t timerIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS; devIndex++)
   {

      timerIndex = lpc54102PulseCaptureDeviceDescriptionTable[devIndex].timerIndex;

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

      pTMR->CTCR = (pTMR->CTCR & TIMER_CTCR_MASK)
                            | ((uint32_t) (1 << 4)) /* Clear the timer and the preescaler on edge. */
                            | ((uint32_t) (0 << 5)) /* Act on rising edge. */
                            | ((uint32_t) (CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL << 6));

      Chip_TIMER_Reset(
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex));

      Chip_TIMER_CaptureEnableInt(
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL);

      NVIC_EnableIRQ(
            lpc54102PulseCaptureDeviceDescriptionTable[devIndex].lpcNvicInterrupt);

      Chip_IOCON_PinMux(
            LPC_IOCON,
            lpc54102PulseCaptureDeviceDescriptionTable[devIndex].lpcIoconPort,
            lpc54102PulseCaptureDeviceDescriptionTable[devIndex].lpcIoconPin,
            lpc54102PulseCaptureDeviceDescriptionTable[devIndex].lpcIoconMode,
            lpc54102PulseCaptureDeviceDescriptionTable[devIndex].lpcIoconFunc);

      Chip_TIMER_Enable(
            CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex));
   }
}


void ciaaDriverPulseCaptureLpc54102_registerDevices()
{
   int32_t devIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_PORTS; devIndex++)
   {
      ciaaSerialDevices_addDriver(&lpc54102PulseCapturePosixRegistrationTable[devIndex]);
   }
}


static void ciaaDriverPulseCaptureLpc54102_rxIndication(ciaaDevices_deviceType const * const device)
{
   /* receive the data and forward to upper layer */
   ciaaSerialDevices_rxIndication(device->upLayer, 1);
}


void ciaaDriverPulseCaptureLpc54102_unifiedIRQn(int32_t timerIndex)
{
   ciaaDriverUartLpc54102InternalBuffer *rxBufferPtr;
   uint8_t *capturedValuePtr;
   uint32_t capturedValue;
   int32_t devIndex;

   devIndex = lpc54102PulseCaptureTimer2DevLookup[timerIndex];

   rxBufferPtr = &lpc54102PulseCaptureRxBufferTable[devIndex];

   capturedValue = Chip_TIMER_ReadCapture(
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL);

   Chip_TIMER_ClearCapture(
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_TIMER(timerIndex),
         CIAA_DRIVER_PULSE_CAPTURE_LPC54102_CAPTURE_CHANNEL);

   capturedValuePtr = (uint8_t *)&capturedValue;

   for (byteIndex = 0; byteIndex < 4; byteIndex++)
   {
      ciaaDriverUartLpc54102_internalFifoPush(
            rxBufferPtr,
            capturedValuePtr[byteIndex]);
   }

   ciaaDriverPulseCapture_disableInterrupt(timerIndex);

   ciaaDriverUartLpc54102_rxIndication(
         &lpc54102PulseCapturePosixRegistrationTable[devIndex]);
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

   timerIndex = (ciaaDriverPulseCaptureLpc54102DeviceDescriptiontype *)device->loLayer;

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
   ciaaDriverPulseCaptureLpc54102InternalBufferType *rxBufferPtr;
   int32_t timerIndex;
   int32_t devIndex;
   ssize_t ret = -1;

   timerIndex = (ciaaDriverPulseCaptureLpc54102DeviceDescriptiontype *)device->loLayer;

   devIndex   = lpc54102PulseCaptureTimer2DevLookup[timerIndex];

   rxBufferPtr = &lpc54102PulseCaptureRxBufferTable[devIndex];

   for (ret = 0; ret < size; ret++)
   {
      if (ciaaDriverPulseCaptureLpc54102_internalFifoIsEmpty(rxBufferPtr) == 0)
      {
         buffer[ret] = ciaaDriverPulseCaptureLpc54102_internalFifoPop(rxBufferPtr);

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

   ciaaDriverPulseCaptureLpc54102_registerDevices()
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

