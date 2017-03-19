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
/** \addtogroup ENCODER Drivers
 ** @{ */

/*==================[inclusions]=============================================*/



#include "ciaaDriverEncoder.h"
#include "ciaaDriverConfig.h"
#include "ciaaDriverCommon.h"
#include "ciaaPOSIX_stdlib.h"
#include "ciaaPOSIX_stdio.h"
#include "os.h"

#undef INLINE

#include "chip.h"



/*==================[macros and definitions]=================================*/



#define CIAA_DRIVER_ENCODER_LPC54102_GRAY_ENCODERS               (sizeof(lpc54102EncoderDescriptionStructures) / sizeof(lpc54102EncoderDescriptionStructureType))

#define CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_UNKNOWN    0x0f
#define CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_00         0x00
#define CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_01         0x01
#define CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_02         0x03
#define CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_03         0x02

typedef struct {

   int32_t encoderIndex;

   char const * posixName;

   uint32_t lpcIoconPort1;
   uint32_t lpcIoconPort2;

   uint32_t lpcIoconPin1;
   uint32_t lpcIoconPin2;

} lpc54102EncoderDescriptionStructureType;



/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



const IRQn_Type lpc54102PinIntInterrupts[] = {
      PIN_INT0_IRQn,
      PIN_INT1_IRQn,
      PIN_INT2_IRQn,
      PIN_INT3_IRQn,
      PIN_INT4_IRQn,
      PIN_INT5_IRQn,
      PIN_INT6_IRQn,
      PIN_INT7_IRQn
};


const lpc54102EncoderDescriptionStructureType lpc54102EncoderDescriptionStructures[] = {
      {
            0,                            /* encoderIndex   */
            "enc/0",                      /* posixName      */
            1,                            /* lpcIoconPort1  */
            1,                            /* lpcIoconPort2  */
            10,                           /* lpcIoconPin1   */
            9                             /* lpcIoconPin2   */
      },
      {
            1,                            /* encoderIndex   */
            "enc/1",                      /* posixName      */
            0,                            /* lpcIoconPort1  */
            1,                            /* lpcIoconPort2  */
            21,                           /* lpcIoconPin1   */
            15                            /* lpcIoconPin2   */
      }
};


ciaaDevices_deviceType lpc54102EncoderPosixRegistrationDataTable[CIAA_DRIVER_ENCODER_LPC54102_GRAY_ENCODERS];

uint32_t lpc54102EncoderPreviousEncoderStatus[CIAA_DRIVER_ENCODER_LPC54102_GRAY_ENCODERS];

uint32_t lpc54102EncoderPreviousEncoderPosition[CIAA_DRIVER_ENCODER_LPC54102_GRAY_ENCODERS];



/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/



void ciaaDriverEncoderLpc54102_InitializeControlStructures()
{
   int32_t devIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_ENCODER_LPC54102_GRAY_ENCODERS; devIndex++)
   {
      lpc54102EncoderPosixRegistrationDataTable[devIndex].path    = lpc54102EncoderDescriptionStructures[devIndex].posixName;

      lpc54102EncoderPosixRegistrationDataTable[devIndex].open    = ciaaDriverEncoder_open;
      lpc54102EncoderPosixRegistrationDataTable[devIndex].close   = ciaaDriverEncoder_close;
      lpc54102EncoderPosixRegistrationDataTable[devIndex].read    = ciaaDriverEncoder_read;
      lpc54102EncoderPosixRegistrationDataTable[devIndex].write   = ciaaDriverEncoder_write;
      lpc54102EncoderPosixRegistrationDataTable[devIndex].ioctl   = ciaaDriverEncoder_ioctl;
      lpc54102EncoderPosixRegistrationDataTable[devIndex].lseek   = NULL;

      lpc54102EncoderPosixRegistrationDataTable[devIndex].upLayer = NULL;
      lpc54102EncoderPosixRegistrationDataTable[devIndex].layer   = (void *)&lpc54102EncoderDescriptionStructures[devIndex];
      lpc54102EncoderPosixRegistrationDataTable[devIndex].loLayer = NULL;
   }
}


void ciaaDriverEncoderLpc54102_hardwareInit()
{
   uint32_t mode1, mode2;
   int32_t pIntSelIndex1, pIntSelIndex2;
   uint32_t pIntChannelMask;
   int32_t devIndex;

   Chip_PININT_Init(LPC_PININT);

   pIntChannelMask = 0;

   for (devIndex = 0; (devIndex < CIAA_DRIVER_ENCODER_LPC54102_GRAY_ENCODERS) && (devIndex < 4); devIndex++)
   {

      pIntSelIndex1 = (2 * devIndex + 0);
      pIntSelIndex2 = (2 * devIndex + 1);

      pIntChannelMask = pIntChannelMask | PININTCH(pIntSelIndex1);
      pIntChannelMask = pIntChannelMask | PININTCH(pIntSelIndex2);

      /* *** */

      mode1 = ciaaDriverCommonLpc54102_determineInputPinMode(
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort1,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort1);

      mode2 = ciaaDriverCommonLpc54102_determineInputPinMode(
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort2,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort2);

      /* *** */

      Chip_INMUX_PinIntSel(
            pIntSelIndex1,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort1,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPin1);

      Chip_INMUX_PinIntSel(
            pIntSelIndex2,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort2,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPin2);

      /* *** */

      Chip_IOCON_PinMux(
            LPC_IOCON,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort1,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPin1,
            mode1,
            IOCON_FUNC0);

      Chip_IOCON_PinMux(
            LPC_IOCON,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort2,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPin2,
            mode2,
            IOCON_FUNC0);

      /* *** */

      Chip_GPIO_SetDir(
            LPC_GPIO,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort1,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPin1,
            0); /* INPUT */

      Chip_GPIO_SetDir(
            LPC_GPIO,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort2,
            lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPin2,
            0); /* INPUT */

      /* *** */

      NVIC_EnableIRQ(
            lpc54102PinIntInterrupts[pIntSelIndex1]);

      NVIC_EnableIRQ(
            lpc54102PinIntInterrupts[pIntSelIndex2]);

      /* *** */

      lpc54102EncoderPreviousEncoderPosition[devIndex] = 0;
      lpc54102EncoderPreviousEncoderStatus[devIndex]   = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_UNKNOWN;
   }

   Chip_PININT_SetPinModeEdge(
         LPC_PININT,
         pIntChannelMask);

   Chip_PININT_EnableIntHigh(
         LPC_PININT,
         pIntChannelMask);

   Chip_PININT_EnableIntLow(
         LPC_PININT,
         pIntChannelMask);
}


void ciaaDriverEncoderLpc54102_registerDevices()
{
   int32_t devIndex;

   for (devIndex = 0; (devIndex < CIAA_DRIVER_ENCODER_LPC54102_GRAY_ENCODERS) && (devIndex < 4); devIndex++)
   {
      ciaaDioDevices_addDriver(&lpc54102EncoderPosixRegistrationDataTable[devIndex]);
   }
}


void ciaaDriverEncoderLpc54102_processEncoderInput(int32_t devIndex, uint32_t currentEncoderStatus)
{
   uint32_t oneStepUp;
   uint32_t oneStepDown;

   oneStepUp   = 0;
   oneStepDown = 0;

   switch (lpc54102EncoderPreviousEncoderStatus[devIndex]) {

      case CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_00 :
         oneStepUp   = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_03;
         oneStepDown = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_01;
         break;

      case CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_01 :
         oneStepUp   = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_00;
         oneStepDown = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_02;
         break;

      case CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_02 :
         oneStepUp   = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_01;
         oneStepDown = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_03;
         break;

      case CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_03 :
         oneStepUp   = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_02;
         oneStepDown = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_00;
         break;

      default :
         oneStepUp   = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_UNKNOWN;
         oneStepDown = CIAA_DRIVER_ENCODER_LPC54102_ENCODER_GRAY_POS_UNKNOWN;
         break;
   }

   if (currentEncoderStatus == oneStepUp)
   {
      lpc54102EncoderPreviousEncoderPosition[devIndex]--;

   }

   if (currentEncoderStatus == oneStepDown)
   {
      lpc54102EncoderPreviousEncoderPosition[devIndex]++;
   }

   lpc54102EncoderPreviousEncoderStatus[devIndex] = currentEncoderStatus;
}


void ciaaDriverEncoderLpc54102_irqHandler(uint32_t pIntSelIndex)
{
   int32_t devIndex;
   uint32_t pinIntStatus;
   uint32_t currentEncoderStatus;
   int32_t pin0, pin1;

   pinIntStatus = Chip_PININT_GetIntStatus(LPC_PININT);

   Chip_PININT_ClearIntStatus(LPC_PININT, pinIntStatus);

   for (devIndex = 0; (devIndex < CIAA_DRIVER_ENCODER_LPC54102_GRAY_ENCODERS) && (devIndex < 4); devIndex++)
   {
      pin0 = Chip_GPIO_GetPinState(LPC_GPIO, lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort1, lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPin1) ? 1 : 0;
      pin1 = Chip_GPIO_GetPinState(LPC_GPIO, lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPort2, lpc54102EncoderDescriptionStructures[devIndex].lpcIoconPin2) ? 1 : 0;

      currentEncoderStatus = pin1 * 2 + pin0;

      ciaaDriverEncoderLpc54102_processEncoderInput(devIndex, currentEncoderStatus);
   }
}



/*==================[external functions definition]==========================*/



extern ciaaDevices_deviceType * ciaaDriverEncoder_open(char const * path, ciaaDevices_deviceType * device, uint8_t const oflag)
{
   return device;
}


extern int32_t ciaaDriverEncoder_close(ciaaDevices_deviceType const * const device)
{
   return 0;
}


extern int32_t ciaaDriverEncoder_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param)
{
   return -1;
}


extern ssize_t ciaaDriverEncoder_read(ciaaDevices_deviceType const * const device, uint8_t* buffer, size_t const size)
{
   lpc54102EncoderDescriptionStructureType *deviceDescription;

   deviceDescription = (lpc54102EncoderDescriptionStructureType *)device->layer;

   size_t retValue;

   retValue = -1;

   if (size > 0)
   {
      buffer[0] = (uint8_t)(lpc54102EncoderPreviousEncoderPosition[deviceDescription->encoderIndex] & 0x00ff);

      retValue = 1;
   }

   return retValue;
}


extern ssize_t ciaaDriverEncoder_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size)
{
   return -1;
}


void ciaaDriverEncoder_init(void)
{
   ciaaDriverEncoderLpc54102_InitializeControlStructures();

   ciaaDriverEncoderLpc54102_hardwareInit();

   ciaaDriverEncoderLpc54102_registerDevices();
}



/*==================[interrupt handlers]=====================================*/



ISR(PIN_INT0_IRQn)
{
   ciaaDriverEncoderLpc54102_irqHandler(0);
}


ISR(PIN_INT1_IRQn)
{
   ciaaDriverEncoderLpc54102_irqHandler(1);
}


ISR(PIN_INT2_IRQn)
{
   ciaaDriverEncoderLpc54102_irqHandler(2);
}


ISR(PIN_INT3_IRQn)
{
   ciaaDriverEncoderLpc54102_irqHandler(3);
}


ISR(PIN_INT4_IRQn)
{
   ciaaDriverEncoderLpc54102_irqHandler(4);
}


ISR(PIN_INT5_IRQn)
{
   ciaaDriverEncoderLpc54102_irqHandler(5);
}


ISR(PIN_INT6_IRQn)
{
   ciaaDriverEncoderLpc54102_irqHandler(6);
}


ISR(PIN_INT7_IRQn)
{
   ciaaDriverEncoderLpc54102_irqHandler(7);
}



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

