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
/** \addtogroup ADC Drivers
 ** @{ */

/*==================[inclusions]=============================================*/



#include "ciaaDriverAdc.h"
#include "ciaaDriverCommon.h"
#include "chip.h"



/*==================[macros and definitions]=================================*/



#define CIAA_DRIVER_ADC_LPC54102_ADC_PORTS         (sizeof(lpc54102AdcConfigurationStructures) / sizeof(lpc54102AdcConfigurationStructuresType))

#define CIAA_DRIVER_ADC_LPC54102_ADC_FREQUENCY     10000000


typedef struct {

   int32_t adcIndex;

   char const * posixName;       /* Name of the device on the POSIX device tree. */

   uint32_t lpcIoconPort;
   uint32_t lpcIoconPin;

} lpc54102AdcConfigurationStructuresType;



/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



const lpc54102AdcConfigurationStructuresType lpc54102AdcConfigurationStructures[] =
      {
            {
                  9,                                  /* adcIndex           */
                  "adc/0",                            /* posixName          */
                  1,                                  /* lpcIoconPort       */
                  6                                   /* lpcIoconPin        */
            },
            {
                  7,                                  /* adcIndex           */
                  "adc/1",                            /* posixName          */
                  1,                                  /* lpcIoconPort       */
                  4                                   /* lpcIoconPin        */
            },
            {
                  6,                                  /* adcIndex           */
                  "adc/2",                            /* posixName          */
                  1,                                  /* lpcIoconPort       */
                  3                                   /* lpcIoconPin        */
            },
            {
                  5,                                  /* adcIndex           */
                  "adc/3",                            /* posixName          */
                  1,                                  /* lpcIoconPort       */
                  2                                   /* lpcIoconPin        */
            },
            {
                  4,                                  /* adcIndex           */
                  "adc/4",                            /* posixName          */
                  1,                                  /* lpcIoconPort       */
                  1                                   /* lpcIoconPin        */
            },
            {
                  3,                                  /* adcIndex           */
                  "adc/5",                            /* posixName          */
                  1,                                  /* lpcIoconPort       */
                  0                                   /* lpcIoconPin        */
            }
      };


ciaaDevices_deviceType lpc54102AdcPosixRegistrationDataTable[CIAA_DRIVER_ADC_LPC54102_ADC_PORTS];



/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/


void ciaaDriverAdcLpc54102_SetAdcInputPinInSafeMode(uint32_t port, uint32_t pin)
{
   uint32_t mode;

   mode = ciaaDriverCommonLpc54102_determineInputPinMode(port, pin);

   Chip_IOCON_PinMux(LPC_IOCON, port, pin, mode, IOCON_FUNC0);

   Chip_GPIO_SetDir(LPC_GPIO, port, pin, 0); /* INPUT */
}


void ciaaDriverAdcLpc54102_SetAdcInputPinInActiveMode(uint32_t port, uint32_t pin)
{
   Chip_IOCON_PinMux(LPC_IOCON, port, pin, IOCON_ANALOG_EN | IOCON_INPFILT_OFF, IOCON_FUNC0);

   Chip_GPIO_SetDir(LPC_GPIO, port, pin, 0); /* INPUT */
}


void ciaaDriverAdcLpc54102_InitializeControlStructures()
{
   int32_t usartIndex;
   int32_t devIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_ADC_LPC54102_ADC_PORTS; devIndex++)
   {
      lpc54102AdcPosixRegistrationDataTable[devIndex].path    = lpc54102AdcConfigurationStructures[devIndex].posixName;

      lpc54102AdcPosixRegistrationDataTable[devIndex].open    = ciaaDriverAdc_open;
      lpc54102AdcPosixRegistrationDataTable[devIndex].close   = ciaaDriverAdc_close;
      lpc54102AdcPosixRegistrationDataTable[devIndex].read    = ciaaDriverAdc_read;
      lpc54102AdcPosixRegistrationDataTable[devIndex].write   = ciaaDriverAdc_write;
      lpc54102AdcPosixRegistrationDataTable[devIndex].ioctl   = ciaaDriverAdc_ioctl;
      lpc54102AdcPosixRegistrationDataTable[devIndex].lseek   = NULL;

      lpc54102AdcPosixRegistrationDataTable[devIndex].upLayer = NULL;
      lpc54102AdcPosixRegistrationDataTable[devIndex].layer   = (void *)&lpc54102AdcConfigurationStructures[devIndex];
      lpc54102AdcPosixRegistrationDataTable[devIndex].loLayer = NULL;
   }
}


void ciaaDriverAdcLpc54102_hardwareInit()
{
   int32_t devIndex;

   Chip_ADC_Init(
         LPC_ADC,
         ADC_CR_RESOL(0x01) |             /* 8 bits per sample */
         ADC_CR_TSAMP(ADC_TSAMP_5CLK5));  /* Minimum required sample times for slow channels. */

   Chip_ADC_Calibration(
         LPC_ADC);

   Chip_ADC_SetClockRate(
         LPC_ADC,
         CIAA_DRIVER_ADC_LPC54102_ADC_FREQUENCY);

   /*
    * Set the corresponding input pins in a safe mode, digital with
    * input over-voltage protection hardware.
    * Pins will only be turned into analog  mode when the driver device
    * file is opened.
    */

   for (devIndex = 0; devIndex < CIAA_DRIVER_ADC_LPC54102_ADC_PORTS; devIndex++)
   {
      ciaaDriverAdcLpc54102_SetAdcInputPinInSafeMode(
            lpc54102AdcConfigurationStructures[devIndex].lpcIoconPort,
            lpc54102AdcConfigurationStructures[devIndex].lpcIoconPin);
   }
}


void ciaaDriverAdcLpc54102_registerDevices()
{
   int32_t devIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_ADC_LPC54102_ADC_PORTS; devIndex++)
   {
      ciaaDioDevices_addDriver(&lpc54102AdcPosixRegistrationDataTable[devIndex]);
   }
}



/*==================[external functions definition]==========================*/



extern ciaaDevices_deviceType * ciaaDriverAdc_open(char const * path, ciaaDevices_deviceType * device, uint8_t const oflag)
{
   lpc54102AdcConfigurationStructuresType *dev;

   dev = (lpc54102AdcConfigurationStructuresType *)device->layer;

   /* Set the input pin into analog mode */
   ciaaDriverAdcLpc54102_SetAdcInputPinInActiveMode(dev->lpcIoconPort, dev->lpcIoconPin);

   return device;
}


extern int32_t ciaaDriverAdc_close(ciaaDevices_deviceType const * const device)
{
   lpc54102AdcConfigurationStructuresType *dev;

   dev = (lpc54102AdcConfigurationStructuresType *)device->layer;

   /* Put the input pin back in a safe digital over-voltage protected mode. */
   ciaaDriverAdcLpc54102_SetAdcInputPinInSafeMode(dev->lpcIoconPort, dev->lpcIoconPin);

   return 0;
}


extern int32_t ciaaDriverAdc_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param)
{
   return -1;
}


extern ssize_t ciaaDriverAdc_read(ciaaDevices_deviceType const * const device, uint8_t* buffer, size_t const size)
{
   lpc54102AdcConfigurationStructuresType *dev;
   uint32_t dataRegister;

   dev = (lpc54102AdcConfigurationStructuresType *)device->layer;

   for (i = 0; i < size; i++)
   {

      Chip_ADC_DisableSequencer(
            LPC_ADC,
            ADC_SEQA_IDX);

      Chip_ADC_SetupSequencer(
            LPC_ADC,
            ADC_SEQA_IDX,
            ADC_SEQ_CTRL_CHANSEL(dev->adcIndex));

      Chip_ADC_EnableSequencer(
            LPC_ADC,
            ADC_SEQA_IDX);

      Chip_ADC_StartSequencer(
            LPC_ADC,
            ADC_SEQA_IDX);

      /*
       * This loop should only wait for about 15 ADC conversion
       * clock cycles for each sample.
       *  */

      do {

         dataRegister = Chip_ADC_GetDataReg(LPC_ADC, dev->adcIndex);

      } while ((dataRegister & ADC_DR_DATAVALID) == 0);

      buffer[i] = (dataRegister >> 8) & 0x00ff;

   }

   Chip_ADC_DisableSequencer(
         LPC_ADC,
         ADC_SEQA_IDX);

   return size;
}


extern ssize_t ciaaDriverAdc_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size)
{
   return -1;
}


void ciaaDriverAdc_init(void)
{
   ciaaDriverAdcLpc54102_InitializeControlStructures();

   ciaaDriverAdcLpc54102_hardwareInit();

   ciaaDriverAdcLpc54102_registerDevices()
}



/*==================[interrupt handlers]=====================================*/



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

