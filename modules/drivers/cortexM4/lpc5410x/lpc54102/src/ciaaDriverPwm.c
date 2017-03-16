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
/** \addtogroup PWM Drivers
 ** @{ */

/*==================[inclusions]=============================================*/



#include "ciaaDriverPwm.h"
#include "chip.h"



/*==================[macros and definitions]=================================*/



#define CIAA_DRIVER_PWM_LPC54102_PWM_PORTS         (sizeof(lpc54102PwmConfigurationStructures) / sizeof(lpc54102PwmConfigurationStructuresType))

#define CIAA_DRIVER_PWM_LPC54102_PWM_RATE          1000

#define CIAA_DRIVER_PWM_LPC54102_SERVO_RATE        50

#define CIAA_DRIVER_PWM_LPC54102_MODE_PWM          0
#define CIAA_DRIVER_PWM_LPC54102_MODE_SERVO        1


typedef struct {

   int32_t pwmIndex;             /* match register number, from 1 to 4 */

   char const * posixName;       /* Name of the device on the POSIX device tree. */

   uint32_t sctOutputPinIndex;   /* SCT timer output pin where the PWM signal will appear. */

   uint32_t lpcIoconPort;
   uint32_t lpcIoconPin;
   uint32_t lpcIoconMode;
   uint32_t lpcIoconFunc;

   uint32_t operatingMode;

} lpc54102PwmConfigurationStructuresType;



/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



const lpc54102PwmConfigurationStructuresType lpc54102PwmConfigurationStructures[] =
      {
            {
                  1,                                  /* pwmIndex           */
                  "pwm/0",                            /* posixName          */
                  0,                                  /* sctOutputPinIndex  */
                  0,                                  /* lpcIoconPort       */
                  0,                                  /* lpcIoconPin        */
                  (IOCON_DIGITAL_EN),                 /* lpcIoconMode       */
                  0,                                  /* lpcIoconFunc       */
                  CIAA_DRIVER_PWM_LPC54102_MODE_PWM   /* operatingMode      */
            },
            {
                  2,                                  /* pwmIndex           */
                  "pwm/1",                            /* posixName          */
                  1,                                  /* sctOutputPinIndex  */
                  0,                                  /* lpcIoconPort       */
                  0,                                  /* lpcIoconPin        */
                  (IOCON_DIGITAL_EN),                 /* lpcIoconMode       */
                  0,                                  /* lpcIoconFunc       */
                  CIAA_DRIVER_PWM_LPC54102_MODE_PWM   /* operatingMode      */
            },
            {
                  3,                                  /* pwmIndex           */
                  "srv/0",                            /* posixName          */
                  2,                                  /* sctOutputPinIndex  */
                  0,                                  /* lpcIoconPort       */
                  0,                                  /* lpcIoconPin        */
                  (IOCON_DIGITAL_EN),                 /* lpcIoconMode       */
                  0,                                  /* lpcIoconFunc       */
                  CIAA_DRIVER_PWM_LPC54102_MODE_SERVO /* operatingMode      */
            },
            {
                  4,                                  /* pwmIndex           */
                  "srv/1",                            /* posixName          */
                  3,                                  /* sctOutputPinIndex  */
                  0,                                  /* lpcIoconPort       */
                  0,                                  /* lpcIoconPin        */
                  (IOCON_DIGITAL_EN)                  /* lpcIoconMode       */
                  0,                                  /* lpcIoconFunc       */
                  CIAA_DRIVER_PWM_LPC54102_MODE_SERVO /* operatingMode      */
            }
      };


ciaaDevices_deviceType lpc54102PosixRegistrationDataTable[CIAA_DRIVER_PWM_LPC54102_PWM_PORTS];



/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/



void ciaaDriverPwmLpc54102_InitializeControlStructures()
{
   int32_t usartIndex;
   int32_t devIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_PWM_LPC54102_PWM_PORTS; devIndex++)
   {
      lpc54102PosixRegistrationDataTable[devIndex].path    = lpc54102PwmConfigurationStructures[devIndex].posixName;

      lpc54102PosixRegistrationDataTable[devIndex].open    = ciaaDriverPwm_open;
      lpc54102PosixRegistrationDataTable[devIndex].close   = ciaaDriverPwm_close;
      lpc54102PosixRegistrationDataTable[devIndex].read    = ciaaDriverPwm_read;
      lpc54102PosixRegistrationDataTable[devIndex].write   = ciaaDriverPwm_write;
      lpc54102PosixRegistrationDataTable[devIndex].ioctl   = ciaaDriverPwm_ioctl;
      lpc54102PosixRegistrationDataTable[devIndex].lseek   = NULL;

      lpc54102PosixRegistrationDataTable[devIndex].upLayer = NULL;
      lpc54102PosixRegistrationDataTable[devIndex].layer   = (void *)&lpc54102PwmConfigurationStructures[devIndex];
      lpc54102PosixRegistrationDataTable[devIndex].loLayer = NULL;
   }
}


void ciaaDriverPwmLpc54102_hardwareInit()
{
   int32_t devIndex;

   Chip_SCTPWM_Init(LPC_SCT0);

   Chip_SCTPWM_SetRate(
         LPC_SCT0,
         CIAA_DRIVER_USART_LPC54102_PWM_RATE);

   for (devIndex = 0; devIndex < CIAA_DRIVER_PWM_LPC54102_PWM_PORTS; devIndex++)
   {
      Chip_SCTPWM_SetDutyCycle(
            LPC_SCT0,
            devIndex,
            0);

      Chip_SCTPWM_SetOutputPin(
            LPC_SCT0,
            devIndex,
            lpc54102PwmConfigurationStructures[devIndex].sctOutputPinIndex);

      Chip_IOCON_PinMux(
            LPC_IOCON,
            lpc54102PwmConfigurationStructures[devIndex].lpcIoconPort,
            lpc54102PwmConfigurationStructures[devIndex].lpcIoconPin,
            lpc54102PwmConfigurationStructures[devIndex].lpcIoconMode,
            lpc54102PwmConfigurationStructures[devIndex].lpcIoconFunc);
   }

   Chip_SCTPWM_Start(LPC_SCT0);
}


void ciaaDriverPwmLpc54102_registerDevices()
{
   int32_t devIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_PWM_LPC54102_PWM_PORTS; devIndex++)
   {
      ciaaDioDevices_addDriver(&lpc54102PosixRegistrationDataTable[devIndex]);
   }
}



/*==================[external functions definition]==========================*/



extern ciaaDevices_deviceType * ciaaDriverPwm_open(char const * path, ciaaDevices_deviceType * device, uint8_t const oflag)
{
   return device;
}


extern int32_t ciaaDriverPwm_close(ciaaDevices_deviceType const * const device)
{
   return 0;
}


extern int32_t ciaaDriverPwm_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param)
{
   return -1;
}


extern ssize_t ciaaDriverPwm_read(ciaaDevices_deviceType const * const device, uint8_t* buffer, size_t const size)
{
   return -1;
}


extern ssize_t ciaaDriverPwm_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size)
{
   lpc54102PwmConfigurationStructuresType *dev;
   uint32_t ticks;
   ssize_t ret;

   ret = 0;

   dev = (lpc54102PwmConfigurationStructuresType *)device->layer;

   if (size > 0)
   {

      if (dev->operatingMode == CIAA_DRIVER_PWM_LPC54102_MODE_PWM)
      {
         /*
          * PWM MODE
          *
          * The input is a percentage (0 to 100) indicating the PWM signal PWM duty cycle.
          * */

         if ((buffer[size - 1] >= 0) && (buffer[size - 1] <= 100))
         {
            ticks = Chip_SCTPWM_PercentageToTicks(
                  LPC_SCT0,
                  dev->pwmIndex,
                  buffer[size - 1]);

            Chip_SCTPWM_SetDutyCycle(
                  LPC_SCT0,
                  dev->pwmIndex,
                  ticks);

            ret = 0;

         } else {

            ret = -1;
         }

      } else {

         /*
          * SERVO MODE
          *
          * In this case, the input value is the width of the active pulse
          * measured in 1/100'ths of a millisecond.
          *
          *    100 = 1.00 ms.
          *    255 = 2.55 ms.
          *
          * Normally this would go from 100 to 200, since that is the valid range for most servos,
          * but since there might be reasons to go outside that range, no additional validation
          * is made on the value.
          *
          * */

         ticks = ((Chip_Clock_GetSystemClockRate() / 1000) * ((uint32_t)buffer[size - 1])) / 100;

         Chip_SCTPWM_SetDutyCycle(
               LPC_SCT0,
               dev->pwmIndex,
               ticks);

         ret = 0;
      }
   }

   return ret;
}


void ciaaDriverPwm_init(void)
{
   ciaaDriverPwmLpc54102_InitializeControlStructures();

   ciaaDriverPwmLpc54102_hardwareInit();

   ciaaDriverPwmLpc54102_registerDevices()
}



/*==================[interrupt handlers]=====================================*/



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

