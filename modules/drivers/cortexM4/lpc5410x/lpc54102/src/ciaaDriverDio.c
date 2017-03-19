/* Copyright 2014, 2015 Pablo Ridolfi (UTN-FRBA)
 * Copyright 2017, Gerardo Puga
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

/** \brief CIAA Dio Driver for LPC54102
 **
 ** Implements the Digital Input/Output (Dio) Driver for LPC54102
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Drivers CIAA Drivers
 ** @{ */
/** \addtogroup DIO DIO Drivers
 ** @{ */

/*==================[inclusions]=============================================*/



#include "ciaaDriverDio.h"
#include "ciaaDriverConfig.h"
#include "ciaaDriverCommon.h"
#include "ciaaPOSIX_stdlib.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_string.h"
#include "os.h"

#undef INLINE

#include "chip.h"



/*==================[macros and definitions]=================================*/



#define CIAA_DRIVER_DIO_LPC54102_INPUT_PIN_COUNT    (sizeof(ciaaDriverDioLpc54102Inputs) / sizeof(ciaaDriverDioPinDescriptionType))

#define CIAA_DRIVER_DIO_LPC54102_OUTPUT_PIN_COUNT   (sizeof(ciaaDriverDioLpc54102Outputs) / sizeof(ciaaDriverDioPinDescriptionType))


typedef struct {

   ciaaDevices_deviceType * const * const devices;

   uint8_t countOfDevices;

} ciaaDriverConstType;


typedef struct {

   uint32_t port;

   uint32_t pin;

} ciaaDriverDioPinDescriptionType;



/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



const ciaaDriverDioPinDescriptionType ciaaDriverDioLpc54102Inputs[] = {
      {  0,   3 },   /* IN/0 [ 0] */
      {  0,   2 },   /* IN/0 [ 1] */
      {  0,   4 },   /* IN/0 [ 2] */
      {  0,   7 },   /* IN/0 [ 3] */
      {  1,  11 },   /* IN/0 [ 4] */
      {  0,  12 },   /* IN/0 [ 5] */
      {  0,  11 },   /* IN/0 [ 6] */
      {  1,  13 },   /* IN/0 [ 7] */
      {  1,  12 }    /* IN/0 [ 8] */
};


const ciaaDriverDioPinDescriptionType ciaaDriverDioLpc54102Outputs[] = {
      {  0,  29 },   /* OUT/0 [ 0] - LED R */
      {  0,  30 },   /* OUT/0 [ 1] - LED G */
      {  0,  31 },   /* OUT/0 [ 2] - LED B */
      {  0,  23 },   /* OUT/0 [ 3] */
      {  0,  22 },   /* OUT/0 [ 4] */
      {  0,  25 },   /* OUT/0 [ 5] */
      {  0,  24 },   /* OUT/0 [ 6] */
      {  0,  27 },   /* OUT/0 [ 7] */
      {  0,  26 },   /* OUT/0 [ 8] */
      {  1,   8 },   /* OUT/0 [ 9] */
      {  1,   7 },   /* OUT/0 [10] */
      {  1,  17 },   /* OUT/0 [11] */
      {  0,  28 },   /* OUT/0 [12] */
      {  1,  14 },   /* OUT/0 [13] */
      {  0,  19 }    /* OUT/0 [14] */
};


static ciaaDevices_deviceType ciaaDriverDioLpc54102InputDevice = {
      "in/0",                         /* driver name                    */
      ciaaDriverDio_open,             /* open function                  */
      ciaaDriverDio_close,            /* close function                 */
      ciaaDriverDio_read,             /* read function                  */
      ciaaDriverDio_write,            /* write function                 */
      ciaaDriverDio_ioctl,            /* ioctl function                 */
      NULL,                           /* seek function is not provided  */
      NULL,                           /* upper layer                    */
      NULL,                           /* layer                          */
      NULL                            /* NULL no lower layer            */
};


static ciaaDevices_deviceType ciaaDriverDioLpc54102OutputDevice = {
      "out/0",                        /* Driver name                    */
      ciaaDriverDio_open,             /* Open function                  */
      ciaaDriverDio_close,            /* Close function                 */
      ciaaDriverDio_read,             /* Read function                  */
      ciaaDriverDio_write,            /* Write function                 */
      ciaaDriverDio_ioctl,            /* IOCTL function                 */
      NULL,                           /* Seek function is not provided  */
      NULL,                           /* Upper layer                    */
      NULL,                           /* Layer                          */
      NULL                            /* NULL no lower layer            */
};


static ciaaDevices_deviceType * ciaaDriverDioLpc54102DevicesList[] =
      {
            &ciaaDriverDioLpc54102InputDevice,
            &ciaaDriverDioLpc54102OutputDevice,
            NULL
      };



/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/



static void ciaaDriverDioLpc54102_registerDevices()
{
   int32_t devIndex;

   for(devIndex = 0; ciaaDriverDioLpc54102DevicesList[devIndex] != NULL; devIndex++)
   {
      ciaaDioDevices_addDriver(ciaaDriverDioLpc54102DevicesList[devIndex]);
   }
}


static void ciaaDriverDioLpc54102_hardwareInit(void)
{
   int32_t gpioIndex;
   uint32_t mode;

   Chip_GPIO_Init(LPC_GPIO);

   /*
    * Inputs
    * */

   for (gpioIndex = 0; gpioIndex < CIAA_DRIVER_DIO_LPC54102_INPUT_PIN_COUNT; gpioIndex++)
   {
      mode = ciaaDriverCommonLpc54102_determineInputPinMode(
            ciaaDriverDioLpc54102Inputs[gpioIndex].port,
            ciaaDriverDioLpc54102Inputs[gpioIndex].pin);

      Chip_IOCON_PinMux(
            LPC_IOCON,
            ciaaDriverDioLpc54102Inputs[gpioIndex].port,
            ciaaDriverDioLpc54102Inputs[gpioIndex].pin,
            mode,
            IOCON_FUNC0);

      Chip_GPIO_SetDir(
            LPC_GPIO,
            ciaaDriverDioLpc54102Inputs[gpioIndex].port,
            ciaaDriverDioLpc54102Inputs[gpioIndex].pin,
            0); /* INPUT */
   }

   /*
    * Outputs
    * */

   for (gpioIndex = 0; gpioIndex < CIAA_DRIVER_DIO_LPC54102_OUTPUT_PIN_COUNT; gpioIndex++)
   {
      mode = ciaaDriverCommonLpc54102_determineOutputPinMode(
            ciaaDriverDioLpc54102Outputs[gpioIndex].port,
            ciaaDriverDioLpc54102Outputs[gpioIndex].pin);

      Chip_IOCON_PinMux(
            LPC_IOCON,
            ciaaDriverDioLpc54102Outputs[gpioIndex].port,
            ciaaDriverDioLpc54102Outputs[gpioIndex].pin,
            mode,
            IOCON_FUNC0);

      Chip_GPIO_SetDir(
            LPC_GPIO,
            ciaaDriverDioLpc54102Outputs[gpioIndex].port,
            ciaaDriverDioLpc54102Outputs[gpioIndex].pin,
            1); /* OUTPUT */

      Chip_GPIO_WritePortBit(
            LPC_GPIO,
            ciaaDriverDioLpc54102Outputs[gpioIndex].port,
            ciaaDriverDioLpc54102Outputs[gpioIndex].pin,
            0);
   }
}


static void ciaaDriverDioLpc54102_writeOutput(uint32_t outputNumber, uint32_t value)
{
   if (outputNumber < CIAA_DRIVER_DIO_LPC54102_OUTPUT_PIN_COUNT)
   {
      Chip_GPIO_SetPinState(LPC_GPIO,
            ciaaDriverDioLpc54102Outputs[outputNumber].port,
            ciaaDriverDioLpc54102Outputs[outputNumber].pin,
            value != 0 ? 1 : 0);
   }
}


static int32_t ciaaDriverDioLpc54102_readInput(uint32_t inputNumber)
{
   int32_t rv = -1;

   if (inputNumber < CIAA_DRIVER_DIO_LPC54102_INPUT_PIN_COUNT)
   {
      rv = Chip_GPIO_GetPinState(LPC_GPIO,
            ciaaDriverDioLpc54102Inputs[inputNumber].port,
            ciaaDriverDioLpc54102Inputs[inputNumber].pin) ? 1 : 0;
   }

   return rv;
}


static int32_t ciaaDriverDioLpc54102_readOutput(uint32_t outputNumber)
{
   int32_t rv = -1;

   if (outputNumber < CIAA_DRIVER_DIO_LPC54102_OUTPUT_PIN_COUNT)
   {
      rv = Chip_GPIO_GetPinState(LPC_GPIO,
            ciaaDriverDioLpc54102Outputs[outputNumber].port,
            ciaaDriverDioLpc54102Outputs[outputNumber].pin) ? 1 : 0;
   }

   return rv;
}


static int32_t ciaaDriverDioLpc54102_readPins(int32_t pinCount, uint8_t * buffer, size_t size, int32_t (*readFunction)(uint32_t))
{
   int32_t count;
   int32_t i, j;

   /*
    * Amount of bytes necessary to store all input states.
    * */

   count = (pinCount + 7) >> 3; /* +7 ensures rounding up on division. */

   /*
    * Truncate the pin count based on the buffer length.
    * */

   if(count > size)
   {
      count = size;
   }


   /*
    * Initialize the buffer with zeros.
    * */

   ciaaPOSIX_memset(buffer, 0, count);

   /*
    * Read and store all inputs in user buffer
    * */

   for(i = 0, j = 0; (i < pinCount) && (j < count); i++)
   {
      if((i > 0) && ((i & 0x07) == 0))
      {
         j++;
      }

      buffer[j] |= readFunction(i) << (i - 8 * j);
   }

   return count;
}



/*==================[external functions definition]==========================*/



extern ciaaDevices_deviceType * ciaaDriverDio_open(char const * path,
      ciaaDevices_deviceType * device, uint8_t const oflag)
{
   return device;
}


extern int32_t ciaaDriverDio_close(ciaaDevices_deviceType const * const device)
{
   return 0;
}


extern int32_t ciaaDriverDio_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param)
{
   return -1;
}


extern ssize_t ciaaDriverDio_read(ciaaDevices_deviceType const * const device, uint8_t * buffer, size_t size)
{
   ssize_t ret = -1;

   if(device == ciaaDriverDioLpc54102DevicesList[0])
   {
      /* accessing inputs */
      ret = ciaaDriverDioLpc54102_readPins(CIAA_DRIVER_DIO_LPC54102_INPUT_PIN_COUNT, buffer, size, ciaaDriverDioLpc54102_readInput);

   } else {

      if(device == ciaaDriverDioLpc54102DevicesList[1])
      {
         /* accessing outputs */
         ret = ciaaDriverDioLpc54102_readPins(CIAA_DRIVER_DIO_LPC54102_OUTPUT_PIN_COUNT, buffer, size, ciaaDriverDioLpc54102_readOutput);

      } else {

         /* Invalid device */
         ret = -1;
      }
   }

   return ret;
}


extern ssize_t ciaaDriverDio_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size)
{
   ssize_t ret = -1;
   int32_t i, j;

   if(size != 0)
   {
      if(device == ciaaDriverDioLpc54102DevicesList[0])
      {

         /* Inputs can't be written. */

         ret = -1;

      } else {

         if(device == ciaaDriverDioLpc54102DevicesList[1])
         {

            /* Set outputs according to bits defined in user buffer */

            for(i = 0, j = 0; (i < CIAA_DRIVER_DIO_LPC54102_OUTPUT_PIN_COUNT) && (j < size); i++)
            {
               if( (i > 0) && ((i & 0x07) == 0) )
               {
                  j++;
               }

               ciaaDriverDioLpc54102_writeOutput(i, buffer[j] & (1 << (i - 8 * j)));
            }

            /* Account for the last incomplete byte on the return value. */

            if((CIAA_DRIVER_DIO_LPC54102_OUTPUT_PIN_COUNT & 0x07) != 0)
            {
               j++;
            }

            ret = j;

         } else {

            /*
             * Unknown device.
             * */

            ret = -1;
         }
      }
   }

   return ret;
}


void ciaaDriverDio_init(void)
{
   ciaaDriverDioLpc54102_hardwareInit();

   ciaaDriverDioLpc54102_registerDevices();
}



/*==================[interrupt handlers]=====================================*/



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
