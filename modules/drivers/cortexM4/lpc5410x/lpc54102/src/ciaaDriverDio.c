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
#include "ciaaPOSIX_stdlib.h"
#include "ciaaPOSIX_string.h"
#include "chip.h"



/*==================[macros and definitions]=================================*/



#define ciaaDriverDio_InputCount          (sizeof(ciaaDriverDio_Inputs) / sizeof(ciaaDriverDio_dioType))

#define ciaaDriverDio_OutputCount         (sizeof(ciaaDriverDio_Outputs) / sizeof(ciaaDriverDio_dioType))


typedef struct {

   ciaaDevices_deviceType * const * const devices;

   uint8_t countOfDevices;

} ciaaDriverConstType;


typedef struct {

   uint32_t port;

   uint32_t pin;

} ciaaDriverDio_dioType;



/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



const ciaaDriverDio_dioType ciaaDriverDioLpc54102Inputs[]  = { {2, 0}, {2, 1}, {2, 2}, {2, 3}, {3, 11}, {3, 12}, {3, 13}, {3, 14} };

const ciaaDriverDio_dioType ciaaDriverDioLpc54102Outputs[] = { {5, 1}, {2, 6}, {2, 5}, {2, 4}, {5, 12}, {5, 13}, {5, 14}, {1, 8} };


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


static ciaaDevices_deviceType * const ciaaDriverDioLpc54102DevicesList[] =
      {
            &ciaaDriverDioLpc54102InputDevice,
            &ciaaDriverDioLpc54102OutputDevice,
            NULL
      };



/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/



static void ciaaDriverDioLpc54102_registerDevices()
{
   uint8_t devIndex;

   for(devIndex = 0; ciaaDriverDioLpc54102DevicesList[devIndex] != NULL; devIndex++)
   {
      ciaaDioDevices_addDriver(ciaaDriverDioConst.devices[devIndex]);
   }
}


static void ciaaDriverDioLpc54102_hardwareInit(void)
{
   Chip_GPIO_Init(LPC_GPIO_PORT);

#if (BOARD == ciaa_nxp)
   /* Inputs */
   Chip_SCU_PinMux(4,0,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO2[0]  */
   Chip_SCU_PinMux(4,1,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO2[1]  */
   Chip_SCU_PinMux(4,2,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO2[2]  */
   Chip_SCU_PinMux(4,3,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO2[3]  */
   Chip_SCU_PinMux(7,3,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO3[11] */
   Chip_SCU_PinMux(7,4,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO3[12] */
   Chip_SCU_PinMux(7,5,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO3[13] */
   Chip_SCU_PinMux(7,6,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO3[14] */

   Chip_GPIO_SetDir(LPC_GPIO_PORT, 2,0xF, 0);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, 3, 0xF<<11, 0);

   /* MOSFETs */
   Chip_SCU_PinMux(4,8,MD_PUP|MD_EZI,FUNC4);  /* GPIO5[12] */
   Chip_SCU_PinMux(4,9,MD_PUP|MD_EZI,FUNC4);  /* GPIO5[13] */
   Chip_SCU_PinMux(4,10,MD_PUP|MD_EZI,FUNC4); /* GPIO5[14] */
   Chip_SCU_PinMux(1,5,MD_PUP|MD_EZI,FUNC0);  /* GPIO1[8]  */
   Chip_GPIO_SetDir(LPC_GPIO_PORT, 5,(1<<12)|(1<<13)|(1<<14),1);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, 1,(1<<8),1);
   Chip_GPIO_ClearValue(LPC_GPIO_PORT, 5,(1<<12)|(1<<13)|(1<<14));
   Chip_GPIO_ClearValue(LPC_GPIO_PORT, 1,(1<<8));

   /* Relays */
   Chip_SCU_PinMux(4,4,MD_PUP|MD_EZI,FUNC0); /* GPIO2[4] */
   Chip_SCU_PinMux(4,5,MD_PUP|MD_EZI,FUNC0); /* GPIO2[5] */
   Chip_SCU_PinMux(4,6,MD_PUP|MD_EZI,FUNC0); /* GPIO2[6] */
   Chip_SCU_PinMux(2,1,MD_PUP|MD_EZI,FUNC4); /* GPIO5[1] */
   Chip_GPIO_SetDir(LPC_GPIO_PORT, 2,(1<<4)|(1<<5)|(1<<6),1);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, 5,(1<<1),1);
   Chip_GPIO_ClearValue(LPC_GPIO_PORT, 2,(1<<4)|(1<<5)|(1<<6));
   Chip_GPIO_ClearValue(LPC_GPIO_PORT, 5,(1<<1));

   /* LV-TTL GPIOs (not used yet) */
   Chip_SCU_PinMux(6,1,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO0/P6_1/GPIO3[0] */
   Chip_SCU_PinMux(2,5,MD_PUP|MD_EZI|MD_ZI,FUNC4); /* GPIO1/P2_5/GPIO5[5] */
}


static void ciaaDriverDioLpc54102_writeOutput(uint32_t outputNumber, uint32_t value)
{
   if (outputNumber < ciaaDriverDio_OutputCount)
   {
      Chip_GPIO_SetPinState(LPC_GPIO_PORT,
            ciaaDriverDio_Outputs[outputNumber].port,
            ciaaDriverDio_Outputs[outputNumber].pin,
            value != 0 ? 1 : 0);
   }
}


static int32_t ciaaDriverDioLpc54102_readInput(uint32_t inputNumber)
{
   int32_t rv = -1;

   if (inputNumber < ciaaDriverDio_InputCount)
   {
      rv = Chip_GPIO_GetPinState(LPC_GPIO_PORT,
            ciaaDriverDio_Inputs[inputNumber].port,
            ciaaDriverDio_Inputs[inputNumber].pin) ? 1 : 0;
   }

   return rv;
}


static int32_t ciaaDriverDioLpc54102_readOutput(uint32_t outputNumber)
{
   int32_t rv = -1;

   if (outputNumber < ciaaDriverDio_OutputCount)
   {
      rv = Chip_GPIO_GetPinState(LPC_GPIO_PORT,
            ciaaDriverDio_Outputs[outputNumber].port,
            ciaaDriverDio_Outputs[outputNumber].pin) ? 1 : 0;
   }

   return rv;
}



static int32_t ciaaDriverDioLpc54102_readPins(int32_t pinCount, uint8_t * buffer, size_t size, int32_t (*readFunction)(uint32_t))
{
   int32_t count, i, j;
   /* amount of bytes necessary to store all input states */
   count = pinCount >> 3; /* ciaaDriverDio_InputCount / 8 */
   if( (pinCount & 0x07) != 0) /* (ciaaDriverDio_InputCount % 8) != 0 */
   {
      count += 1;
   }
   /* adjust gpios to read according to provided buffer length */
   if(count > size)
   {
      count = size;
   }
   /* read and store all inputs in user buffer */
   ciaaPOSIX_memset(buffer, 0, count);
   for(i = 0, j = 0; (i < pinCount) && (j < count); i++)
   {
      if((i > 0) && ((i & 0x07)==0))
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

   if(device == ciaaDioDevices[0])
   {
      /* accessing to inputs */
      ret = ciaa_lpc4337_readPins(ciaaDriverDio_InputCount, buffer, size, ciaa_lpc4337_readInput);

   } else {

      if(device == ciaaDioDevices[1])
      {
         /* accessing to outputs */
         ret = ciaa_lpc4337_readPins(ciaaDriverDio_OutputCount, buffer, size, ciaa_lpc4337_readOutput);

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
      if(device == ciaaDioDevices[0])
      {
         /* Inputs can't be written. */
         ret = -1;

      } else {

         if(device == ciaaDioDevices[1])
         {

            /* set outputs according to bits defined in user buffer */
            for(i = 0, j = 0; (i < ciaaDriverDio_OutputCount) && (j < size); i++)
            {
               if( (i > 0) && ((i & 0x07) == 0) )
               {
                  j++;
               }
               ciaa_lpc4337_writeOutput(i, buffer[j] & (1 << (i - 8 * j)));
            }

            if((ciaaDriverDio_OutputCount & 0x07) != 0)
            {
               j++;
            }
            ret = j;

         } else {

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
