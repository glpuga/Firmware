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



#include "ciaaDriverConfig.h"
#include "ciaaDriverCommon.h"
#include "chip.h"



/*==================[macros and definitions]=================================*/



/*
 * Standard configuration for INPUT GPIO pins.
 * */

/* MODE for Port 0, Pins 0 to 22 */
#define CIAA_DRIVER_COMMON_LPC54102_DEFAULT_INPUT_PIN_MODE_PIO0_00TO22     (IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_MODE_PULLUP)
/* MODE for Port 0, Pins 23 to 28 */
#define CIAA_DRIVER_COMMON_LPC54102_DEFAULT_INPUT_PIN_MODE_PIO0_23TO28     (IOCON_DIGITAL_EN | IOCON_INPFILT_ON )
/* MODE for Port 0, Pins 29 to 31 */
#define CIAA_DRIVER_COMMON_LPC54102_DEFAULT_INPUT_PIN_MODE_PIO0_29TO31     (IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_MODE_PULLUP)

/* MODE for Port 1, Pins 0 to 08 */
#define CIAA_DRIVER_COMMON_LPC54102_DEFAULT_INPUT_PIN_MODE_PIO1_00TO08     (IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_MODE_PULLUP)
/* MODE for Port 1, Pins 9 to 17 */
#define CIAA_DRIVER_COMMON_LPC54102_DEFAULT_INPUT_PIN_MODE_PIO1_09TO17     (IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_MODE_PULLUP)


/*
 * Standard configuration for OUPTUT GPIO pins.
 * */

/* MODE for Port 0, Pins 0 to 22 */
#define CIAA_DRIVER_COMMON_LPC54102_DEFAULT_OUTPUT_PIN_MODE_PIO0_00TO22     (IOCON_DIGITAL_EN)
/* MODE for Port 0, Pins 23 to 28 */
#define CIAA_DRIVER_COMMON_LPC54102_DEFAULT_OUTPUT_PIN_MODE_PIO0_23TO28     (IOCON_DIGITAL_EN)
/* MODE for Port 0, Pins 29 to 31 */
#define CIAA_DRIVER_COMMON_LPC54102_DEFAULT_OUTPUT_PIN_MODE_PIO0_29TO31     (IOCON_DIGITAL_EN)

/* MODE for Port 1, Pins 0 to 08 */
#define CIAA_DRIVER_COMMON_LPC54102_DEFAULT_OUTPUT_PIN_MODE_PIO1_00TO08     (IOCON_DIGITAL_EN)
/* MODE for Port 1, Pins 9 to 17 */
#define CIAA_DRIVER_COMMON_LPC54102_DEFAULT_OUTPUT_PIN_MODE_PIO1_09TO17     (IOCON_DIGITAL_EN)



/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/



/*==================[external functions definition]==========================*/



uint32_t ciaaDriverCommonLpc54102_determineInputPinMode(uint32_t port, uint32_t pin)
{
   uint32_t mode;

   mode = 0;

   if (port == 0)
   {
      if ((pin >= 0) && (pin <= 22))
      {
         mode = CIAA_DRIVER_COMMON_LPC54102_DEFAULT_INPUT_PIN_MODE_PIO0_00TO22;

      } else {

         if ((pin >= 23) && (pin <= 28))
         {
            mode = CIAA_DRIVER_COMMON_LPC54102_DEFAULT_INPUT_PIN_MODE_PIO0_23TO28;

         } else {

            if ((pin >= 29) && (pin <= 31))
            {
               mode = CIAA_DRIVER_COMMON_LPC54102_DEFAULT_INPUT_PIN_MODE_PIO0_29TO31;
            }
         }
      }
   }


   if (port == 1)
   {
      if ((pin >= 0) && (pin <= 8))
      {
         mode = CIAA_DRIVER_COMMON_LPC54102_DEFAULT_INPUT_PIN_MODE_PIO1_00TO08;

      } else {

         if ((pin >= 9) && (pin <= 17))
         {
            mode = CIAA_DRIVER_COMMON_LPC54102_DEFAULT_INPUT_PIN_MODE_PIO1_09TO17;

         }
      }
   }

   return mode;
}


uint32_t ciaaDriverCommonLpc54102_determineOutputPinMode(uint32_t port, uint32_t pin)
{
   uint32_t mode;

   mode = 0;

   if (port == 0)
   {
      if ((pin >= 0) && (pin <= 22))
      {
         mode = CIAA_DRIVER_COMMON_LPC54102_DEFAULT_OUTPUT_PIN_MODE_PIO0_00TO22;

      } else {

         if ((pin >= 23) && (pin <= 28))
         {
            mode = CIAA_DRIVER_COMMON_LPC54102_DEFAULT_OUTPUT_PIN_MODE_PIO0_23TO28;

         } else {

            if ((pin >= 29) && (pin <= 31))
            {
               mode = CIAA_DRIVER_COMMON_LPC54102_DEFAULT_OUTPUT_PIN_MODE_PIO0_29TO31;
            }
         }
      }
   }


   if (port == 1)
   {
      if ((pin >= 0) && (pin <= 8))
      {
         mode = CIAA_DRIVER_COMMON_LPC54102_DEFAULT_OUTPUT_PIN_MODE_PIO1_00TO08;

      } else {

         if ((pin >= 9) && (pin <= 17))
         {
            mode = CIAA_DRIVER_COMMON_LPC54102_DEFAULT_OUTPUT_PIN_MODE_PIO1_09TO17;

         }
      }
   }

   return mode;
}



uint32_t ciaaDriverCommonLpc54102_internalFifoNextIndex(uint32_t index)
{
   int32_t incrementedIndex;

   incrementedIndex = index + 1;

   if (incrementedIndex >= CIAA_DRIVER_COMMON_LPC54102_INTERNAL_BUFFER_SIZE)
   {
      incrementedIndex = incrementedIndex - CIAA_DRIVER_COMMON_LPC54102_INTERNAL_BUFFER_SIZE;
   }

   return incrementedIndex;
}


void ciaaDriverCommonLpc54102_internalFifoClear(ciaaDriverCommonLpc54102InternalBufferType *fifo)
{
   fifo->head = 0;

   fifo->tail = 0;
}


int32_t ciaaDriverCommonLpc54102_internalFifoIsEmpty(ciaaDriverCommonLpc54102InternalBufferType *fifo)
{
   return (fifo->head == fifo->tail) ? 1 : 0;
}


uint32_t ciaaDriverCommonLpc54102_internalFifoIsFull(ciaaDriverCommonLpc54102InternalBufferType *fifo)
{
   uint32_t nextTail;

   nextTail = ciaaDriverCommonLpc54102_internalFifoNextIndex(fifo->tail);

   return (fifo->head == nextTail) ? 1 : 0;
}


void ciaaDriverCommonLpc54102_internalFifoPush(ciaaDriverCommonLpc54102InternalBufferType *fifo, uint32_t item)
{
   /*
    * You must check whether the FIFO is full before calling this function.
    * */

   fifo->circularQueue[fifo->tail] = item;

   fifo->tail = ciaaDriverCommonLpc54102_internalFifoNextIndex(fifo->tail);
}


int32_t ciaaDriverCommonLpc54102_internalFifoPop(ciaaDriverCommonLpc54102InternalBufferType *fifo)
{
   uint32_t oldHead;

   /*
    * You must check whether the FIFO is empty before calling this function.
    * */

   oldHead = fifo->head;

   fifo->head = ciaaDriverCommonLpc54102_internalFifoNextIndex(fifo->head);

   return fifo->circularQueue[oldHead];
}





/*==================[interrupt handlers]=====================================*/



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
