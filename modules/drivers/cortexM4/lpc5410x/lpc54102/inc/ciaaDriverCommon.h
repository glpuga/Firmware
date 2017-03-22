/* Copyright 2017, Gerardo Puga
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

#ifndef _CIAADRIVERCOMMON_H_
#define _CIAADRIVERCOMMON_H_

/** \brief Internal Header file of DIO Driver
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Drivers CIAA Drivers
 ** @{ */




/*==================[inclusions]=============================================*/



#include "ciaaPOSIX_stdint.h"



/*==================[cplusplus]==============================================*/



#ifdef __cplusplus
extern "C" {
#endif



/*==================[macros]=================================================*/



#define CIAA_DRIVER_COMMON_LPC54102_INTERNAL_BUFFER_SIZE 16


#define ciaaDriverCriticalSectionEntry()     SuspendAllInterrupts()

#define ciaaDriverCriticalSectionExit()      ResumeAllInterrupts()



/*==================[typedef]================================================*/



typedef struct {

   uint32_t circularQueue[CIAA_DRIVER_COMMON_LPC54102_INTERNAL_BUFFER_SIZE];

   uint32_t head;

   uint32_t tail;

} ciaaDriverCommonLpc54102InternalBufferType;



/*==================[external data declaration]==============================*/



/*==================[external functions declaration]=========================*/



uint32_t ciaaDriverCommonLpc54102_determineInputPinMode(uint32_t port, uint32_t pin);

uint32_t ciaaDriverCommonLpc54102_determineOutputPinMode(uint32_t port, uint32_t pin);


uint32_t ciaaDriverCommonLpc54102_internalFifoNextIndex(uint32_t index);

void ciaaDriverCommonLpc54102_internalFifoClear(ciaaDriverCommonLpc54102InternalBufferType *fifo);

int32_t ciaaDriverCommonLpc54102_internalFifoIsEmpty(ciaaDriverCommonLpc54102InternalBufferType *fifo);

uint32_t ciaaDriverCommonLpc54102_internalFifoIsFull(ciaaDriverCommonLpc54102InternalBufferType *fifo);

void ciaaDriverCommonLpc54102_internalFifoPush(ciaaDriverCommonLpc54102InternalBufferType *fifo, uint32_t item);

int32_t ciaaDriverCommonLpc54102_internalFifoPop(ciaaDriverCommonLpc54102InternalBufferType *fifo);



/*==================[cplusplus]==============================================*/



#ifdef __cplusplus
}
#endif



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _CIAADRIVERCOMMON_H_ */
