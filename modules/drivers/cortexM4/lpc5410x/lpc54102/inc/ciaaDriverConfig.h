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

#ifndef _CIAADRIVERCONFIG_H_
#define _CIAADRIVERCONFIG_H_

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



/*
 * UART DRIVER
 * */

#define CIAA_DRIVER_USART_LPC54102_DEFAULT_BAUDRATE         (9600)

#define CIAA_DRIVER_USART_LPC54102_DEFAULT_CONFIG           (UART_CFG_DATALEN_8 | UART_CFG_STOPLEN_1 | UART_CFG_PARITY_NONE)



/*
 * PWM DRIVER
 * */

#define CIAA_DRIVER_PWM_LPC54102_PWM_RATE                   (1000)

#define CIAA_DRIVER_PWM_LPC54102_SERVO_RATE                 (50)



/*
 * PWM DRIVER
 * */

#define CIAA_DRIVER_AIO_LPC54102_ADC_CLOCK_RATE             (10000000)



/*==================[typedef]================================================*/



/*==================[external data declaration]==============================*/



/*==================[external functions declaration]=========================*/



/*==================[cplusplus]==============================================*/



#ifdef __cplusplus
}
#endif



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _CIAADRIVERCONFIG_H_ */
