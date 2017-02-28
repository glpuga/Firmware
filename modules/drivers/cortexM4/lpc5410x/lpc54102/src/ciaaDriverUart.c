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



#include "ciaaDriverUart.h"
#include "chip.h"



/*==================[macros and definitions]=================================*/


#define CIAA_DRIVER_USART_LPC54102_DEFAULT_BAUDRATE   (9600)

#define CIAA_DRIVER_USART_LPC54102_DEFAULT_CONFIG     (UART_CFG_DATALEN_8 | UART_CFG_STOPLEN_1 | UART_CFG_PARITY_NONE)


#define CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK  (LPC_PERIPFIFO_INT_TXTH)

#define CIAA_DRIVER_USART_LPC54102_RX_INTERRUPT_MASK  (LPC_PERIPFIFO_INT_RXTH | LPC_PERIPFIFO_INT_RXTIMEOUT)


typedef struct {

   int32_t usartIndex;

   LPC_USART_T lpcDevice;

   const char *posixDevice;

   int32_t txFifoSize;
   int32_t rxFifoSize;

   int32_t defaultBaudrate;
   uint32_t defaultConfig;

   IRQn_Type lpcNvicInterrupt;

   uint32_t lpcIoconPort;
   uint32_t lpcIoconPin;
   uint32_t lpcIoconMode;
   uint32_t lpcIoconFunc;

} ciaaDriverUartLpc54102DeviceDescriptiontype;


/*==================[internal data declaration]==============================*/


const ciaaDriverUartLpc54102DeviceDescriptiontype deviceDescriptionTable[] =
      {
            {
                  0,                                              /* usartIndex       */
                  LPC_USART0,                                     /* lpcDevice        */
                  "uart/0",                                       /* posixDevice      */
                  8,                                              /* txFifoSize       */
                  8,                                              /* rxFifoSize       */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_BAUDRATE,    /* defaultBaudrate  */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_CONFIG,      /* defualtConfig    */
                  UART0_IRQn,                                     /* lpcNvicInterrupt */
                  0,                                              /* lpcIoconPort     */
                  0,                                              /* lpcIoconPin      */
                  0,                                              /* lpcIoconMode     */
                  0                                               /* lpcIoconFunc     */
            },
            {
                  1,                                              /* usartIndex       */
                  LPC_USART0,                                     /* lpcDevice        */
                  "uart/1",                                       /* posixDevice      */
                  8,                                              /* txFifoSize       */
                  8,                                              /* rxFifoSize       */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_BAUDRATE,    /* defaultBaudrate  */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_CONFIG,      /* defualtConfig    */
                  UART1_IRQn,                                     /* lpcNvicInterrupt */
                  0,                                              /* lpcIoconPort     */
                  0,                                              /* lpcIoconPin      */
                  0,                                              /* lpcIoconMode     */
                  0                                               /* lpcIoconFunc     */
            },
            {
                  -1,                                             /* usartIndex       */
                  NULL,                                           /* lpcDevice        */
                  NULL,                                           /* posixDevice      */
                  0,                                              /* txFifoSize       */
                  0,                                              /* rxFifoSize       */
                  0,                                              /* defaultBaudrate  */
                  0,                                              /* defualtConfig    */
                  0,                                              /* lpcNvicInterrupt */
                  0,                                              /* lpcIoconPort     */
                  0,                                              /* lpcIoconPin      */
                  0,                                              /* lpcIoconMode     */
                  0                                               /* lpcIoconFunc     */
            }/* End-of-table entry */
      };


/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/


void ciaaDriverUartLpc54102_init()
{
   LPC_FIFO_CFGSIZE_T fifoSizes;

   int32_t devIndex;

   /*
    * The USART needs to use watchdog peripheral clock to generate RX timeouts.
    * */
   Chip_WWDT_Init(LPC_WWDT);

   /* Datasheet instructions for configuring the FIFO USART
    *
    * 1. Pause the desired directions (transmit and/or receive) of the desired peripheral
    *    types (USART and/or SPI).
    * 2. Wait for that function to reach the paused and empty state.
    * 3. Write the FIFO configurations for all of the desired directions and peripheral types.
    * 4. Reset the FIFOs for all of the desired directions and peripheral types.
    * 5. Un-pause the desired directions and peripheral types.
    *
    * */

   Chip_FIFO_Init(LPC_FIFO);

   /*
    * Pause the FIFOs before configuration. Probably not needed after resetting
    * the devices, but playing it on the safe side.
    * */

   Chip_FIFO_PauseFifo(LPC_FIFO, FIFO_USART, FIFO_TX);
   Chip_FIFO_PauseFifo(LPC_FIFO, FIFO_USART, FIFO_RX);

   /*
    * Prepare the configuration for the devices listed on the device description table.
    * */

   for (devIndex = 0; deviceDescriptionTable[devIndex < 4; devIndex++)
   {
      fifoSizes.fifoTXSize[devIndex] = 0;
      fifoSizes.fifoRXSize[devIndex] = 0;
   }

   for (devIndex = 0; (devIndex < 4) && (deviceDescriptionTable[devIndex].usartIndex >= 0); devIndex++)
   {
      fifoSizes.fifoTXSize[deviceDescriptionTable[devIndex].usartIndex] = deviceDescriptionTable[devIndex].txFifoSize;
      fifoSizes.fifoRXSize[deviceDescriptionTable[devIndex].usartIndex] = deviceDescriptionTable[devIndex].rxFifoSize;
   }

   /*
    * Configure and update the new USART FIFO sizes and un-pause them.
    * */

   Chip_FIFO_ConfigFifoSize(LPC_FIFO, FIFO_USART, fifoSizes);

   /*
    * Configure the initial FIFO USART interrupt setup.
    * */

   for (devIndex = 0; deviceDescriptionTable[devIndex].usartIndex >= 0; devIndex++)
   {s
      Chip_FIFOUSART_EnableInts(
            LPC_FIFO,
            deviceDescriptionTable[devIndex].usartIndex,
            CIAA_DRIVER_USART_LPC54102_RX_INTERRUPT_MASK);
   }

   Chip_FIFO_UnpauseFifo(LPC_FIFO, FIFO_USART, FIFO_TX);
   Chip_FIFO_UnpauseFifo(LPC_FIFO, FIFO_USART, FIFO_RX);


   /*
    * USART specific configuration
    *
    * */

   /* Datasheet instructions for configuring the USART
    *
    * If using the USARTs with FIFO support, configure the FIFOs, see Chapter 24.
    *
    * Configure USARTs for receiving and transmitting data:
    * 1) In the ASYNCAPBCLKCTRL register, set bit 1 to 4 (Table 93) to enable the clock to
    *    the register interface.
    * 2) Clear the USART0/1/2/3 peripheral resets using the ASYNCPRESETCTRL register
    *    (Table 90).
    * 3) Enable or disable the USART0/1/2/3 interrupts in slots #17 to 20 in the NVIC.
    * 4) Configure the USART0/1/2/3 pin functions via IOCON, see Chapter 7.
    * 5) Configure the USART clock and baud rate. See Section 21.3.1.
    * 6) Send and receive lines are connected to DMA request lines. See Table 176.
    *
    * */

   for (devIndex = 0; deviceDescriptionTable[devIndex].usartIndex >= 0; devIndex++)
   {
      Chip_UART_Init(deviceDescriptionTable[devIndex].lpcName);

      Chip_UART_ConfigData(
            deviceDescriptionTable[devIndex].lpcName,
            deviceDescriptionTable[devIndex].defaultConfig);

      /*
       * WARNING:
       *
       * All USART share the same fractional divider, so all the USARTs
       * will always run at the same baudrate, and the following line will
       * leave all of them configured with the baudrate of the last one
       * configured.
       * */
      Chip_UART_SetBaud(
            deviceDescriptionTable[devIndex].lpcName,
            deviceDescriptionTable[devIndex].defaultBaudrate);

      NVIC_EnableIRQ(
            deviceDescriptionTable[devIndex].lpcNvicInterrupt);

      Chip_IOCON_PinMux(
            LPC_IOCON,
            deviceDescriptionTable[devIndex].lpcIoconPort,
            deviceDescriptionTable[devIndex].lpcIoconPin,
            deviceDescriptionTable[devIndex].lpcIoconMode,
            deviceDescriptionTable[devIndex].lpcIoconFunc);
   }
}


/*==================[external functions definition]==========================*/



extern ciaaDevices_deviceType * ciaaDriverUart_open(char const * path, ciaaDevices_deviceType * device, uint8_t const oflag)
{
   return device;
}


extern int32_t ciaaDriverUart_close(ciaaDevices_deviceType const * const device)
{
   return -1;
}


extern int32_t ciaaDriverUart_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param)
{
   int32_t ret = -1;

   return ret;
}


extern ssize_t ciaaDriverUart_read(ciaaDevices_deviceType const * const device, uint8_t* buffer, size_t const size)
{
   ssize_t ret = -1;

   return ret;
}


extern ssize_t ciaaDriverUart_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size)
{
   ssize_t ret = -1;

   return ret;
}


void ciaaDriverUart_init(void)
{
   ciaaDriverUartLpc54102_init();
}



/*==================[interrupt handlers]=====================================*/



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

