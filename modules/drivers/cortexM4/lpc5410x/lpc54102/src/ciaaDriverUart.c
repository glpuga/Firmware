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
#include "ciaaDriverConfig.h"
#include "ciaaDriverCommon.h"
#include "ciaaPOSIX_stdlib.h"
#include "ciaaPOSIX_stdio.h"
#include "os.h"

#undef INLINE

#include "chip.h"



/*==================[macros and definitions]=================================*/



#define CIAA_DRIVER_USART_LPC54102_DEVICE_TABLE_SIZE        (sizeof(ciaaDriverUartLpc54102DeviceDescription) / sizeof(ciaaDriverUartLpc54102DeviceDescriptionType))

#define CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT          (4)

#define CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK        (LPC_PERIPFIFO_INT_TXTH)

#define CIAA_DRIVER_USART_LPC54102_RX_INTERRUPT_MASK        (LPC_PERIPFIFO_INT_RXTH | LPC_PERIPFIFO_INT_RXTIMEOUT)



typedef struct {

   int32_t usartIndex;

   char const * posixName;

   int32_t txFifoSize;
   int32_t rxFifoSize;

   int32_t defaultBaudrate;
   uint32_t defaultConfig;

   uint32_t lpcIoconTxPort;
   uint32_t lpcIoconTxPin;
   uint32_t lpcIoconTxMode;
   uint32_t lpcIoconTxFunc;

   uint32_t lpcIoconRxPort;
   uint32_t lpcIoconRxPin;
   uint32_t lpcIoconRxMode;
   uint32_t lpcIoconRxFunc;

} ciaaDriverUartLpc54102DeviceDescriptionType;



/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



const ciaaDriverUartLpc54102DeviceDescriptionType ciaaDriverUartLpc54102DeviceDescription[] =
      {
            {
                  1,                                              /* usartIndex       */
                  "uart/0",                                       /* posixName        */
                  8,                                              /* txFifoSize       */
                  8,                                              /* rxFifoSize       */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_BAUDRATE,    /* defaultBaudrate  */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_CONFIG,      /* defaultConfig    */
                  0,                                              /* lpcIoconTxPort     */
                  6,                                              /* lpcIoconTxPin      */
                  (IOCON_DIGITAL_EN),                             /* lpcIoconTxMode     */
                  (IOCON_FUNC1),                                  /* lpcIoconTxFunc     */
                  0,                                              /* lpcIoconRxPort     */
                  5,                                              /* lpcIoconRxPin      */
                  (IOCON_DIGITAL_EN | IOCON_MODE_PULLUP),         /* lpcIoconRxMode     */
                  (IOCON_FUNC1)                                   /* lpcIoconRxFunc     */
            },
            {
                  2,                                              /* usartIndex       */
                  "uart/1",                                       /* posixName        */
                  4,                                              /* txFifoSize       */
                  4,                                              /* rxFifoSize       */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_BAUDRATE,    /* defaultBaudrate  */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_CONFIG,      /* defaultConfig    */
                  0,                                              /* lpcIoconTxPort     */
                  9,                                              /* lpcIoconTxPin      */
                  (IOCON_DIGITAL_EN),                             /* lpcIoconTxMode     */
                  (IOCON_FUNC1),                                  /* lpcIoconTxFunc     */
                  0,                                              /* lpcIoconRxPort     */
                  8,                                              /* lpcIoconRxPin      */
                  (IOCON_DIGITAL_EN | IOCON_MODE_PULLUP),         /* lpcIoconRxMode     */
                  (IOCON_FUNC1)                                   /* lpcIoconRxFunc     */
            },
            {
                  0,                                              /* usartIndex       */
                  "console/0",                                    /* posixName        */
                  4,                                              /* txFifoSize       */
                  4,                                              /* rxFifoSize       */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_BAUDRATE,    /* defaultBaudrate  */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_CONFIG,      /* defaultConfig    */
                  0,                                              /* lpcIoconTxPort     */
                  1,                                              /* lpcIoconTxPin      */
                  (IOCON_DIGITAL_EN),                             /* lpcIoconTxMode     */
                  (IOCON_FUNC1),                                  /* lpcIoconTxFunc     */
                  0,                                              /* lpcIoconRxPort     */
                  0,                                              /* lpcIoconRxPin      */
                  (IOCON_DIGITAL_EN | IOCON_MODE_PULLUP),         /* lpcIoconRxMode     */
                  (IOCON_FUNC1)                                   /* lpcIoconRxFunc     */
            }
      };


ciaaDriverCommonLpc54102InternalBufferType ciaaDriverUartLpc54102InternalTxBuffers[CIAA_DRIVER_USART_LPC54102_DEVICE_TABLE_SIZE];
ciaaDriverCommonLpc54102InternalBufferType ciaaDriverUartLpc54102InternalRxBuffers[CIAA_DRIVER_USART_LPC54102_DEVICE_TABLE_SIZE];

ciaaDevices_deviceType ciaaDriverUartLpc54102PosixRegistrationDataTable[CIAA_DRIVER_USART_LPC54102_DEVICE_TABLE_SIZE];

int32_t ciaaDriverUartLpc54102Uart2devIndexMap[CIAA_DRIVER_USART_LPC54102_DEVICE_TABLE_SIZE];



/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/


inline IRQn_Type ciaaDriverUartLpc54102_getLpcIrqIdFromUsartIndex(int32_t usartIndex)
{
   IRQn_Type lpcDeviceIrqId;

   lpcDeviceIrqId = UART0_IRQn;

   switch (usartIndex) {

      case 0 :
         lpcDeviceIrqId = UART0_IRQn;
         break;

      case 1 :
         lpcDeviceIrqId = UART1_IRQn;
         break;

      case 2 :
         lpcDeviceIrqId = UART2_IRQn;
         break;

      case 3 :
         lpcDeviceIrqId = UART3_IRQn;
         break;

      default :
         lpcDeviceIrqId = UART0_IRQn;
         break;
   }

   return lpcDeviceIrqId;
}


inline LPC_USART_T *ciaaDriverUartLpc54102_getLpcDeviceIdFromUsartIndex(int32_t usartIndex)
{
   LPC_USART_T *lpcDeviceId;

   lpcDeviceId = LPC_USART0;

   switch (usartIndex) {

      case 0 :
         lpcDeviceId = LPC_USART0;
         break;

      case 1 :
         lpcDeviceId = LPC_USART1;
         break;

      case 2 :
         lpcDeviceId = LPC_USART2;
         break;

      case 3 :
         lpcDeviceId = LPC_USART3;
         break;

      default :
         lpcDeviceId = LPC_USART0;
         break;
   }

   return lpcDeviceId;
}


inline uint32_t ciaaDriverUartLpc54102_EnabledInts(LPC_FIFO_T *pFIFO, int usartIndex)
{
   /*
    * This function is not provided in LPCOpen, so I made it.
    * */

   return pFIFO->usart[usartIndex].CTLSET;
}


void ciaaDriverUartLpc54102_InitializeControlStructures()
{
   LPC_USART_T *lpcDeviceId;
   int32_t usartIndex;
   int32_t devIndex;

   for (usartIndex = 0; usartIndex < CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT; usartIndex++)
   {
      ciaaDriverUartLpc54102Uart2devIndexMap[usartIndex] = 0;
   }

   for (devIndex = 0; devIndex < CIAA_DRIVER_USART_LPC54102_DEVICE_TABLE_SIZE; devIndex++)
   {
      usartIndex  = ciaaDriverUartLpc54102DeviceDescription[devIndex].usartIndex;

      lpcDeviceId = ciaaDriverUartLpc54102_getLpcDeviceIdFromUsartIndex(usartIndex);

      /*
       * Build the backwards map from usartIndex to devIndex
       * */

      ciaaDriverUartLpc54102Uart2devIndexMap[usartIndex] = devIndex;

      /*
       * Internal RX buffer
       * */

      ciaaDriverCommonLpc54102_internalFifoClear(&ciaaDriverUartLpc54102InternalTxBuffers[devIndex]);
      ciaaDriverCommonLpc54102_internalFifoClear(&ciaaDriverUartLpc54102InternalRxBuffers[devIndex]);

      /*
       * POSIX device information information
       * */

      ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex].path    = ciaaDriverUartLpc54102DeviceDescription[devIndex].posixName;

      ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex].open    = ciaaDriverUart_open;
      ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex].close   = ciaaDriverUart_close;
      ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex].read    = ciaaDriverUart_read;
      ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex].write   = ciaaDriverUart_write;
      ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex].ioctl   = ciaaDriverUart_ioctl;
      ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex].lseek   = NULL;

      ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex].upLayer = NULL;
      ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex].layer   = (void *)&ciaaDriverUartLpc54102DeviceDescription[devIndex];
      ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex].loLayer = (void *)lpcDeviceId;
   }
}


void ciaaDriverUartLpc54102_hardwareInit()
{
   LPC_FIFO_CFGSIZE_T fifoSizes;
   LPC_FIFO_CFG_T fifoConfig;
   LPC_USART_T *lpcDeviceId;

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
    * the devices, but it's better to fail on the safe side...
    * */

   Chip_FIFO_PauseFifo(LPC_FIFO, FIFO_USART, FIFO_TX);
   Chip_FIFO_PauseFifo(LPC_FIFO, FIFO_USART, FIFO_RX);

   /*
    * Prepare the configuration for the devices listed on the device description table.
    * */

   for (devIndex = 0; devIndex < 4; devIndex++)
   {
      fifoSizes.fifoTXSize[devIndex] = 0;
      fifoSizes.fifoRXSize[devIndex] = 0;
   }

   for (devIndex = 0; (devIndex < CIAA_DRIVER_USART_LPC54102_DEVICE_TABLE_SIZE) && (devIndex < 4); devIndex++)
   {
      fifoSizes.fifoTXSize[ciaaDriverUartLpc54102DeviceDescription[devIndex].usartIndex] = ciaaDriverUartLpc54102DeviceDescription[devIndex].txFifoSize;
      fifoSizes.fifoRXSize[ciaaDriverUartLpc54102DeviceDescription[devIndex].usartIndex] = ciaaDriverUartLpc54102DeviceDescription[devIndex].rxFifoSize;
   }

   /*
    * Configure and update the new USART FIFO sizes.
    * */

   Chip_FIFO_ConfigFifoSize(LPC_FIFO, FIFO_USART, &fifoSizes);

   /*
    * Configure the initial FIFO USART interrupt setup.
    * */

   for (devIndex = 0; devIndex < CIAA_DRIVER_USART_LPC54102_DEVICE_TABLE_SIZE; devIndex++)
   {

      fifoConfig.rxThreshold = ciaaDriverUartLpc54102DeviceDescription[devIndex].rxFifoSize / 2;
      fifoConfig.txThreshold = ciaaDriverUartLpc54102DeviceDescription[devIndex].txFifoSize / 2;

      fifoConfig.noTimeoutContWrite = 1;
      fifoConfig.noTimeoutContEmpty = 0;

      /*
       * The following constants select an RX timeout value
       * of about 20 ms.
       * */
      fifoConfig.timeoutBase  = 10;
      fifoConfig.timeoutValue = 10;

      Chip_FIFOUSART_Configure(
            LPC_FIFO,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].usartIndex,
            &fifoConfig);

      Chip_FIFOUSART_EnableInts(
            LPC_FIFO,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].usartIndex,
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

   for (devIndex = 0; devIndex < CIAA_DRIVER_USART_LPC54102_DEVICE_TABLE_SIZE; devIndex++)
   {
      lpcDeviceId = ciaaDriverUartLpc54102_getLpcDeviceIdFromUsartIndex(ciaaDriverUartLpc54102DeviceDescription[devIndex].usartIndex);

      Chip_UART_Init(lpcDeviceId);

      Chip_UART_ConfigData(
            lpcDeviceId,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].defaultConfig);

      /*
       * WARNING:
       *
       * All USARTs share the same fractional divider, so all the USARTs
       * will always run at the same baudrate, and the following line will
       * leave all of them configured with the baudrate of the last one.
       * */
      Chip_UART_SetBaud(
            lpcDeviceId,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].defaultBaudrate);

      NVIC_EnableIRQ(
            ciaaDriverUartLpc54102_getLpcIrqIdFromUsartIndex(ciaaDriverUartLpc54102DeviceDescription[devIndex].usartIndex));

      Chip_IOCON_PinMux(
            LPC_IOCON,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].lpcIoconTxPort,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].lpcIoconTxPin,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].lpcIoconTxMode,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].lpcIoconTxFunc);

      Chip_IOCON_PinMux(
            LPC_IOCON,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].lpcIoconRxPort,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].lpcIoconRxPin,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].lpcIoconRxMode,
            ciaaDriverUartLpc54102DeviceDescription[devIndex].lpcIoconRxFunc);
   }
}


void ciaaDriverUartLpc54102_registerDevices()
{
   int32_t devIndex;

   for (devIndex = 0; devIndex < CIAA_DRIVER_USART_LPC54102_DEVICE_TABLE_SIZE; devIndex++)
   {
      ciaaSerialDevices_addDriver(&ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex]);
   }
}


static void ciaaDriverUartLpc54102_rxIndication(ciaaDevices_deviceType const * const device)
{
   /* receive the data and forward to upper layer */
   ciaaSerialDevices_rxIndication(device->upLayer, 1);
}


static void ciaaDriverUartLpc54102_txConfirmation(ciaaDevices_deviceType const * const device)
{
   /* receive the data and forward to upper layer */
   ciaaSerialDevices_txConfirmation(device->upLayer, 1);
}


void ciaaDriverUartLpc54102_unifiedIRQn(int32_t usartIndex)
{
   ciaaDriverCommonLpc54102InternalBufferType *rxBufferPtr;
   ciaaDriverCommonLpc54102InternalBufferType *txBufferPtr;
   uint32_t uartStatus;
   int32_t devIndex;
   int32_t usartTxBufferIsFull;
   uint8_t dataByte;

   devIndex = ciaaDriverUartLpc54102Uart2devIndexMap[usartIndex];

   txBufferPtr = &ciaaDriverUartLpc54102InternalTxBuffers[devIndex];
   rxBufferPtr = &ciaaDriverUartLpc54102InternalRxBuffers[devIndex];

   /* *** */

   uartStatus = Chip_FIFOUSART_GetStatus(LPC_FIFO, usartIndex);

   if((uartStatus & LPC_PERIPFIFO_STAT_RXEMPTY) == 0)
   {
      do {

         Chip_FIFOUSART_ReadRX(LPC_FIFO, usartIndex, true, (void*)&dataByte, 1);

         ciaaDriverCommonLpc54102_internalFifoPush(rxBufferPtr, dataByte);

         uartStatus = Chip_FIFOUSART_GetStatus(LPC_FIFO, usartIndex);

      } while ((ciaaDriverCommonLpc54102_internalFifoIsFull(rxBufferPtr) == 0) && ((uartStatus & LPC_PERIPFIFO_STAT_RXEMPTY) == 0));

      ciaaDriverUartLpc54102_rxIndication(&ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex]);
   }

   /* *** */

   if ((ciaaDriverUartLpc54102_EnabledInts(LPC_FIFO, usartIndex) & CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK) != 0)
   {

      usartTxBufferIsFull = 0;

      ciaaDriverUartLpc54102_txConfirmation(&ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex]);

      while (ciaaDriverCommonLpc54102_internalFifoIsEmpty(txBufferPtr) == 0)
      {

         if (Chip_FIFOUSART_GetTxCount(LPC_FIFO, usartIndex) > 0)
         {
            /*
             * Extract a byte from the internal FIFO and push it into the UART TX
             * buffer.
             * */

            dataByte = ciaaDriverCommonLpc54102_internalFifoPop(txBufferPtr);

            Chip_FIFOUSART_WriteTX(LPC_FIFO, usartIndex, true, (void*)&dataByte, 1);

         } else {

            /*
             * The UART FIFO is full, we will not be able to write anything more.
             * Exit and wait for the next interrupt.
             * */

            usartTxBufferIsFull = 1;
            break;
         }

         /*
          * If the internal FIFO is empty, try to refill it from the POSIX layer
          * FIFO.
          * */

         if (ciaaDriverCommonLpc54102_internalFifoIsEmpty(txBufferPtr) != 0)
         {
            ciaaDriverUartLpc54102_txConfirmation(&ciaaDriverUartLpc54102PosixRegistrationDataTable[devIndex]);
         }
      }

      /*
       * If we left the previous loop because there was no more space left
       * on the UART TX FIFO, then we must keep the interrupts on. Otherwise
       * turn them off because there are no more bytes waiting to be sent.
       * */

      if(usartTxBufferIsFull == 0)
      {
         /* There are no more data to be sent */
         Chip_FIFOUSART_DisableInts(LPC_FIFO, usartIndex, CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK);
      }
   }
}



/*==================[external functions definition]==========================*/



extern ciaaDevices_deviceType * ciaaDriverUart_open(char const * path, ciaaDevices_deviceType * device, uint8_t const oflag)
{
   return device;
}


extern int32_t ciaaDriverUart_close(ciaaDevices_deviceType const * const device)
{
   return 0;
}


extern int32_t ciaaDriverUart_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param)
{
   ciaaDriverUartLpc54102DeviceDescriptionType *dev;
   ssize_t ret = -1;

   dev = (ciaaDriverUartLpc54102DeviceDescriptionType *)device->layer;

   switch(request)
   {
      case ciaaPOSIX_IOCTL_STARTTX:

         /* Enable FIFO TX Threshold interrupts */
         Chip_FIFOUSART_EnableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK);
         ret = 0;
         break;

      case ciaaPOSIX_IOCTL_SET_BAUDRATE:

         /*
          * WARNING: this will set the baudrate of ALL the USARTs at the same time,
          * since they all share the same fractional divider.
          * */
         Chip_UART_SetBaud(
               ciaaDriverUartLpc54102_getLpcDeviceIdFromUsartIndex(dev->usartIndex),
               (int32_t)param);
         ret = 0;
         break;

      case ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL:

         /*
          * Not implemented.
          * */
         ret = -1;
         break;

      case ciaaPOSIX_IOCTL_SET_ENABLE_TX_INTERRUPT:

         if((bool)(intptr_t)param == false)
         {
            /*
             * Disable TX interrupts.
             * */
            Chip_FIFOUSART_DisableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK);

         } else {

            /*
             * Enable TX interrupts.
             * */
            Chip_FIFOUSART_EnableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK);
         }
         ret = 0;
         break;

      case ciaaPOSIX_IOCTL_SET_ENABLE_RX_INTERRUPT:

         if((bool)(intptr_t)param == false)
         {
            /*
             * Disable RX interrupts.
             * */
            Chip_FIFOUSART_DisableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_RX_INTERRUPT_MASK);

         } else {

            /*
             * Enable RX interrupts.
             * */
            Chip_FIFOUSART_EnableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_RX_INTERRUPT_MASK);
         }
         ret = 0;
         break;
   }

   return ret;
}


extern ssize_t ciaaDriverUart_read(ciaaDevices_deviceType const * const device, uint8_t* buffer, size_t const size)
{
   ciaaDriverUartLpc54102DeviceDescriptionType *dev;
   ciaaDriverCommonLpc54102InternalBufferType *rxBufferPtr;
   int32_t devIndex;
   ssize_t ret = -1;

   dev = (ciaaDriverUartLpc54102DeviceDescriptionType *)device->layer;

   devIndex = ciaaDriverUartLpc54102Uart2devIndexMap[dev->usartIndex];

   rxBufferPtr = &ciaaDriverUartLpc54102InternalRxBuffers[devIndex];

   for (ret = 0; ret < size; ret++)
   {
      if (ciaaDriverCommonLpc54102_internalFifoIsEmpty(rxBufferPtr) == 0)
      {
         buffer[ret] = ciaaDriverCommonLpc54102_internalFifoPop(rxBufferPtr);

      } else {

         break;
      }
   }

   return ret;
}


extern ssize_t ciaaDriverUart_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size)
{
   ciaaDriverUartLpc54102DeviceDescriptionType *dev;
   ciaaDriverCommonLpc54102InternalBufferType *txBufferPtr;
   int32_t devIndex;
   ssize_t ret = -1;

   dev = (ciaaDriverUartLpc54102DeviceDescriptionType *)device->layer;

   devIndex = ciaaDriverUartLpc54102Uart2devIndexMap[dev->usartIndex];

   txBufferPtr = &ciaaDriverUartLpc54102InternalTxBuffers[devIndex];

   for (ret = 0; ret < size; ret++)
   {
      if (ciaaDriverCommonLpc54102_internalFifoIsFull(txBufferPtr) == 0)
      {
         ciaaDriverCommonLpc54102_internalFifoPush(txBufferPtr, buffer[ret]);

      } else {

         break;
      }
   }

   return ret;
}


void ciaaDriverUart_init(void)
{
   ciaaDriverUartLpc54102_InitializeControlStructures();

   ciaaDriverUartLpc54102_hardwareInit();

   ciaaDriverUartLpc54102_registerDevices();
}



/*==================[interrupt handlers]=====================================*/



ISR(UART0_IRQHandler)
{
   ciaaDriverUartLpc54102_unifiedIRQn(0);
}


ISR(UART1_IRQHandler)
{
   ciaaDriverUartLpc54102_unifiedIRQn(1);
}


ISR(UART2_IRQHandler)
{
   ciaaDriverUartLpc54102_unifiedIRQn(2);
}


ISR(UART3_IRQHandler)
{
   ciaaDriverUartLpc54102_unifiedIRQn(3);
}



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

