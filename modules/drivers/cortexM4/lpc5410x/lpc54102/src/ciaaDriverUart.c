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



#define CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT          (4)

#define CIAA_DRIVER_USART_LPC54102_DEFAULT_BAUDRATE         (9600)

#define CIAA_DRIVER_USART_LPC54102_DEFAULT_CONFIG           (UART_CFG_DATALEN_8 | UART_CFG_STOPLEN_1 | UART_CFG_PARITY_NONE)


#define CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK        (LPC_PERIPFIFO_INT_TXTH)

#define CIAA_DRIVER_USART_LPC54102_RX_INTERRUPT_MASK        (LPC_PERIPFIFO_INT_RXTH | LPC_PERIPFIFO_INT_RXTIMEOUT)


#define CIAA_DRIVER_USART_LPC54102_INTERNAL_BUFFER_SIZE  (16)


typedef struct {

   uint32_t circularQueue[CIAA_DRIVER_USART_LPC54102_INTERNAL_BUFFER_SIZE];

   uint32_t head;

   uint32_t tail;

} ciaaDriverUartLpc54102InternalBuffer;


typedef struct {

   int32_t usartIndex;

   LPC_USART_T lpcDevice;

   char const * posixName;

   int32_t txFifoSize;
   int32_t rxFifoSize;

   int32_t defaultBaudrate;
   uint32_t defaultConfig;

   IRQn_Type lpcNvicInterrupt;

   uint32_t lpcIoconTxPort;
   uint32_t lpcIoconTxPin;
   uint32_t lpcIoconTxMode;
   uint32_t lpcIoconTxFunc;

   uint32_t lpcIoconRxPort;
   uint32_t lpcIoconRxPin;
   uint32_t lpcIoconRxMode;
   uint32_t lpcIoconRxFunc;

   ciaaDriverUartLpc54102InternalBuffer *internalTxBufferPtr;
   ciaaDriverUartLpc54102InternalBuffer *internalRxBufferPtr;

   ciaaDevices_deviceType *posixDeviceDataPtr;

} ciaaDriverUartLpc54102DeviceDescriptiontype;



/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/



/*==================[internal data definition]===============================*/



ciaaDriverUartLpc54102InternalRxBuffer lpc54102InternalTxBuffers[CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT];
ciaaDriverUartLpc54102InternalRxBuffer lpc54102InternalRxBuffers[CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT];

ciaaDevices_deviceType lpc54102PosixRegistrationDataTable[CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT];

int32_t lpc54102Uart2devIndexMap[CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT];

ciaaDriverUartLpc54102DeviceDescriptiontype lpc54102DeviceControlStructures[] =
      {
            {
                  0,                                              /* usartIndex       */
                  LPC_USART0,                                     /* lpcDevice        */
                  "uart/0",                                       /* posixName        */
                  8,                                              /* txFifoSize       */
                  8,                                              /* rxFifoSize       */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_BAUDRATE,    /* defaultBaudrate  */
                  CIAA_DRIVER_USART_LPC54102_DEFAULT_CONFIG,      /* defualtConfig    */
                  UART0_IRQn,                                     /* lpcNvicInterrupt */
                  0,                                              /* lpcIoconTxPort     */
                  0,                                              /* lpcIoconTxPin      */
                  0,                                              /* lpcIoconTxMode     */
                  0,                                              /* lpcIoconTxFunc     */
                  0,                                              /* lpcIoconRxPort     */
                  0,                                              /* lpcIoconRxPin      */
                  0,                                              /* lpcIoconRxMode     */
                  0,                                              /* lpcIoconRxFunc     */
                  NULL,                                           /* internalTxBuffer, will be filled later. */
                  NULL,                                           /* internalRxBuffer, will be filled later. */
                  NULL                                            /* ciaaDevices_deviceType, will be filled later. */
            },
            {
                  -1,                                             /* usartIndex       */
                  0,                                              /* lpcDevice        */
                  NULL,                                           /* posixName        */
                  0,                                              /* txFifoSize       */
                  0,                                              /* rxFifoSize       */
                  0,                                              /* defaultBaudrate  */
                  0,                                              /* defualtConfig    */
                  0,                                              /* lpcNvicInterrupt */
                  0,                                              /* lpcIoconTxPort     */
                  0,                                              /* lpcIoconTxPin      */
                  0,                                              /* lpcIoconTxMode     */
                  0,                                              /* lpcIoconTxFunc     */
                  0,                                              /* lpcIoconRxPort     */
                  0,                                              /* lpcIoconRxPin      */
                  0,                                              /* lpcIoconRxMode     */
                  0,                                              /* lpcIoconRxFunc     */
                  NULL,                                           /* internalTxBuffer, will be filled later. */
                  NULL,                                           /* internalRxBuffer, will be filled later. */
                  NULL                                            /* ciaaDevices_deviceType, will be filled later. */
            }/* End-of-table entry */
      };



/*==================[external data definition]===============================*/



/*==================[internal functions definition]==========================*/


inline uint32_t ciaaDriverUartLpc54102_EnabledInts(LPC_FIFO_T *pFIFO, int usartIndex)
{
   /*
    * This function is not provided in LPCOpen, so I made it.
    * */

   return pFIFO->usart[usartIndex].CTLSET;
}


inline uint32_t ciaaDriverUartLpc54102_internalFifoNextIndex(uint32_t index)
{
   int32_t incrementedIndex;

   incrementedIndex = index + 1;

   if (incrementedIndex >= CIAA_DRIVER_USART_LPC54102_INTERNAL_BUFFER_SIZE)
   {
      incrementedIndex = incrementedIndex - CIAA_DRIVER_USART_LPC54102_INTERNAL_BUFFER_SIZE;
   }

   return incrementedIndex;
}


inline void ciaaDriverUartLpc54102_internalFifoClear(ciaaDriverUartLpc54102InternalRxBuffer *fifo)
{
   fifo->head = 0;

   fifo->tail = 0;
}


inline int32_t ciaaDriverUartLpc54102_internalFifoIsEmpty(ciaaDriverUartLpc54102InternalRxBuffer *fifo)
{
   return (fifo->head == fifo->tail) ? 1 : 0;
}


inline uint32_t ciaaDriverUartLpc54102_internalFifoIsFull(ciaaDriverUartLpc54102InternalRxBuffer *fifo)
{
   uint32_t nextTail;

   nextTail = ciaaDriverUartLpc54102_internalFifoNextIndex(fifo->tail);

   return (fifo->head == nextTail) ? 1 : 0;
}


void ciaaDriverUartLpc54102_internalFifoPush(ciaaDriverUartLpc54102InternalRxBuffer *fifo, uint32_t item)
{
   /*
    * You must check whether the FIFO is full before calling this function.
    * */

   fifo->circularQueue[fifo->tail] = item;

   fifo->tail = ciaaDriverUartLpc54102_internalFifoNextIndex(fifo->tail);
}


int32_t ciaaDriverUartLpc54102_internalFifoPop(ciaaDriverUartLpc54102InternalRxBuffer *fifo)
{
   uint32_t oldHead;

   /*
    * You must check whether the FIFO is empty before calling this function.
    * */

   oldHead = fifo->head;

   fifo->head = ciaaDriverUartLpc54102_internalFifoNextIndex(fifo->head);

   return fifo->circularQueue[oldHead];
}


void ciaaDriverUartLpc54102_InitializeControlStructures()
{
   int32_t usartIndex;
   int32_t devIndex;

   for (usartIndex = 0; usartIndex < CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT; usartIndex++)
   {
      lpc54102Uart2devIndexMap[usartIndex] = -1;
   }

   for (devIndex = 0; (devIndex < CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT) && (lpc54102DeviceControlStructures[devIndex].usartIndex >= 0); devIndex++)
   {
      /*
       * Build the backwards map from usartIndex to devIndex
       * */

      lpc54102Uart2devIndexMap[lpc54102DeviceControlStructures[devIndex].usartIndex] = devIndex;

      /*
       * Internal RX buffer
       * */

      lpc54102DeviceControlStructures[devIndex].internalTxBufferPtr = &lpc54102InternalTxBuffers[devIndex];
      lpc54102DeviceControlStructures[devIndex].internalRxBufferPtr = &lpc54102InternalRxBuffers[devIndex];

      ciaaDriverUartLpc54102_internalFifoClear(lpc54102DeviceControlStructures[devIndex].internalTxBufferPtr);
      ciaaDriverUartLpc54102_internalFifoClear(lpc54102DeviceControlStructures[devIndex].internalRxBufferPtr);

      /*
       * POSIX device information information
       * */

      lpc54102PosixRegistrationDataTable[devIndex].path    = lpc54102DeviceControlStructures[devIndex].posixName;

      lpc54102PosixRegistrationDataTable[devIndex].open    = ciaaDriverUart_open;
      lpc54102PosixRegistrationDataTable[devIndex].close   = ciaaDriverUart_close;
      lpc54102PosixRegistrationDataTable[devIndex].read    = ciaaDriverUart_read;
      lpc54102PosixRegistrationDataTable[devIndex].write   = ciaaDriverUart_write;
      lpc54102PosixRegistrationDataTable[devIndex].ioctl   = ciaaDriverUart_ioctl;
      lpc54102PosixRegistrationDataTable[devIndex].lseek   = NULL;

      lpc54102PosixRegistrationDataTable[devIndex].upLayer = NULL;
      lpc54102PosixRegistrationDataTable[devIndex].layer   = (void *)&lpc54102DeviceControlStructures[devIndex];
      lpc54102PosixRegistrationDataTable[devIndex].loLayer = (void *)lpc54102DeviceControlStructures[devIndex].lpcDevice;

      lpc54102DeviceControlStructures[devIndex].posixDeviceDataPtr = &lpc54102PosixRegistrationDataTable[devIndex];
   }
}


void ciaaDriverUartLpc54102_hardwareInit()
{
   LPC_FIFO_CFGSIZE_T fifoSizes;
   LPC_FIFO_CFG_T fifoConfig;

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

   for (devIndex = 0; devIndex < CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT; devIndex++)
   {
      fifoSizes.fifoTXSize[devIndex] = 0;
      fifoSizes.fifoRXSize[devIndex] = 0;
   }

   for (devIndex = 0; (devIndex < CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT) && (lpc54102DeviceControlStructures[devIndex].usartIndex >= 0); devIndex++)
   {
      fifoSizes.fifoTXSize[lpc54102DeviceControlStructures[devIndex].usartIndex] = lpc54102DeviceControlStructures[devIndex].txFifoSize;
      fifoSizes.fifoRXSize[lpc54102DeviceControlStructures[devIndex].usartIndex] = lpc54102DeviceControlStructures[devIndex].rxFifoSize;
   }

   /*
    * Configure and update the new USART FIFO sizes.
    * */

   Chip_FIFO_ConfigFifoSize(LPC_FIFO, FIFO_USART, fifoSizes);

   /*
    * Configure the initial FIFO USART interrupt setup.
    * */

   for (devIndex = 0; lpc54102DeviceControlStructures[devIndex].usartIndex >= 0; devIndex++)
   {

      fifoConfig.rxThreshold = lpc54102DeviceControlStructures[devIndex].rxFifoSize / 2;
      fifoConfig.txThreshold = lpc54102DeviceControlStructures[devIndex].txFifoSize / 2;

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
            lpc54102DeviceControlStructures[devIndex].usartIndex,
            &fifoConfig);

      Chip_FIFOUSART_EnableInts(
            LPC_FIFO,
            lpc54102DeviceControlStructures[devIndex].usartIndex,
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

   for (devIndex = 0; lpc54102DeviceControlStructures[devIndex].usartIndex >= 0; devIndex++)
   {
      Chip_UART_Init(lpc54102DeviceControlStructures[devIndex].lpcDevice);

      Chip_UART_ConfigData(
            lpc54102DeviceControlStructures[devIndex].lpcDevice,
            lpc54102DeviceControlStructures[devIndex].defaultConfig);

      /*
       * WARNING:
       *
       * All USARTs share the same fractional divider, so all the USARTs
       * will always run at the same baudrate, and the following line will
       * leave all of them configured with the baudrate of the last one.
       * */
      Chip_UART_SetBaud(
            lpc54102DeviceControlStructures[devIndex].lpcDevice,
            lpc54102DeviceControlStructures[devIndex].defaultBaudrate);

      NVIC_EnableIRQ(
            lpc54102DeviceControlStructures[devIndex].lpcNvicInterrupt);

      Chip_IOCON_PinMux(
            LPC_IOCON,
            lpc54102DeviceControlStructures[devIndex].lpcIoconTxPort,
            lpc54102DeviceControlStructures[devIndex].lpcIoconTxPin,
            lpc54102DeviceControlStructures[devIndex].lpcIoconTxMode,
            lpc54102DeviceControlStructures[devIndex].lpcIoconTxFunc);

      Chip_IOCON_PinMux(
            LPC_IOCON,
            lpc54102DeviceControlStructures[devIndex].lpcIoconRxPort,
            lpc54102DeviceControlStructures[devIndex].lpcIoconRxPin,
            lpc54102DeviceControlStructures[devIndex].lpcIoconRxMode,
            lpc54102DeviceControlStructures[devIndex].lpcIoconRxFunc);
   }
}


void ciaaDriverUartLpc54102_registerDevices()
{
   int32_t devIndex;

   for (devIndex = 0; (devIndex < CIAA_DRIVER_USART_LPC54102_HW_USARTS_COUNT) && (lpc54102DeviceControlStructures[devIndex].usartIndex >= 0); devIndex++)
   {
      ciaaSerialDevices_addDriver(&lpc54102PosixRegistrationDataTable[devIndex]);
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
   ciaaDriverUartLpc54102DeviceDescriptiontype *dev;
   ciaaDriverUartLpc54102InternalBuffer *rxBufferPtr;
   ciaaDriverUartLpc54102InternalBuffer *txBufferPtr;
   uint32_t uartStatus;
   uint8_t dataByte;

   dev = &lpc54102DeviceControlStructures[lpc54102Uart2devIndexMap[usartIndex]];

   txBufferPtr = dev->internalTxBufferPtr;
   rxBufferPtr = dev->internalRxBufferPtr;

   /* *** */

   uartStatus = Chip_FIFOUSART_GetStatus(LPC_FIFO, dev->usartIndex);

   if(uartStatus & LPC_PERIPFIFO_STAT_RXEMPTY == 0)
   {
      do {

         Chip_FIFOUSART_ReadRX(LPC_FIFO, dev->usartIndex, true, (void*)&dataByte, 1);

         ciaaDriverUartLpc54102_internalFifoPush(rxBufferPtr, dataByte);

         uartStatus = Chip_FIFOUSART_GetStatus(LPC_FIFO, dev->usartIndex)

      } while ((ciaaDriverUartLpc54102_internalFifoIsFull(rxBufferPtr, dataByte) == 0) &&
            (uartStatus & LPC_PERIPFIFO_STAT_RXEMPTY == 0));

      ciaaDriverUartLpc54102_rxIndication(dev->posixDeviceData);
   }

   /* *** */

   if ((ciaaDriverUartLpc54102_EnabledInts(LPC_FIFO, dev->usartIndex) & CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK) != 0)
   {

      usartTxBufferIsFull = 0;

      ciaaDriverUartLpc54102_txConfirmation(dev->posixDeviceData);

      while (ciaaDriverUartLpc54102_internalFifoIsEmpty(txBufferPtr) == 0)
      {

         if (Chip_FIFOUSART_GetTxCount(LPC_FIFO, dev->usartIndex) > 0)
         {
            /*
             * Extract a byte from the internal FIFO and push it into the UART TX
             * buffer.
             * */

            dataByte = ciaaDriverUartLpc54102_internalFifoPop(txBufferPtr);

            Chip_FIFOUSART_WriteTX(LPC_FIFO, dev->usartIndex, true, (void*)&dataByte, 1);

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

         if (ciaaDriverUartLpc54102_internalFifoIsEmpty(txBufferPtr) != 0)
         {
            ciaaDriverUartLpc54102_txConfirmation(dev->posixDeviceData);
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
         Chip_FIFOUSART_DisableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK)
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
   ciaaDriverUartLpc54102DeviceDescriptiontype *dev;
   ciaaDriverUartLpc54102InternalBuffer *rxBufferPtr;
   ssize_t ret = -1;

   dev = (ciaaDriverUartLpc54102DeviceDescriptiontype *)device->layer;

   switch(request)
   {
      case ciaaPOSIX_IOCTL_STARTTX:

         /* Enable FIFO TX Threshold interrupts */
         Chip_FIFOUSART_EnableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK)
         ret = 0;
         break;

      case ciaaPOSIX_IOCTL_SET_BAUDRATE:

         /*
          * WARNING: this will set the baudrate of ALL the USARTs at the same time,
          * since they all share the same fractional divider.
          * */
         Chip_UART_SetBaud(dev->lpcDevice,  (int32_t)param);
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
            Chip_FIFOUSART_DisableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK)

         } else {

            /*
             * Enable TX interrupts.
             * */
            Chip_FIFOUSART_EnableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_TX_INTERRUPT_MASK)
         }
         break;

      case ciaaPOSIX_IOCTL_SET_ENABLE_RX_INTERRUPT:

         if((bool)(intptr_t)param == false)
         {
            /*
             * Disable RX interrupts.
             * */
            Chip_FIFOUSART_DisableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_RX_INTERRUPT_MASK)

         } else {

            /*
             * Enable RX interrupts.
             * */
            Chip_FIFOUSART_EnableInts(LPC_FIFO, dev->usartIndex, CIAA_DRIVER_USART_LPC54102_RX_INTERRUPT_MASK)
         }
         break;
   }

   return ret;
}


extern ssize_t ciaaDriverUart_read(ciaaDevices_deviceType const * const device, uint8_t* buffer, size_t const size)
{
   ciaaDriverUartLpc54102DeviceDescriptiontype *dev;
   ciaaDriverUartLpc54102InternalBuffer *rxBufferPtr;
   ssize_t ret = -1;

   dev = (ciaaDriverUartLpc54102DeviceDescriptiontype *)device->layer;

   rxBufferPtr = dev->internalTxBufferPtr;

   for (ret = 0; ret < size; ret++)
   {
      if (ciaaDriverUartLpc54102_internalFifoIsEmpty(rxBufferPtr) == 0)
      {
         buffer[ret] = ciaaDriverUartLpc54102_internalFifoPop(rxBufferPtr);

      } else {

         break;
      }
   }

   return ret;
}


extern ssize_t ciaaDriverUart_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size)
{
   ciaaDriverUartLpc54102DeviceDescriptiontype *dev;
   ciaaDriverUartLpc54102InternalBuffer *txBufferPtr;
   ssize_t ret = -1;

   dev = (ciaaDriverUartLpc54102DeviceDescriptiontype *)device->layer;

   txBufferPtr = dev->internalTxBufferPtr;

   for (ret = 0; ret < size; ret++)
   {
      if (ciaaDriverUartLpc54102_internalFifoIsFull(txBufferPtr) == 0)
      {
         ciaaDriverUartLpc54102_internalFifoPush(txBufferPtr, buffer[ret]);

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

   ciaaDriverUartLpc54102_registerDevices()
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

