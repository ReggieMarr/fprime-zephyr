// ======================================================================
// \title  ZephyrAsyncUartDriver.hpp
// \author reggiemarr
// \brief  hpp file for ZephyrAsyncUartDriver component implementation class
// ======================================================================

#ifndef ZephyrAsyncUartDriver_HPP
#define ZephyrAsyncUartDriver_HPP

#include "Fw/Buffer/Buffer.hpp"
#include "Utils/Types/Queue.hpp"
#include "config/FwIndexTypeAliasAc.h"
#include "config/FwSizeTypeAliasAc.h"
#include "fprime-zephyr/Drv/ZephyrAsyncUartDriver/BufferDescriptorSerializableAc.hpp"
#include "fprime-zephyr/Drv/ZephyrAsyncUartDriver/ZephyrAsyncUartDriverComponentAc.hpp"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

namespace Zephyr {

  class ZephyrAsyncUartDriver :
    public ZephyrAsyncUartDriverComponentBase
  {

    const FwSizeType SERIAL_BUFFER_SIZE = 64;

    public:

        // ----------------------------------------------------------------------
        // Construction, initialization, and destruction
        // ----------------------------------------------------------------------

        //! Construct object ZephyrAsyncUartDriver
        //!
        ZephyrAsyncUartDriver(
            const char *const compName /*!< The component name*/
        );

        //! Destroy object ZephyrAsyncUartDriver
        //!
        ~ZephyrAsyncUartDriver();

        void configure(const struct device *dev, U32 baud_rate);

    public:

        static constexpr FwSizeType MAX_RX_BUFF_SIZE = 1024;

        // ----------------------------------------------------------------------
        // Handler implementations for user-defined typed input ports
        // ----------------------------------------------------------------------

        //! Handler implementation for send
        //!
        void asyncSend_handler(
            const FwIndexType portNum, /*!< The port number*/
            Fw::Buffer &sendBuffer 
        );

        // Handler implementation for send
        //!
        Drv::ByteStreamStatus send_handler(
            const FwIndexType portNum, /*!< The port number*/
            Fw::Buffer &sendBuffer
        );

        void recvReturnIn_handler(
            const FwIndexType portNum, /*!< The port number*/
            Fw::Buffer &returnBuffer
        );

    private:
        void static uartEventCallback(const struct device *dev, struct uart_event *evt, void *user_data);
        const struct device *m_dev;
        // NOTE this is a wag, come up with better estimates
        // static constexpr FwSizeType FIFO_QUEUE_SIZE = 5;
        // Types::Queue m_queue;  //!< Stores queued data waiting for transmission
        // U8 m_allocation[sizeof(BufferDescriptor::SERIALIZED_SIZE) * FIFO_QUEUE_SIZE];
        Fw::Buffer m_txPending;
    };

} // end namespace Zephyr

#endif
