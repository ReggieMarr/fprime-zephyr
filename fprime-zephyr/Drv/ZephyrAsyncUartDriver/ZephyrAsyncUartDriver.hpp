// ======================================================================
// \title  ZephyrAsyncUartDriver.hpp
// \author reggiemarr
// \brief  hpp file for ZephyrAsyncUartDriver component implementation class
// ======================================================================

#ifndef ZephyrAsyncUartDriver_HPP
#define ZephyrAsyncUartDriver_HPP

#include "Fw/Buffer/Buffer.hpp"
#include "Os/Mutex.hpp"
#include "Svc/Ccsds/Types/SpacePacketHeaderSerializableAc.hpp"
#include "Utils/Types/Queue.hpp"
#include "config/FwIndexTypeAliasAc.h"
#include "config/FwSizeTypeAliasAc.h"
#include "fprime-zephyr/Drv/ZephyrAsyncUartDriver/BufferDescriptorSerializableAc.hpp"
#include "fprime-zephyr/Drv/ZephyrAsyncUartDriver/ZephyrAsyncUartDriverComponentAc.hpp"

#include <array>
#include <atomic>
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

        static constexpr FwSizeType RX_ACCUMULATE_SIZE = 256;
        static constexpr FwSizeType RX_WORK_QUEUE_SIZE = 5;

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
        U32 m_rxBuffContext;
        typedef struct UartWorkContext_s {
            struct k_work work;
            Fw::Buffer pendingBuff;
            ZephyrAsyncUartDriver *driver;
        } UartWorkContext_t;
        Fw::Buffer m_pendingTxBuff;

        std::array<UartWorkContext_t, RX_WORK_QUEUE_SIZE>  m_rxWorkContexts;
    };

} // end namespace Zephyr

#endif
