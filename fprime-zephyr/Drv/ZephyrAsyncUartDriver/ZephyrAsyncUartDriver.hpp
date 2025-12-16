// ======================================================================
// \title  ZephyrAsyncUartDriver.hpp
// \author reggiemarr
// \brief  hpp file for ZephyrAsyncUartDriver component implementation class
// ======================================================================

#ifndef ZephyrAsyncUartDriver_HPP
#define ZephyrAsyncUartDriver_HPP

#include "Fw/Buffer/Buffer.hpp"
#include "Os/Mutex.hpp"
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
        typedef struct UartWorkContext_s {
            struct k_work work;
            ZephyrAsyncUartDriver *driver;
        } UartWorkContext_t;

        UartWorkContext_t m_txWorkContext;
        static void txDoneWorkHandler(struct k_work *work);
        Fw::Buffer m_txPending;
        Os::Mutex m_txLock;

        UartWorkContext_t m_rxWorkContext;
        static void rxDoneWorkHandler(struct k_work *work);
        Fw::Buffer m_rxPending;
    };

} // end namespace Zephyr

#endif
