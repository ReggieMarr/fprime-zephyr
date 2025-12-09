// ======================================================================
// \title  ZephyrInterruptUartDriver.hpp
// \author reggiemarr
// \brief  hpp file for ZephyrInterruptUartDriver component implementation class
// ======================================================================

#ifndef ZephyrInterruptUartDriver_HPP
#define ZephyrInterruptUartDriver_HPP

#include "Drv/ByteStreamDriverModel/ByteStreamStatusEnumAc.hpp"
#include "Fw/Buffer/Buffer.hpp"
#include "Os/Task.hpp"
#include "config/FwIndexTypeAliasAc.h"
#include "config/FwSizeTypeAliasAc.h"
#include "fprime-zephyr/Drv/ZephyrInterruptUartDriver/ZephyrInterruptUartDriverComponentAc.hpp"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

namespace Zephyr {

  class ZephyrInterruptUartDriver :
    public ZephyrInterruptUartDriverComponentBase
  {

    const FwSizeType SERIAL_BUFFER_SIZE = 64;

    public:

        // ----------------------------------------------------------------------
        // Construction, initialization, and destruction
        // ----------------------------------------------------------------------

        //! Construct object ZephyrInterruptUartDriver
        //!
        ZephyrInterruptUartDriver(
            const char *const compName /*!< The component name*/
        );

        //! Destroy object ZephyrInterruptUartDriver
        //!
        ~ZephyrInterruptUartDriver();

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

        static constexpr size_t RING_SIZE = 2048;
    private:
        void static uartISR(const struct device *dev, void *user_data);
        const struct device *m_dev;
        Fw::Buffer m_txBuff;
        Fw::Buffer m_rxBuff;

        struct ring_buf m_RxRingbuf;
        struct ring_buf m_TxRingbuf;

        Os::Task m_isrRcvTask;
        k_poll_signal m_rxSignal;

        static void handle_isrRcv(void* ptr);
        // Signal error flags (for bytes_or_error when negative)
        static constexpr int SIGNAL_ERR_RING_OVERFLOW  = (1 << 0);  // -1
        static constexpr int SIGNAL_ERR_UART_OVERRUN   = (1 << 1);  // -2
        static constexpr int SIGNAL_ERR_UART_PARITY    = (1 << 2);  // -4
        static constexpr int SIGNAL_ERR_UART_FRAMING   = (1 << 3);  // -8
        static constexpr int SIGNAL_ERR_UART_BREAK     = (1 << 4);  // -16
    };

} // end namespace Zephyr

#endif
