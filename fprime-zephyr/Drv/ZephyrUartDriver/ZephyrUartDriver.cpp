// ======================================================================
// \title  ZephyrUartDriver.cpp
// \author ethanchee
// \brief  cpp file for ZephyrUartDriver component implementation class
// ======================================================================


#include "fprime-zephyr/Drv/ZephyrUartDriver/ZephyrUartDriver.hpp"
#include "Drv/ByteStreamDriverModel/ByteStreamStatusEnumAc.hpp"
#include "Fw/Buffer/Buffer.hpp"
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/Assert.hpp"
#include <Fw/FPrimeBasicTypes.hpp>
#include <Fw/Logger/Logger.hpp>
#include <cerrno>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>


namespace Zephyr {

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    ZephyrUartDriver ::
        ZephyrUartDriver(
            const char *const compName
        ) : ZephyrUartDriverComponentBase(compName)
    {
    }

    ZephyrUartDriver ::
        ~ZephyrUartDriver()
    {

    }

    void ZephyrUartDriver::configure(const struct device *dev, U32 baud_rate) {
        FW_ASSERT(dev != nullptr);
        m_dev = dev;

        if (!device_is_ready(this->m_dev)) {
            return;
        }

        struct uart_config uart_cfg = {
            .baudrate = baud_rate,
            .parity = UART_CFG_PARITY_NONE,
            .stop_bits = UART_CFG_STOP_BITS_1,
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        };
        uart_configure(this->m_dev, &uart_cfg);

        this->setup_async_rx();

        if (this->isConnected_ready_OutputPort(0)) {
            this->ready_out(0);
        }
    }

#if defined(CONFIG_UART_ASYNC_API)
    void ZephyrUartDriver::setup_async_rx()
    {
        I32 ret = uart_callback_set(this->m_dev, this->serial_cb, (void *)this);  // Pass 'this' instead of uart_dev
        FW_ASSERT(ret == 0, ret);
        Fw::Buffer rxBuff = this->allocate_out(0, MAX_RX_BUFF_SIZE);
        FW_ASSERT(rxBuff.getSize() >= MAX_RX_BUFF_SIZE, rxBuff.getSize());
        ret = uart_rx_enable(this->m_dev, rxBuff.getData(), rxBuff.getSize(), SYS_FOREVER_US);
        FW_ASSERT(ret == 0, ret);
    }

    void ZephyrUartDriver::serial_cb(const struct device *dev, struct uart_event *evt, void *user_data)
    {
        ZephyrUartDriver *driver = reinterpret_cast<ZephyrUartDriver *>(user_data);
        I32 rc;
        Fw::String evtName("Unknown");
        Fw::Buffer rxBuff;

        // LOG_DBG("EVENT: %d", evt->type);
        switch (evt->type) {
        case UART_RX_BUF_REQUEST:
            evtName = "UART_RX_BUF_REQUEST";
            rxBuff = driver->allocate_out(0, MAX_RX_BUFF_SIZE);
            rc = uart_rx_buf_rsp(dev, rxBuff.getData(), rxBuff.getSize());
            FW_ASSERT(rxBuff.getSize() >= MAX_RX_BUFF_SIZE, rxBuff.getSize());
            FW_ASSERT(rc == 0, rc);
            break;
        case UART_RX_RDY:
            evtName = "UART_RX_RDY";
            // TODO consider using a ring buffer here
            rxBuff.set(evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len, Fw::Buffer::NO_CONTEXT);
            driver->recv_out(0, rxBuff, evt->data.rx.len > 0 ? Drv::ByteStreamStatus::OP_OK : Drv::ByteStreamStatus::RECV_NO_DATA);
            break;
        case UART_RX_STOPPED:
            evtName = "UART_RX_STOPPED";
        case UART_RX_BUF_RELEASED:
            evtName = "UART_RX_BUF_RELEASED";
        case UART_RX_DISABLED:
            if (evtName != "Unknown") {
                evtName = "UART_RX_DISABLED";
            }

            rxBuff.setData(evt->data.rx_buf.buf);
            driver->deallocate_out(0, rxBuff);
            break;
        case UART_TX_DONE:
            evtName = "UART_TX_DONE";
            driver->drvAsyncSendReturnOut_out(0, driver->txPendingBuffer, Drv::ByteStreamStatus::OP_OK);
            break;
        default:
            break;
        }
        driver->log_ACTIVITY_LO_ZEPHYR_UART_STATE_CHANGE(evt->type, evtName);
    }
#elif defined(CONFIG_UART_INTERRUPT_DRIVEN)
    void ZephyrUartDriver::setup_async_rx()
    {
        uart_irq_callback_user_data_set(this->m_dev, this->serial_cb, &this->m_ring_buf);
        uart_irq_rx_enable(this->m_dev);
	    uart_irq_tx_disable(this->m_dev);
    }

    void ZephyrUartDriver::serial_cb(const struct device *dev, void *user_data)
    {
        struct ring_buf *ring_buf = reinterpret_cast<struct ring_buf *>(user_data);

        if (!uart_irq_update(dev)) {
            return;
        }

        if (!uart_irq_rx_ready(dev)) {
            return;
        }

        U8 c;
        // TODO: Get rid of the endless loop (in an IRQ handler!).
        while (uart_fifo_read(dev, &c, 1) == 1) {
            if (ring_buf_put(ring_buf, &c, 1) != 1) {
                // TODO: Handle properly.
                printk("UART buffer overrun\n");
            }
        }
    }
#else
#error "Cannot build ZephyrUartDriver without an rx mechanism"
#endif


    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void ZephyrUartDriver ::
        schedIn_handler(
            const FwIndexType portNum,
            U32 context
        )
    {
#if defined(CONFIG_UART_ASYNC_API)
        // If using UART Async api no schedIn_handler is needed
        FW_ASSERT(0);
#elif defined(CONFIG_UART_INTERRUPT_DRIVEN)
        Fw::Buffer recv_buffer = this->allocate_out(0, SERIAL_BUFFER_SIZE);

        U32 recv_size = ring_buf_get(&this->m_ring_buf, recv_buffer.getData(), recv_buffer.getSize());
        if (recv_size > 0) {
            recv_buffer.setSize(recv_size);
            recv_out(0, recv_buffer, Drv::ByteStreamStatus::OP_OK);
        } else {
            // No data available, return the buffer
            this->deallocate_out(0, recv_buffer);
        }
#else
#error "Cannot build ZephyrUartDriver without an rx mechanism"
#endif
    }

    Drv::ByteStreamStatus ZephyrUartDriver ::
        send_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        Drv::ByteStreamStatus sendResponse = Drv::ByteStreamStatus::OP_OK;
#if defined(CONFIG_UART_ASYNC_API)
        FW_ASSERT(this->isConnected_drvAsyncSendReturnOut_OutputPort(0));
        int rc;
        rc = uart_tx(this->m_dev, sendBuffer.getData(), sendBuffer.getSize(), SYS_FOREVER_US);
        if (rc == -EBUSY) {
           sendResponse = Drv::ByteStreamStatus::SEND_RETRY;
           this->drvAsyncSendReturnOut_out(0, sendBuffer, sendResponse);
        }
        else if (rc != 0) {
           sendResponse = Drv::ByteStreamStatus::OTHER_ERROR;
           this->drvAsyncSendReturnOut_out(0, sendBuffer, sendResponse);
        }
#elif defined(CONFIG_UART_INTERRUPT_DRIVEN)
        for (U32 i = 0; i < sendBuffer.getSize(); i++) {
            uart_poll_out(this->m_dev, sendBuffer.getData()[i]);
        }
#else
#error "Cannot build ZephyrUartDriver without an rx mechanism"
#endif
        return sendResponse;
    }

    void ZephyrUartDriver ::recvReturnIn_handler(const FwIndexType portNum, Fw::Buffer &returnBuffer) {
        this->deallocate_out(0, returnBuffer);
    }

} // end namespace Zephyr
