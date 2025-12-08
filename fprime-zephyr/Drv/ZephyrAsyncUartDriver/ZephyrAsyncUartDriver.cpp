// ======================================================================
// \title  ZephyrAsyncUartDriver.cpp
// \author reggiemarr
// \brief  cpp file for ZephyrAsyncUartDriver component implementation class
// ======================================================================


#include "fprime-zephyr/Drv/ZephyrAsyncUartDriver/ZephyrAsyncUartDriver.hpp"
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

#if !defined(CONFIG_UART_ASYNC_API)
#error "Async Uart driver requires CONFIG_UART_ASYNC_API=y"
#endif

namespace Zephyr {

    void ZephyrAsyncUartDriver::uartEventCallback(const struct device *dev, struct uart_event *evt, void *user_data) {
        ZephyrAsyncUartDriver *driver = reinterpret_cast<ZephyrAsyncUartDriver *>(user_data);
        I32 rc;
        Fw::String evtName("Unknown");
        Fw::Buffer rxBuff;

        switch (evt->type) {
        case UART_RX_BUF_REQUEST:
            evtName = "UART_RX_BUF_REQUEST";
            rxBuff = driver->allocate_out(0, MAX_RX_BUFF_SIZE);
            rc = uart_rx_buf_rsp(dev, rxBuff.getData(), rxBuff.getSize());
            FW_ASSERT(rxBuff.getSize() >= MAX_RX_BUFF_SIZE, rxBuff.getSize());
            // Indicates Next Buffer is already set so we can de allocate this one
            if (rc == -EBUSY) {
                driver->deallocate_out(0, rxBuff);
                break;
            }
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
            if (evtName != "Unknown") {
                evtName = "UART_RX_BUF_RELEASED";
            }
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
        // driver->log_ACTIVITY_LO_ZEPHYR_UART_STATE_CHANGE(evt->type, evtName);
    }

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    ZephyrAsyncUartDriver ::
        ZephyrAsyncUartDriver(
            const char *const compName
        ) : ZephyrAsyncUartDriverComponentBase(compName)
    {
    }

    ZephyrAsyncUartDriver ::
        ~ZephyrAsyncUartDriver()
    {

    }

    void ZephyrAsyncUartDriver::configure(const struct device *dev, U32 baud_rate) {
        FW_ASSERT(dev != nullptr);
        this->m_dev = dev;

        FW_ASSERT(device_is_ready(this->m_dev));

        struct uart_config uart_cfg = {
            .baudrate = baud_rate,
            .parity = UART_CFG_PARITY_NONE,
            .stop_bits = UART_CFG_STOP_BITS_1,
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        };
        uart_configure(this->m_dev, &uart_cfg);

        I32 ret = uart_callback_set(this->m_dev, this->uartEventCallback, (void *)this);
        FW_ASSERT(ret == 0, ret);

        Fw::Buffer rxBuff = this->allocate_out(0, MAX_RX_BUFF_SIZE);
        FW_ASSERT(rxBuff.getSize() >= MAX_RX_BUFF_SIZE, rxBuff.getSize());

        ret = uart_rx_enable(this->m_dev, rxBuff.getData(), rxBuff.getSize(), SYS_FOREVER_US);
        FW_ASSERT(ret == 0, ret);

        if (this->isConnected_ready_OutputPort(0)) {
            this->ready_out(0);
        }
    }

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------
    void ZephyrAsyncUartDriver ::
        asyncSend_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        Drv::ByteStreamStatus sendResponse = Drv::ByteStreamStatus::OP_OK;
        FW_ASSERT(this->isConnected_drvAsyncSendReturnOut_OutputPort(0));
        int rc;
        rc = uart_tx(this->m_dev, sendBuffer.getData(), sendBuffer.getSize(), SYS_FOREVER_US);
        switch (rc) {
            case (-EBUSY):
                this->drvAsyncSendReturnOut_out(0, sendBuffer, Drv::ByteStreamStatus::SEND_RETRY);
                break;
            default:
                this->drvAsyncSendReturnOut_out(0, sendBuffer, Drv::ByteStreamStatus::OTHER_ERROR);
                break;
            case 0:
                this->txPendingBuffer = sendBuffer;
                break;
        }
    }

    Drv::ByteStreamStatus ZephyrAsyncUartDriver ::
        send_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        // This driver doesn't support this send command
        return Drv::ByteStreamStatus::OTHER_ERROR;
    }

    void ZephyrAsyncUartDriver ::recvReturnIn_handler(const FwIndexType portNum, Fw::Buffer &returnBuffer) {
        this->deallocate_out(0, returnBuffer);
    }

} // end namespace Zephyr
