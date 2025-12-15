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
#include "Platform/PlatformTypes.h"
#include "config/FwIndexTypeAliasAc.h"
#include "fprime-zephyr/Drv/ZephyrAsyncUartDriver/BufferDescriptorSerializableAc.hpp"
#include <Fw/FPrimeBasicTypes.hpp>
#include <Fw/Logger/Logger.hpp>
#include <cerrno>
#include <cstdint>
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
        Fw::Buffer uartBuff;
        BufferDescriptor bufferDesc;

        switch (evt->type) {
        case UART_RX_BUF_REQUEST:
            evtName = "UART_RX_BUF_REQUEST";
            uartBuff = driver->allocate_out(0, MAX_RX_BUFF_SIZE);
            rc = uart_rx_buf_rsp(dev, reinterpret_cast<uint8_t*>(uartBuff.getData()), uartBuff.getSize());
            FW_ASSERT(uartBuff.getSize() >= MAX_RX_BUFF_SIZE, uartBuff.getSize());
            // Indicates Next Buffer is already set so we can de allocate this one
            if (rc == -EBUSY) {
                driver->deallocate_out(0, uartBuff);
                break;
            }
            FW_ASSERT(rc == 0, rc);
            break;
        case UART_RX_RDY:
            evtName = "UART_RX_RDY";
            // TODO consider using a ring buffer here
            uartBuff.set(reinterpret_cast<U8*>(evt->data.rx.buf + evt->data.rx.offset), evt->data.rx.len, Fw::Buffer::NO_CONTEXT);
            driver->recv_out(0, uartBuff, evt->data.rx.len > 0 ? Drv::ByteStreamStatus::OP_OK : Drv::ByteStreamStatus::RECV_NO_DATA);
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

            uartBuff.setData(reinterpret_cast<U8*>(evt->data.rx_buf.buf));
            driver->deallocate_out(0, uartBuff);
            break;
        case UART_TX_DONE:
            evtName = "UART_TX_DONE";
            // driver->m_queue.dequeue(reinterpret_cast<U8*>(&bufferDesc), bufferDesc.serializedSize());
            // uartBuff.set(reinterpret_cast<U8*>(bufferDesc.get_Address()), bufferDesc.get_Length(), bufferDesc.get_Context());
            // FW_ASSERT(evt->data.tx.buf == reinterpret_cast<uint8_t *>(uartBuff.getData()));
            // FW_ASSERT(evt->data.tx.len == uartBuff.getSize());
            // driver->drvAsyncSendReturnOut_out(0, uartBuff, Drv::ByteStreamStatus::OP_OK);
            driver->drvAsyncSendReturnOut_out(0, driver->m_txPending, Drv::ByteStreamStatus::OP_OK);
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

        I32 ret = uart_callback_set(this->m_dev, this->uartEventCallback, (void *)this);
        FW_ASSERT(ret == 0, ret);

        Fw::Buffer rxBuff = this->allocate_out(0, MAX_RX_BUFF_SIZE);
        FW_ASSERT(rxBuff.isValid() && rxBuff.getSize() >= MAX_RX_BUFF_SIZE, rxBuff.getSize());

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
        this->m_txPending = sendBuffer;
        rc = uart_tx(this->m_dev, reinterpret_cast<uint8_t*>(sendBuffer.getData()), sendBuffer.getSize(), SYS_FOREVER_US);
        switch (rc) {
            case (-EBUSY):
                this->drvAsyncSendReturnOut_out(0, sendBuffer, Drv::ByteStreamStatus::SEND_RETRY);
                break;
            default:
                this->drvAsyncSendReturnOut_out(0, sendBuffer, Drv::ByteStreamStatus::OTHER_ERROR);
                break;
            case 0:
                // Track the buffer here via sendBuffer.getContext()
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
