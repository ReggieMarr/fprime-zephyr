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
#include "Os/Mutex.hpp"
#include "Platform/PlatformTypes.h"
#include "config/FwIndexTypeAliasAc.h"
#include "fprime-zephyr/Drv/ZephyrAsyncUartDriver/BufferDescriptorSerializableAc.hpp"
#include "fprime-zephyr/Drv/ZephyrAsyncUartDriver/ZephyrUartStopReasonEnumAc.hpp"
#include "fprime-zephyr/Os/Mutex.hpp"
#include "zephyr/irq.h"
#include "zephyr/sys/util.h"
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
        Fw::Buffer uartBuff;
        ZephyrUartStopReason stopReason(static_cast<ZephyrUartStopReason::T>(evt->data.rx_stop.reason));

        switch (evt->type) {
        case UART_RX_BUF_REQUEST:
            uartBuff = driver->allocate_out(0, RX_ACCUMULATE_SIZE);
            rc = uart_rx_buf_rsp(dev, reinterpret_cast<uint8_t*>(uartBuff.getData()), uartBuff.getSize());
            FW_ASSERT(uartBuff.getSize() >= RX_ACCUMULATE_SIZE, uartBuff.getSize());
            // Indicates Next Buffer is already set so we can de allocate this one
            if (rc == -EBUSY) {
                driver->deallocate_out(0, uartBuff);
                break;
            }
            FW_ASSERT(rc == 0, rc);
            break;
        case UART_RX_RDY:
            // TODO consider using a ring buffer here
            uartBuff.set(reinterpret_cast<U8*>(evt->data.rx.buf + evt->data.rx.offset), evt->data.rx.len, Fw::Buffer::NO_CONTEXT);
            // NOTE this is expected to block until the buffer is passed down stream and the ownership is passed back to us
            // Currently this will work since it passes through a series of sync ports but we should leverage a semaphore here
            driver->recv_out(0, uartBuff, evt->data.rx.len > 0 ? Drv::ByteStreamStatus::OP_OK : Drv::ByteStreamStatus::RECV_NO_DATA);
            break;
        case UART_RX_STOPPED:
            driver->log_WARNING_HI_ZEPHYR_RX_STOPPED(stopReason);
        case UART_RX_BUF_RELEASED:
        case UART_RX_DISABLED:
            uartBuff.set(reinterpret_cast<U8*>(evt->data.rx_buf.buf), evt->data.rx.len, Fw::Buffer::NO_CONTEXT);
            driver->deallocate_out(0, uartBuff);
            break;
        case UART_TX_DONE:
            (void)k_work_submit(&driver->m_txWorkContext.work);
            break;
        default:
            break;
        }
    }

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

    void ZephyrAsyncUartDriver::rxDoneWorkHandler(struct k_work *workReference) {
        UartWorkContext_t *context = CONTAINER_OF(workReference, UartWorkContext_t, work);

        // Fw::Buffer pendingRxBuff = context->driver->m_rxPending;
        // context->driver->m_rxPending = Fw::Buffer();  // Clear/invalidate txPending

        // // Safe to call F' ports in thread context
        // if (pendingRxBuff.isValid() && pendingRxBuff.getSize() > 0) {
        //     context->driver->recv_out(0, pendingRxBuff, Drv::ByteStreamStatus::OP_OK);
        // }
        // else {
        //     context->driver->deallocate_out(0, pendingRxBuff);
        // }
    }

    void ZephyrAsyncUartDriver::txDoneWorkHandler(struct k_work *workReference) {
        UartWorkContext_t *context = CONTAINER_OF(workReference, UartWorkContext_t, work);

        FW_ASSERT(context->pendingBuff.isValid());

        // Safe to call F' ports in thread context
        context->driver->drvAsyncSendReturnOut_out(0, context->pendingBuff, Drv::ByteStreamStatus::OP_OK);
    }

    void ZephyrAsyncUartDriver::configure(const struct device *dev, U32 baud_rate) {
        FW_ASSERT(dev != nullptr);
        this->m_dev = dev;

        FW_ASSERT(device_is_ready(this->m_dev));

        I32 ret = uart_callback_set(this->m_dev, this->uartEventCallback, (void *)this);
        FW_ASSERT(ret == 0, ret);

        FW_ASSERT(ret == 0, ret);
        // Initialize worker queue/handler
        this->m_txWorkContext.driver = this;
        k_work_init(&this->m_txWorkContext.work, this->txDoneWorkHandler);

        for (UartWorkContext_t rxWork: this->m_rxWorkContexts) {
            rxWork.driver = this;
            k_work_init(&rxWork.work, this->rxDoneWorkHandler);
        }
        this->m_rxWorkContexts.at(0).pendingBuff = this->allocate_out(0, RX_ACCUMULATE_SIZE);
        FW_ASSERT(this->m_rxWorkContexts.at(0).pendingBuff.isValid() &&
                  this->m_rxWorkContexts.at(0).pendingBuff.getSize() >= RX_ACCUMULATE_SIZE,
                  this->m_rxWorkContexts.at(0).pendingBuff.getSize());
        ret = uart_rx_enable(this->m_dev,
                            reinterpret_cast<uint8_t*>(this->m_rxWorkContexts.at(0).pendingBuff.getData()),
                            this->m_rxWorkContexts.at(0).pendingBuff.getSize(),
                            SYS_FOREVER_US);

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

        uint8_t* orig_buf = reinterpret_cast<uint8_t*>(sendBuffer.getData());
        size_t orig_size = sendBuffer.getSize();

        int rc;
        // TODO check the workqueue status here
        this->m_txWorkContext.pendingBuff = sendBuffer;

        rc = uart_tx(this->m_dev, orig_buf, orig_size, SYS_FOREVER_US);
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
        // NOTE this is a NO-OP since we manage the buffer based off of the uart state machine
        // this->deallocate_out(0, returnBuffer);
    }

} // end namespace Zephyr
