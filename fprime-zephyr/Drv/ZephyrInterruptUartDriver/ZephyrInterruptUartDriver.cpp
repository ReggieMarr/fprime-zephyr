// ======================================================================
// \title  ZephyrInterruptUartDriver.cpp
// \author reggiemarr
// \brief  cpp file for ZephyrInterruptUartDriver component implementation class
// ======================================================================


#include "fprime-zephyr/Drv/ZephyrInterruptUartDriver/ZephyrInterruptUartDriver.hpp"
#include "Drv/ByteStreamDriverModel/ByteStreamStatusEnumAc.hpp"
#include "Fw/Buffer/Buffer.hpp"
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/Assert.hpp"
#include "Os/Task.hpp"
#include "fprime-zephyr/Drv/ZephyrInterruptUartDriver/ZephyrUartErrorTypeEnumAc.hpp"
#include <Fw/FPrimeBasicTypes.hpp>
#include <Fw/Logger/Logger.hpp>
#include <cerrno>
#include <cstdint>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#if !defined(CONFIG_UART_INTERRUPT_DRIVEN)
#error "Async Uart driver requires CONFIG_UART_INTERRUPT_DRIVEN=y"
#endif

namespace Zephyr {

    void ZephyrInterruptUartDriver::uartISR(const struct device *dev, void *user_data) {
        ZephyrInterruptUartDriver *me = reinterpret_cast<ZephyrInterruptUartDriver *>(user_data);
        // // This is a mandatory call to satisfy the isr api. If it fails we can do nothing
        if (uart_irq_update(me->m_dev) != 1) {
            return;
        }

        // CHECK: Hardware errors first
        // RX: Read directly into ring buffer
        if (uart_irq_rx_ready(me->m_dev)) {
            uint8_t *data;
            uint32_t len = ring_buf_put_claim(&me->m_RxRingbuf, &data, RING_SIZE);
            if (len > 0) {
                int received = uart_fifo_read(me->m_dev, data, len);
                ring_buf_put_finish(&me->m_RxRingbuf, received);
                k_poll_signal_raise(&me->m_rxSignal, received);
            } else {
                // Ring buffer full! Could signal error
                uint8_t discard[64];
                uart_fifo_read(me->m_dev, discard, sizeof(discard));
                k_poll_signal_raise(&me->m_rxSignal, -1); // Error indicator
            }
        }

        // TX: Write directly from ring buffer
        if (uart_irq_tx_ready(me->m_dev)) {
            uint8_t *data;
            uint32_t len = ring_buf_get_claim(&me->m_TxRingbuf, &data, 512);
            if (len > 0) {
                int sent = uart_fifo_fill(me->m_dev, data, len);
                ring_buf_get_finish(&me->m_TxRingbuf, sent);
            } else {
                uart_irq_tx_disable(me->m_dev);  // Nothing to send
            }
        }

        if (uart_irq_tx_complete(me->m_dev)) {
            uart_irq_tx_disable(me->m_dev);  // Nothing to send
            // me->drvAsyncSendReturnOut_out(0, me->m_txBuff, Drv::ByteStreamStatus::OP_OK);
        }

        int uart_errors = uart_err_check(me->m_dev);
        if (uart_errors != 0) {
            // Build error flags
            int error_flags = 0;

            if (uart_errors & UART_ERROR_OVERRUN) {
                error_flags |= SIGNAL_ERR_UART_OVERRUN;
            }
            if (uart_errors & UART_ERROR_PARITY) {
                error_flags |= SIGNAL_ERR_UART_PARITY;
            }
            if (uart_errors & UART_ERROR_FRAMING) {
                error_flags |= SIGNAL_ERR_UART_FRAMING;
            }
            if (uart_errors & UART_BREAK) {
                error_flags |= SIGNAL_ERR_UART_BREAK;
            }

            // Signal the errors (negative value)
            k_poll_signal_raise(&me->m_rxSignal, -error_flags);

            // Still need to drain UART FIFO to clear the error condition
            if (uart_irq_rx_ready(me->m_dev)) {
                uint8_t discard[64];
                uart_fifo_read(me->m_dev, discard, sizeof(discard));
            }

            return; // Don't process data on error
        }

    }
    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    ZephyrInterruptUartDriver ::
        ZephyrInterruptUartDriver(
            const char *const compName
        ) : ZephyrInterruptUartDriverComponentBase(compName)
    {
        k_poll_signal_init(&this->m_rxSignal);
    }

    ZephyrInterruptUartDriver ::
        ~ZephyrInterruptUartDriver()
    {

    }

    void ZephyrInterruptUartDriver ::handle_isrRcv(void* ptr) {
        ZephyrInterruptUartDriver* me = reinterpret_cast<ZephyrInterruptUartDriver*>(ptr);
        struct k_poll_event events[1];
        k_poll_event_init(&events[0], K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
                          &me->m_rxSignal);

        while (true) {
            int res = k_poll(events, 1, K_FOREVER);
            FW_ASSERT(res == 0, -res);

            unsigned int signaled_count;
            int bytes_or_error;
            k_poll_signal_check(&me->m_rxSignal, &signaled_count, &bytes_or_error);

            if (signaled_count > 1) {
                me->log_WARNING_LO_RxError(ZephyrUartErrorType::SIGNAL_BUFFER_ERROR);
            }

            // CHECK: Error conditions (negative values)
            if (bytes_or_error < 0) {
                int error_flags = -bytes_or_error;  // Make positive for bit checking

                // Check specific error types
                if (error_flags == SIGNAL_ERR_RING_OVERFLOW ||
                    (error_flags & SIGNAL_ERR_RING_OVERFLOW)) {
                    me->log_WARNING_LO_RxError(ZephyrUartErrorType::RING_BUFFER_OVERFLOW);
                }

                if (error_flags & SIGNAL_ERR_UART_OVERRUN) {
                    me->log_WARNING_LO_RxError(ZephyrUartErrorType::UART_OVERRUN_ERROR);
                }

                if (error_flags & SIGNAL_ERR_UART_PARITY) {
                    me->log_WARNING_LO_RxError(ZephyrUartErrorType::UART_PARITY_ERROR);
                }

                if (error_flags & SIGNAL_ERR_UART_FRAMING) {
                    me->log_WARNING_LO_RxError(ZephyrUartErrorType::UART_FRAMING_ERROR);
                }

                if (error_flags & SIGNAL_ERR_UART_BREAK) {
                    me->log_WARNING_LO_RxError(ZephyrUartErrorType::UART_BREAK_ERROR);
                }
            }

            k_poll_signal_reset(&me->m_rxSignal);
            events[0].state = K_POLL_STATE_NOT_READY;

            // Drain ring buffer
            while (ring_buf_size_get(&me->m_RxRingbuf) > 0) {
                // Check available space first
                uint32_t available = ring_buf_size_get(&me->m_RxRingbuf);
                uint32_t to_read = (available > 512) ? 512 : available;  // Larger chunks

                Fw::Buffer rxBuff = me->allocate_out(0, to_read);
                FW_ASSERT(rxBuff.isValid());

                U32 read = ring_buf_get(&me->m_RxRingbuf,
                                    reinterpret_cast<uint8_t *>(rxBuff.getData()),
                                    rxBuff.getSize());
                if (read > 0) {
                    rxBuff.setSize(read);
                    me->recv_out(0, rxBuff, Drv::ByteStreamStatus::OP_OK);
                } else {
                    me->deallocate_out(0, rxBuff);
                    break;  // Nothing left to read
                }
            }
        }
    }

    void ZephyrInterruptUartDriver::configure(const struct device *dev, U32 baud_rate) {
        FW_ASSERT(dev != nullptr);
        this->m_dev = dev;

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

        Os::TaskString taskName("IsrRcv");
        Os::Task::ParamType priority = Os::Task::TASK_PRIORITY_DEFAULT;
        Os::Task::ParamType stackSize = 8 * 1024;
        Os::Task::ParamType cpuAffinity = Os::Task::TASK_DEFAULT;
        Os::Task::Arguments arguments(taskName, handle_isrRcv, this, priority, stackSize, cpuAffinity);
        Os::Task::Status status;
        status = this->m_isrRcvTask.start(arguments);
        FW_ASSERT(status == Os::Task::Status::OP_OK, status);

        this->m_rxBuff = this->allocate_out(0, RING_SIZE);
        FW_ASSERT(this->m_rxBuff.isValid());
        ring_buf_init(&this->m_RxRingbuf, this->m_rxBuff.getSize(),
                      reinterpret_cast<uint8_t*>(this->m_rxBuff.getData()));

        this->m_txBuff = this->allocate_out(0, RING_SIZE);
        FW_ASSERT(this->m_txBuff.isValid());
        ring_buf_init(&this->m_TxRingbuf, this->m_txBuff.getSize(),
                      reinterpret_cast<uint8_t*>(this->m_txBuff.getData()));

        uart_irq_callback_user_data_set(this->m_dev, this->uartISR, static_cast<void*>(this));
        uart_irq_rx_enable(this->m_dev);
        if (this->isConnected_ready_OutputPort(0)) {
            this->ready_out(0);
        }
    }

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------
    void ZephyrInterruptUartDriver ::
        asyncSend_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        // Copy to ring buffer
        uint32_t written = ring_buf_put(&this->m_TxRingbuf, reinterpret_cast<uint8_t*>(sendBuffer.getData()),
                                        sendBuffer.getSize());

        if (written == sendBuffer.getSize()) {
            uart_irq_tx_enable(this->m_dev);  // Start transmission
            this->drvAsyncSendReturnOut_out(0, sendBuffer, Drv::ByteStreamStatus::OP_OK);
        } else {
            // Ring buffer full
            this->drvAsyncSendReturnOut_out(0, sendBuffer, Drv::ByteStreamStatus::SEND_RETRY);
        }
    }

    Drv::ByteStreamStatus ZephyrInterruptUartDriver ::
        send_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        // This driver doesn't support this send command
        return Drv::ByteStreamStatus::OTHER_ERROR;
    }

    void ZephyrInterruptUartDriver ::recvReturnIn_handler(const FwIndexType portNum, Fw::Buffer &returnBuffer) {
        this->deallocate_out(0, returnBuffer);
    }

} // end namespace Zephyr
