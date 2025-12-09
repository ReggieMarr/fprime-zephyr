module Zephyr {
  enum ZephyrUartErrorType {
      SIGNAL_BUFFER_ERROR,
      RING_BUFFER_OVERFLOW,
      UART_OVERRUN_ERROR,      # Hardware FIFO overflow
      UART_PARITY_ERROR,       # Parity error
      UART_FRAMING_ERROR,      # Framing error
      UART_BREAK_ERROR         # Break condition
  };

  passive component ZephyrInterruptUartDriver {
    import Drv.ByteStreamDriver

    @Allocate new buffer
    output port allocate: Fw.BufferGet

    @return the allocated buffer
    output port deallocate: Fw.BufferSend

    @return the buffer previously received for sending
    output port drvAsyncSendReturnOut: Drv.ByteStreamData

    @ Invoke this port to send data out the driver (synchronous)
    @ Status is returned, and ownership of the buffer is retained by the caller
    sync input port asyncSend: Fw.BufferSend

    @ Port for requesting the current time
    time get port timeCaller

    @ Port for sending events to downlink
    event port logOut

    @ Port for sending textual representation of events
    text event port logTextOut

    event RxError(errorType: ZephyrUartErrorType) \
            severity warning low \
            format "{}" \
            throttle 250
  }
}
