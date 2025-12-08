module Zephyr {

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
  }
}
