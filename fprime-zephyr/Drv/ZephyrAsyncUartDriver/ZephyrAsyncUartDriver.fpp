module Zephyr {

  passive component ZephyrAsyncUartDriver {
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

    ###############################################################################
    # Standard AC Ports for Events
    ###############################################################################
    @ Port for requesting the current time
    time get port timeCaller

    @ Port for sending textual representation of events
    text event port logTextOut

    @ Port for sending events to downlink
    event port logOut

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------
    @ Invalid PUS packet structure
    event ZEPHYR_UART_STATE_CHANGE(
        driverStateId: U32,
        driverStateStr: string size 25,
    ) \
    severity activity low \
    format "{} {}"
  }
}
