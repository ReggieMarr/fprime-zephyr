// ======================================================================
// \title  ZephyrInterruptUartDriver.cpp
// \author reggiemarr
// \brief  cpp file for ZephyrInterruptUartDriver component implementation class
// ======================================================================

#include "fprime-zephyr/Drv/ZephyrInterruptUartDriver/ZephyrInterruptUartDriver.hpp"

namespace Zephyr {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

ZephyrInterruptUartDriver ::ZephyrInterruptUartDriver(
    const char *const compName)
    : ZephyrInterruptUartDriverComponentBase(compName) {}

ZephyrInterruptUartDriver ::~ZephyrInterruptUartDriver() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void ZephyrInterruptUartDriver ::asyncSend_handler(FwIndexType portNum,
                                                   Fw::Buffer &fwBuffer) {
  // TODO
}

void ZephyrInterruptUartDriver ::recvReturnIn_handler(FwIndexType portNum,
                                                      Fw::Buffer &fwBuffer) {
  // TODO
}

Drv::ByteStreamStatus
ZephyrInterruptUartDriver ::send_handler(FwIndexType portNum,
                                         Fw::Buffer &sendBuffer) {
  // TODO return
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined internal interfaces
// ----------------------------------------------------------------------

void ZephyrInterruptUartDriver ::isrRcv_internalInterfaceHandler(
    const Fw::ComBuffer &rxData, U32 context) {
  // TODO
}

} // namespace Zephyr
