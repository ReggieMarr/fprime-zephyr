// ======================================================================
// \title  ZephyrInterruptUartDriver.hpp
// \author reggiemarr
// \brief  hpp file for ZephyrInterruptUartDriver component implementation class
// ======================================================================

#ifndef Zephyr_ZephyrInterruptUartDriver_HPP
#define Zephyr_ZephyrInterruptUartDriver_HPP

#include "fprime-zephyr/Drv/ZephyrInterruptUartDriver/ZephyrInterruptUartDriverComponentAc.hpp"

namespace Zephyr {

class ZephyrInterruptUartDriver final
    : public ZephyrInterruptUartDriverComponentBase {

public:
  // ----------------------------------------------------------------------
  // Component construction and destruction
  // ----------------------------------------------------------------------

  //! Construct ZephyrInterruptUartDriver object
  ZephyrInterruptUartDriver(const char *const compName //!< The component name
  );

  //! Destroy ZephyrInterruptUartDriver object
  ~ZephyrInterruptUartDriver();

private:
  // ----------------------------------------------------------------------
  // Handler implementations for typed input ports
  // ----------------------------------------------------------------------

  //! Handler implementation for asyncSend
  //!
  //! Invoke this port to send data out the driver (synchronous)
  //! Status is returned, and ownership of the buffer is retained by the caller
  void asyncSend_handler(FwIndexType portNum, //!< The port number
                         Fw::Buffer &fwBuffer //!< The buffer
                         ) override;

  //! Handler implementation for recvReturnIn
  //!
  //! Port receiving back ownership of data sent out on $recv port
  void recvReturnIn_handler(FwIndexType portNum, //!< The port number
                            Fw::Buffer &fwBuffer //!< The buffer
                            ) override;

  //! Handler implementation for send
  //!
  //! Invoke this port to send data out the driver (synchronous)
  //! Status is returned, and ownership of the buffer is retained by the caller
  Drv::ByteStreamStatus send_handler(FwIndexType portNum,   //!< The port number
                                     Fw::Buffer &sendBuffer //!< Data to send
                                     ) override;

private:
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined internal interfaces
  // ----------------------------------------------------------------------

  //! Handler implementation for isrRcv
  //!
  //! Port for receiving rx buffers (note this is essentially an internal Fw.Com
  //! buffer)
  void isrRcv_internalInterfaceHandler(const Fw::ComBuffer &rxData,
                                       U32 context) override;
};

} // namespace Zephyr

#endif
