#ifndef _XBEE_DRIVER_INTERFACE_BASE_H_
#define _XBEE_DRIVER_INTERFACE_BASE_H_
#include <stdio.h>
#include "xbeedriver.h"

// ====================================================
//       MESSENGER INTERFACE CLASS
// ====================================================

class XbeeDriverInterfaceBase
{
public:
    XbeeDriverInterfaceBase() { m_xbee_driver.setXbeeInterface(this);}
    virtual ~XbeeDriverInterfaceBase() { }

    // pure virtual methods.
    // this method is called by driver to inform a valid buffer ws receinved and now ready to be used by application
    virtual void readyBytes(unsigned char *buff_data, unsigned char buff_size, unsigned short source_id=0)=0;
    // this method is called by driver to write a buffer to physical serial port on specific hardware
    virtual void write(unsigned char *buff_data, unsigned char buff_size)=0;
    // this method is called by driver to request a delay on specific hardware
    virtual void delay_us(unsigned long delay)=0;

    // this methods must be called by application each time a data or buffer is received from specific hardware
    virtual void decode(unsigned char newData) { m_xbee_driver.decode(newData); }
    virtual void decode(unsigned char *buff_data, unsigned char buff_size) { m_xbee_driver.decode(buff_data, buff_size); }

    // this method must be called by application to encode a buffer in a xbee format and write it
    virtual void encode(unsigned char *buff_data, unsigned short buff_size, unsigned short dest_address=0) { m_xbee_driver.encode(buff_data, buff_size, dest_address);}

protected :
    XbeeDriver m_xbee_driver;
};

#endif // _XBEE_DRIVER_INTERFACE_BASE_H_
