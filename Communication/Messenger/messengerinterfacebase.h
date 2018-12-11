#ifndef _MESSENGER_INTERFACE_BASE_H_
#define _MESSENGER_INTERFACE_BASE_H_
#include <stdio.h>
#include "transporterbase.h"

// ====================================================
//       MESSENGER INTERFACE CLASS
// ====================================================

class MessengerInterfaceBase
{
public:
    MessengerInterfaceBase() : m_transporter(NULL) { }
    virtual ~MessengerInterfaceBase() { }

    inline void setTransporter(TransporterBase *transporter) { m_transporter = transporter; }

    // pure virtual methods (messenger outpout point).
    // this method is called by messenger to inform a buffer is ready to send on specific hardware or use by application
    virtual void encode(unsigned char *buff_data, unsigned short buff_size, unsigned short dest_address=0) = 0;

    // this methods must be called each time a data or a buffer is received
    virtual void decode(unsigned char newData, unsigned short source_address=0) { if (m_transporter) m_transporter->decode(newData, source_address); }
    virtual void decode(unsigned char *buff_data, unsigned char buff_size, unsigned short source_address=0) { if (m_transporter) m_transporter->decode(buff_data, buff_size, source_address); }

protected :
    TransporterBase *m_transporter;
};

#endif // _MESSENGER_INTERFACE_BASE_H_
