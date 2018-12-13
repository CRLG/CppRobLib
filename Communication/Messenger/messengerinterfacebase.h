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
    MessengerInterfaceBase();
    virtual ~MessengerInterfaceBase();

    void setTransporter(TransporterBase *transporter);

    // pure virtual methods (messenger outpout point).
    // this method is called by messenger to inform a buffer is ready to send on specific hardware or use by application
    virtual void encode(unsigned char *buff_data, unsigned short buff_size, unsigned short dest_address=0) = 0;

    // this methods must be called each time a data or a buffer is received
    virtual void decode(unsigned char newData, unsigned short source_address=0);
    virtual void decode(unsigned char *buff_data, unsigned char buff_size, unsigned short source_address=0);

protected :
    TransporterBase *m_transporter;
};

#endif // _MESSENGER_INTERFACE_BASE_H_
