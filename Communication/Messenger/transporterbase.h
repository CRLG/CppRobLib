#ifndef _TRANSPORTER_BASE_H
#define _TRANSPORTER_BASE_H

#include "messengerdef.h"

class DatabaseBase;
class MessengerInterfaceBase;
class MessengerEventBase;

// ====================================================
//        ABSTRACT DRIVER CLASS FOR XBEE
// ====================================================
class TransporterBase
{
public:
    TransporterBase();
    virtual ~TransporterBase();

    // this methods must be called each time a data or a buffer is received
    virtual void decode(unsigned char newData, unsigned short source_address=0) = 0;
    virtual void decode(unsigned char *buff_data, unsigned char buff_size, unsigned short source_address=0)=0;
    // entry point to encapsulate and send message
    virtual void encode(tMessengerFrame *frame) =  0;

    void setDatabase(DatabaseBase* database);
    void setMessengerInterface(MessengerInterfaceBase* mesenger_interface);
    void setEventManager(MessengerEventBase* event_mgr);

protected :
    tMessengerFrame m_current_frame;

    DatabaseBase *m_database;
    MessengerInterfaceBase *m_messenger_interface;
    MessengerEventBase *m_event_mgr;

    void readyFrame(tMessengerFrame* frame);
};

#endif // _TRANSPORTER_BASE_H
