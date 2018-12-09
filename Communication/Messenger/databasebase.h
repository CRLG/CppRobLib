#ifndef _DATABASE_BASE_H
#define _DATABASE_BASE_H

#include "messengerdef.h"

class MessageBase;
class TransporterBase;
class MessengerEventBase;

// ====================================================
//        BASE CLASS FOR MESSAGE
// ====================================================
class DatabaseBase
{
public:
    DatabaseBase();
    virtual ~DatabaseBase();

    virtual void decode(const tMessengerFrame* frame);
    virtual void encode(MessageBase* msg);

    virtual const char *getName();
    virtual unsigned short getMessageCount() = 0;  // child provide the number of message in the database

    void setTransporter(TransporterBase *messenger);
    void setEventManager(MessengerEventBase* event_mgr);

protected :
    MessageBase **m_p_messages_list;   // pointer table (table is allocated by child)

    TransporterBase *m_transporter;
    MessengerEventBase *m_event_mgr;

    void initMessages();
};
#endif // _DATABASE_BASE_H
