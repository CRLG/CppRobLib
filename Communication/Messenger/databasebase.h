#ifndef _DATABASE_BASE_H
#define _DATABASE_BASE_H

#include "messengerdef.h"

class MessageBase;
class TransporterBase;
class MessengerInterfaceBase;

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
    void setMessengerInterface(MessengerInterfaceBase* messenger_interface);
    void checkAndSendPeriodicMessages(long current_time);

    void restart();
protected :
    MessageBase **m_p_messages_list;   // pointer table (table is allocated by child)

    TransporterBase *m_transporter;
    MessengerInterfaceBase *m_messenger_interface;

    void initMessages();
};
#endif // _DATABASE_BASE_H
