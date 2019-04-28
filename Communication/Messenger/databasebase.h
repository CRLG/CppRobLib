#ifndef _DATABASE_BASE_H
#define _DATABASE_BASE_H

#include "messengerdef.h"

class MessageBase;
class TransporterBase;
class MessengerInterfaceBase;
class NodeBase;

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
    virtual unsigned short getNodeCount() = 0;  // child provide the number of node in the database
    virtual const char *nodeIdToName(unsigned short id);

    void setTransporter(TransporterBase *messenger);
    void setMessengerInterface(MessengerInterfaceBase* messenger_interface);
    void checkAndSendPeriodicMessages();
    void checkNodeCommunication();

    void restart();

    MessageBase **getMessagesList() const;
    NodeBase **getNodesList() const;

    NodeBase* nodeIdToNode(unsigned short id);
protected :
    MessageBase **m_p_messages_list;   // pointer table (table is allocated by child)
    NodeBase **m_p_nodes_list;         // pointer table (table is allocated by child)

    TransporterBase *m_transporter;
    MessengerInterfaceBase *m_messenger_interface;

    void initMessages();
    void initNodes();
};
#endif // _DATABASE_BASE_H
