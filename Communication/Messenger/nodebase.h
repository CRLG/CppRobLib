#ifndef _NODEBASE_H_
#define _NODEBASE_H_

class DatabaseBase;
class MessengerInterfaceBase;

// ====================================================
//        NODE
// ====================================================
class NodeBase
{
public:
    NodeBase();
    virtual ~NodeBase();

    virtual const char *getName()=0;
    virtual unsigned short getID()=0;

    virtual bool isPresent();
    virtual void diagPresence(long current_time);  // this method must be called periodically to check communication lost

    void setDatabase(DatabaseBase *database);
    void setMessengerInterface(MessengerInterfaceBase* messenger_interface);

    void setLostComDuration(long lost_time); // [msec]
    void newMessageReceived(long current_time);

private :
    long m_lost_time_duration;          // Maximum time without message received before lost communication is declared
    long m_last_time_receive_message;   // Datation of the last message received
    bool m_present;

    DatabaseBase *m_database;
    MessengerInterfaceBase *m_messenger_interface;
};

#endif // _NODEBASE_H_
