#ifndef _MESSAGE_BASE_H
#define _MESSAGE_BASE_H

// Message Direction (bitfiled)
typedef enum {
    MSG_RX = 0x1,
    MSG_TX = 0x2,
    MSG_RXTX  = (MSG_RX|MSG_TX)
}tMessageDirection;

class DatabaseBase;
class MessengerInterfaceBase;

// ====================================================
//        BASE CLASS FOR MESSAGES
// ====================================================
class MessageBase
{
public:
    MessageBase();
    virtual ~MessageBase();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);
    virtual void send();

    void setDatabase(DatabaseBase *database);
    void setMessengerInterface(MessengerInterfaceBase* messenger_interface);
    void setDirection(tMessageDirection direction);
    void setDestinationAddress(unsigned short address);
    void setSourceAddress(unsigned short address);

    inline unsigned short getID() { return m_id; }
    inline unsigned short getDLC() { return m_dlc; }
    inline tMessageDirection getDirection() { return m_direction; }
    inline unsigned short getDestinationAddress() { return m_destination_address; }
    inline unsigned short getSourceAddress() { return m_source_address; }

protected :
    unsigned short m_id;
    unsigned char m_dlc;
    tMessageDirection m_direction;
    unsigned short m_destination_address;
    unsigned short m_source_address;

    DatabaseBase *m_database;
    MessengerInterfaceBase *m_messenger_interface;
};

#endif // _MESSAGE_BASE_H
