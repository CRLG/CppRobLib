#ifndef _MESSENGER_INTERFACE_BASE_H_
#define _MESSENGER_INTERFACE_BASE_H_
#include <stdio.h>
#include "transporterbase.h"
#include "messengerdef.h"

// ====================================================
//       MESSENGER INTERFACE CLASS
// ====================================================

class MessageBase;
class TransporterBase;
class DatabaseBase;

class MessengerInterfaceBase
{
public:
    MessengerInterfaceBase();
    virtual ~MessengerInterfaceBase();

    void init(TransporterBase *transporter, DatabaseBase *database);

    // Messenger entry point for application
    // This methods must be called each time a data or a buffer is received
    virtual void decode(unsigned char newData, unsigned short source_address=0);
    virtual void decode(unsigned char *buff_data, unsigned char buff_size, unsigned short source_address=0);

    // Messenger exit point
    // Pure virtual methods
    // This method is called by messenger to inform a buffer is ready to send on specific hardware or use by application
    virtual void encode(unsigned char *buff_data, unsigned short buff_size, unsigned short dest_address=0) = 0;

    // Events
    // This method is called by messenger (transporter) to inform a valid frame was received
    virtual void newFrameReceived(tMessengerFrame *frame);
    // This method is called by messenger (transporter) to inform a frame was transmited
    virtual void frameTransmited(tMessengerFrame *frame);
    // This method is called by messenger (database) to inform a known message was received
    virtual void newMessageReceived(MessageBase *msg);
    // This method is called by messenger (database) to inform a message was transmited
    virtual void messageTransmited(MessageBase *msg);
    // This method is called by messenger (message) to inform a data in a message was updated
    virtual void dataUpdated(char *name, char *val_str);
    // This method is called by messenger (message) to inform a data in a message changed value
    virtual void dataChanged(char *name, char *val_str);

protected :
    TransporterBase *m_transporter;
    DatabaseBase *m_database;
};

#endif // _MESSENGER_INTERFACE_BASE_H_
