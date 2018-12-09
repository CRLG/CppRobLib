#ifndef _MESSENGER_EVENT_BASE_H_
#define _MESSENGER_EVENT_BASE_H_

#include "messengerdef.h"

class MessageBase;

class MessengerEventBase
{
public:
    MessengerEventBase() { }
    virtual ~MessengerEventBase() { }

    virtual void newFrameReceived(tMessengerFrame *frame) { (void)frame; }
    virtual void frameTransmited(tMessengerFrame *frame) { (void)frame; }
    virtual void newMessageReceived(MessageBase *msg) { (void)msg; }
    virtual void messageTransmited(MessageBase *msg) { (void)msg; }
    virtual void dataUpdated(char *name, char *val_str) { (void)name; (void)val_str; }
    virtual void dataChanged(char *name, char *val_str) { (void)name; (void)val_str; }
};

#endif // _MESSENGER_EVENT_BASE_H_
