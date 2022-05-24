#ifndef _MESSAGES_COMMANDE_EXPERIENCE_H_
#define _MESSAGES_COMMANDE_EXPERIENCE_H_

#include "messagebase.h"

class Message_COMMANDE_EXPERIENCE : public MessageBase
{
public:
    Message_COMMANDE_EXPERIENCE();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    typedef enum {
        EXPERIENCE_CMD_NO_ACTION = 0,
        EXPERIENCE_CMD_AUTOTEST,
        EXPERIENCE_CMD_START,
        EXPERIENCE_CMD_STOP,
    }ExperienceCmdEnum;
    short ExperienceCmd;
};

#endif // _MESSAGES_COMMANDE_EXPERIENCE_H_
