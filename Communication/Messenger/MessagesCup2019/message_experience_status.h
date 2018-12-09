#ifndef _MESSAGES_EXPERIENCE_STATUS
#define _MESSAGES_EXPERIENCE_STATUS

#include "messagebase.h"

class Message_EXPERIENCE_STATUS : public MessageBase
{
public:
    Message_EXPERIENCE_STATUS();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    short ExperienceStatus;   // seconds
};

#endif // _MESSAGES_EXPERIENCE_STATUS
