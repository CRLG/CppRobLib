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

    typedef enum {
        EXPERIENCE_WAITING_FOR_START = 0,
        EXPERIENCE_IN_PROGRESS,
        EXPERIENCE_FINISHED,
        EXPERIENCE_ERROR = -1
    }ExperienceStatusEnum;
    short ExperienceStatus;
};

#endif // _MESSAGES_EXPERIENCE_STATUS
