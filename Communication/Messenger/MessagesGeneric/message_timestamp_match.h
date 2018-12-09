#ifndef _MESSAGES_TIMESTAMP_MATCH
#define _MESSAGES_TIMESTAMP_MATCH

#include "messagebase.h"

class Message_TIMESTAMP_MATCH : public MessageBase
{
public:

#define MATCH_WAITING_FOR_START (-1)
#define MATCH_END 9999

    Message_TIMESTAMP_MATCH();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    short Timestamp;   // seconds
};

#endif // _MESSAGES_TIMESTAMP_MATCH
