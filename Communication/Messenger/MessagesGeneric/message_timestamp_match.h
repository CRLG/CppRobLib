#ifndef _MESSAGES_TIMESTAMP_MATCH
#define _MESSAGES_TIMESTAMP_MATCH

#include "messagebase.h"

class Message_TIMESTAMP_MATCH : public MessageBase
{
public:
    Message_TIMESTAMP_MATCH();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    typedef enum {
        MATCH_WAITING_FOR_START = -1,
        MATCH_END = 9999
    }TimestampEnum;
    short Timestamp;   // seconds (special values possible)
};

#endif // _MESSAGES_TIMESTAMP_MATCH
