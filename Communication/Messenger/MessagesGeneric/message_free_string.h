#ifndef _MESSAGES_FREE_STRING
#define _MESSAGES_FREE_STRING

#include "messagebase.h"

class Message_FREE_STRING : public MessageBase
{
public:
    Message_FREE_STRING();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    static const unsigned char STR_BUFFER_SIZE = 64;
    char str[STR_BUFFER_SIZE];
};

#endif // _MESSAGES_FREE_STRING
