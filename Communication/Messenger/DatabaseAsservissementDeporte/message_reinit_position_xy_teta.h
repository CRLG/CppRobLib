#ifndef _MESSAGES_REINIT_POSITION_XY_TETA_H_
#define _MESSAGES_REINIT_POSITION_XY_TETA_H_

#include "messagebase.h"

class Message_REINIT_POSITION_XY_TETA : public MessageBase
{
public:
    Message_REINIT_POSITION_XY_TETA();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    unsigned short id;
    float x;
    float y;
    float teta;
};

#endif // _MESSAGES_REINIT_POSITION_XY_TETA_H_
