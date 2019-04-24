#ifndef _MESSAGES_BALISE_POSITIONS
#define _MESSAGES_BALISE_POSITIONS

#include "messagebase.h"

class Message_BALISE_POSITIONS : public MessageBase
{
public:
    Message_BALISE_POSITIONS();

    static const short UNKNOWN_POSITION = -1;

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    short PosX_Grosbot;
    short PosY_Grosbot;
    short PosX_Minibot;
    short PosY_Minibot;
    short PosX_Adversaire1;
    short PosY_Adversaire1;
    short PosX_Adversaire2;
    short PosY_Adversaire2;
};

#endif // _MESSAGES_BALISE_POSITIONS
