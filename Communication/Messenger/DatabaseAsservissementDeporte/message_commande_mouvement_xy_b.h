#ifndef _MESSAGES_COMMANDE_MOUVEMENT_XY_B_H_
#define _MESSAGES_COMMANDE_MOUVEMENT_XY_B_H_

#include "messagebase.h"

class Message_COMMANDE_MOUVEMENT_XY_B : public MessageBase
{
public:
    Message_COMMANDE_MOUVEMENT_XY_B();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    unsigned short id;
    float x;
    float y;
};

#endif // _MESSAGES_COMMANDE_MOUVEMENT_XY_B_H_
