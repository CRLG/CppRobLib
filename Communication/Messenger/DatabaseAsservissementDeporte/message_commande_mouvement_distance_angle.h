#ifndef _MESSAGES_COMMANDE_MOUVEMENT_DISTANCE_ANGLE_H_
#define _MESSAGES_COMMANDE_MOUVEMENT_DISTANCE_ANGLE_H_

#include "messagebase.h"

class Message_COMMANDE_MOUVEMENT_DISTANCE_ANGLE : public MessageBase
{
public:
    Message_COMMANDE_MOUVEMENT_DISTANCE_ANGLE();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    unsigned short id;
    float distance;
    float angle;
};

#endif // _MESSAGES_COMMANDE_MOUVEMENT_DISTANCE_ANGLE_H_
