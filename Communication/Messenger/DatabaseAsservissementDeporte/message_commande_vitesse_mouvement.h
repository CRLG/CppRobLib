#ifndef _MESSAGES_COMMANDE_VITESSE_MOUVEMENT_H_
#define _MESSAGES_COMMANDE_VITESSE_MOUVEMENT_H_

#include "messagebase.h"

class Message_COMMANDE_VITESSE_MOUVEMENT : public MessageBase
{
public:
    Message_COMMANDE_VITESSE_MOUVEMENT();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    unsigned short id;
    float vitesse_avance;
    float vitesse_angle;
};

#endif // _MESSAGES_COMMANDE_VITESSE_MOUVEMENT_H_
