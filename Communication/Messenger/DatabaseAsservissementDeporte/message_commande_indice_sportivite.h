#ifndef _MESSAGES_COMMANDE_INDICE_SPORTIVITE_H_
#define _MESSAGES_COMMANDE_INDICE_SPORTIVITE_H_

#include "messagebase.h"

class Message_COMMANDE_INDICE_SPORTIVITE : public MessageBase
{
public:
    Message_COMMANDE_INDICE_SPORTIVITE();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    unsigned short id;
    float indice_sportivite;
};

#endif // _MESSAGES_COMMANDE_INDICE_SPORTIVITE_H_
