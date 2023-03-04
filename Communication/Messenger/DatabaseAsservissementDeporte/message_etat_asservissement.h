#ifndef _MESSAGES_ETAT_ASSERVISSEMENT_H_
#define _MESSAGES_ETAT_ASSERVISSEMENT_H_

#include "messagebase.h"

class Message_ETAT_ASSERVISSEMENT : public MessageBase
{
public:
    Message_ETAT_ASSERVISSEMENT();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    float X_robot;
    float Y_robot;
    float angle_robot;
    char convergence_rapide;
    char convergence_conf;						// 0: Mvt en cours, 1: Convergence ok, 2: Blocage détecté
    char diag_blocage;
    unsigned char ModeAsservissement;
    float cde_moteur_D;
    float cde_moteur_G;
    unsigned short last_cde_id;
};

#endif // _MESSAGES_ETAT_ASSERVISSEMENT_H_
