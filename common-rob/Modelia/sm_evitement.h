#ifndef SM_EVITEMENT_H
#define SM_EVITEMENT_H

#include "sm_statemachinebase.h"


class SM_Evitement : public SM_StateMachineBase
{
public:
    SM_Evitement();

    void step();
    const char* getName();
    const char* stateToName(unsigned short state);

    typedef enum {
        // Etat généraux
        EVITEMENT_INIT = SM_StateMachineBase::SM_FIRST_STATE,
        EVITEMENT_INIT_CHOICE,
        STRATEGIE_EVITEMENT_FIN,
        SORTIE_EVITEMENT,
        EVITEMENT_ATTENTE,

        // Stratégie d'évitement : ATTENDRE
        STRATEGIE_ATTENDRE,

        // Stratégie d'évitement : CONTOURNEMENT
        STRATEGIE_CONTOURNEMENT,
        STRATEGIE_CONTOURNEMENT_ELOIGNEMENT,
        STRATEGIE_CONTOURNEMENT_ROTATION,
        STRATEGIE_CONTOURNEMENT_EVACUE,
        STRATEGIE_CONTOURNEMENT_REDRESSE,
        STRATEGIE_CONTOURNEMENT_EVACUE_2,


        STATE_5,
        STATE_6,
        STATE_7,
        STATE_8,
        STATE_9,
        STATE_10,
        STATE_11,
        STATE_12,
        STATE_13,
        STATE_14,
        STATE_15,
    }tState;

private :
    // Sauvegarde du contexte à l'entrée de la stratégie d'évitement
    // et restauration du contexte à la sortie
    void saveContext();
    void restoreContext();

};

#endif // SM_EVITEMENT_H
