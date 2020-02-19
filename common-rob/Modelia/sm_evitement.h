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
        EVITEMENT_INIT = SM_StateMachineBase::SM_FIRST_STATE,
        EVITEMENT_INIT_CHOICE,
        STRATEGIE_EVITEMENT,
        SORTIE_EVITEMENT,
        EVITEMENT_ATTENTE,
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


    typedef enum {
        STRATEGIE_EVITEMENT_ATTENDRE = 0,
        STRATEGIE_EVITEMENT_CONTOURNER
    }tChoixStrategie;
};

#endif // SM_EVITEMENT_H
