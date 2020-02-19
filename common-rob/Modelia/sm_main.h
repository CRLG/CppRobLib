#ifndef SM_MAIN_H
#define SM_MAIN_H

#include "sm_statemachinebase.h"

class SM_Main : public SM_StateMachineBase
{
public:
    SM_Main();
    const char* getName();
    const char* stateToName(unsigned short state);

    void step();

    bool isMatchEnCours(void);
    bool isMatchTermine(void);

private :
    typedef enum {
        ATTENTE_TIRETTE = SM_StateMachineBase::SM_FIRST_STATE,
        MATCH_EN_COURS,
        FIN_MATCH
    }tState;
};

#endif // SM_MAIN_H
