#include <stdio.h>

#include "sm_main.h"
#include "ia.h"

SM_Main::SM_Main()
{
}

const char* SM_Main::getName()
{
    return "SM_Main";
}

const char* SM_Main::stateToName(unsigned short state)
{
    switch(state)
    {
    case ATTENTE_TIRETTE :          return "ATTENTE_TIRETTE";
    case MATCH_EN_COURS :           return "MATCH_EN_COURS";
    case FIN_MATCH :                return "FIN_MATCH";
    }
    return "UNKNOWN_STATE";
}


// _____________________________________
bool SM_Main::isMatchEnCours(void)
{
    return (m_state==MATCH_EN_COURS);
}
// _____________________________________
bool SM_Main::isMatchTermine(void)
{
    return (m_state==FIN_MATCH);
}

// _____________________________________
void SM_Main::step()
{
    switch(m_state)
    {
    // ___________________________________
    case ATTENTE_TIRETTE :
        if (onEntry()) {
        }
        // Couleur de l'équipe
        internals()->couleur_equipe = inputs()->dde_couleur_equipe;
        internals()->inverse_consignes_XYTeta = (internals()->couleur_equipe==SM_DatasInterface::EQUIPE_COULEUR_2);
        // Autotests
        if (inputs()->dde_test_actionneurs) {
            m_ia ->m_sm_autotest.start();
            inputs()->dde_test_actionneurs = 0;
        }

        gotoStateIfTrue(MATCH_EN_COURS, inputs()->Tirette);
        if (onExit()) {
            internals()->TempsMatch = 0;
            m_ia->m_sm_autotest.stop();
        }
        break;
    // ___________________________________
    case MATCH_EN_COURS :
        if (onEntry()) {
            m_ia->m_sm_sequenceur.start();
        }
        internals()->TempsMatch += inputs()->TE_Modele;
        gotoStateIfTrue(FIN_MATCH, internals()->TempsMatch > DUREE_MATCH);

        if (onExit()) {
            m_ia->stopAllOthersStateMachines(this);
        }
        break;
    // ___________________________________
    case FIN_MATCH :
        if (onEntry()) {
            outputs()->stopAll();
        }
        m_ia->getScore();

        if (onExit()) {
        }
        break;
    default :
        break;
    }
}
