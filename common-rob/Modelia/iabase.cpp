#include "iabase.h"

IABase::IABase()
    : m_sm_debug(nullptr),
      m_state_machine_count(0)
{
    for (int i=0; i<NBRE_MAX_STATE_MACHINES; i++) {
        m_sm_liste[i] = nullptr;
    }

    // States machines qu'on retrouvera à toutes les coupes et sur tous les robots
    m_sm_liste[m_state_machine_count++] = &m_sm_main;
    m_sm_liste[m_state_machine_count++] = &m_sm_sequenceur;
    m_sm_liste[m_state_machine_count++] = &m_sm_evitement;
}

IABase::~IABase()
{
}

// ________________________________________________
void IABase::setDebugger(SM_DebugInterface* sm_debuger)
{
    m_sm_debug = sm_debuger;
}

// ________________________________________________
void IABase::stopAllStateMachines()
{
    for (int i=0; i<m_state_machine_count; i++) {
        if (m_sm_liste[i]) {
            m_sm_liste[i]->stop();
        }
    }
}

// ________________________________________________
void IABase::stopAllOthersStateMachines(SM_StateMachineBase *sm_to_keep_alive)
{
    for (int i=0; i<m_state_machine_count; i++) {
        if (m_sm_liste[i] && (m_sm_liste[i] != sm_to_keep_alive)) {
            m_sm_liste[i]->stop();
        }
    }
}

// ________________________________________________
bool IABase::isOneMainMissionActive()
{
    for (int i=0; i<m_state_machine_count; i++) {
        if (m_sm_liste[i] && m_sm_liste[i]->isMainMission() && m_sm_liste[i]->isActive()) {
            return true;
        }
    }
    return false;
}

// ________________________________________________
SM_StateMachineBase* IABase::activeNextPertinentMainMission(tAlgoNextPertinentMission algo)
{
    SM_StateMachineBase* next_mission = nullptr;
    int score_max = -1;
    unsigned short priorite = 0xFFFF;

    switch (algo) {
    // TODO : implémenter les autres algos
    // __________________________________
    // Dans cet algo, on recherche la mission qui n'est pas terminée et qui rapportera le plus de point
    case ALGO_PERTINENT_MISSION_SCORE_MAX:
        for (int i=0; i<m_state_machine_count; i++) {
            if (m_sm_liste[i] && m_sm_liste[i]->isMainMission() && m_sm_liste[i]->isEnabled() && !m_sm_liste[i]->isSucces()) {
                if (m_sm_liste[i]->getScoreMax() > score_max) {
                    next_mission = m_sm_liste[i];
                    score_max = m_sm_liste[i]->getScoreMax();
                }
            }
        }
        if (next_mission) activeMainMission(next_mission);
        break;
     // __________________________________
     case ALGO_PERTINENT_MISSION_CHOIX_PRIORITE:
        for (int i=0; i<m_state_machine_count; i++) {
            if (m_sm_liste[i] && m_sm_liste[i]->isMainMission() && m_sm_liste[i]->isEnabled() && !m_sm_liste[i]->isSucces()) {
                if (m_sm_liste[i]->getPrioriteExecution() < priorite) {  // petite valeur = le plus prioritaire
                    next_mission = m_sm_liste[i];
                    priorite = m_sm_liste[i]->getPrioriteExecution();
                }
            }
        }
        if (next_mission) activeMainMission(next_mission);
        break;
    // __________________________________
    //case ALGO_PERTINEMENT_MISSION_DISTANCE_MIN
    //break;

    // __________________________________
    // Dans cet algo, la prochaine mission à exécuter est la prochaine de la liste non exécutée
    case ALGO_PERTINENT_MISSION_NEXT_INDEX:
    default :
        for (int i=0; i<m_state_machine_count; i++) {
            if ((m_sm_liste[i]) && (m_sm_liste[i]->isMainMission()) && (m_sm_liste[i]->isEnabled()) && (!m_sm_liste[i]->isSucces())) {
                next_mission = m_sm_liste[i];
                activeMainMission(next_mission);
                break;
            }
        }
        break;
    }
    return next_mission;
}


// ________________________________________________
// Active une seule mission principale
void IABase::activeMainMission(SM_StateMachineBase *mission)
{
    for (int i=0; i<m_state_machine_count; i++) {
        if (m_sm_liste[i] && m_sm_liste[i]->isMainMission() ) {
            if (m_sm_liste[i]->isActive() && m_sm_liste[i]!=mission) { m_sm_liste[i]->stop(); }
            if (!m_sm_liste[i]->isActive() && m_sm_liste[i]==mission){ m_sm_liste[i]->start(); }
        }
    }
}

// ________________________________________________
void IABase::stepAllStateMachines()
{
    for (int i=0; i<m_state_machine_count; i++) {
        if (m_sm_liste[i] && m_sm_liste[i]->isActive()){
            m_sm_liste[i]->step();
        }
    }
}

// ________________________________________________
unsigned short IABase::getScore()
{
    unsigned short score = 0;
    for (int i=0; i<m_state_machine_count; i++) {
        if (m_sm_liste[i]) {
            score+= m_sm_liste[i]->getScore();
        }
    }
    return score;
}

// ________________________________________________
bool IABase::isMatchEnCours()
{
    return m_sm_main.isMatchEnCours();
}

//___________________________________________________________________________
bool IABase::isMatchTermine()
{
    return m_sm_main.isMatchTermine();
}


//___________________________________________________________________________
void IABase::resetAllSMPriority(unsigned char priority)
{
    for (int i=0; i<m_state_machine_count; i++) {
        if (m_sm_liste[i]){
            m_sm_liste[i]->setPrioriteExecution(priority);
        }
    }
}
