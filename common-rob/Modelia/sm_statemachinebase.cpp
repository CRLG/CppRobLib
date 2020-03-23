#include "sm_statemachinebase.h"
#include "ia.h"

SM_StateMachineBase::SM_StateMachineBase()
    : m_ia(nullptr),
      m_main_mission_type(false),
      m_enabled(true),
      m_state(SM_NOT_INITIALIZED),
      m_next_state(SM_NOT_INITIALIZED),
      m_old_state(SM_NOT_INITIALIZED),
      m_active(false),
      m_succes(false),
      m_timeout(0),
      m_score(0),
      m_max_score(0),
      m_priorite_execution(255)
{
}

SM_StateMachineBase::~SM_StateMachineBase()
{

}

// ________________________________________________
const char* SM_StateMachineBase::getName()
{
    return "";
}

// ________________________________________________
const char* SM_StateMachineBase::stateToName(unsigned short state)
{
    (void)state;
    return "";
}


// ________________________________________________
void SM_StateMachineBase::init(IA *ia)
{
    m_ia = ia;
    m_old_state = SM_NOT_INITIALIZED;
    m_state = SM_FIRST_STATE;
    m_next_state = SM_FIRST_STATE;
    m_active = false;
    m_succes = false;
    m_timeout = 0;
    m_priorite_execution = 255;
    m_score = 0;
}

// ________________________________________________
void SM_StateMachineBase::start()
{
    if (m_active) return;  // Si la SM est déjà en fonctionnement, rien à faire
    m_timeout = 0;
    m_active = true;
    if (m_ia->m_sm_debug) {
        m_ia->m_sm_debug->SM_Start(this, internals()->TempsMatch);
    }
}
// ________________________________________________
void SM_StateMachineBase::stop()
{
    m_timeout = 0;
    if (m_active && m_ia->m_sm_debug) {
        m_ia->m_sm_debug->SM_Stop(this, internals()->TempsMatch);
    }
    m_active = false;
}

// ________________________________________________
void SM_StateMachineBase::interruptForEvitement()
{
    m_old_state = SM_INTERRUPTED;  // pour qu'à la réactivation de la SM, on repasse dans le "onEntry"
    m_timeout = 0;
    m_active = false;
    if (m_ia->m_sm_debug) {
        m_ia->m_sm_debug->SM_InterruptedByEvitement(this, m_state, internals()->TempsMatch);
    }
}

// ________________________________________________
int SM_StateMachineBase::getCurrentState()
{
    return m_state;
}

// ________________________________________________
unsigned short SM_StateMachineBase::getScore()
{
    return m_score;
}
// ________________________________________________
unsigned short SM_StateMachineBase::getScoreMax()
{
    return m_max_score;
}
// ________________________________________________
unsigned char SM_StateMachineBase::getPrioriteExecution()
{
    return m_priorite_execution;
}
// ________________________________________________
void SM_StateMachineBase::setPrioriteExecution(unsigned char priorite)
{
    m_priorite_execution = priorite;
}

// ________________________________________________
bool SM_StateMachineBase::isActive()
{
    return m_active;
}

// ________________________________________________
void SM_StateMachineBase::gotoState(unsigned short next_state)
{
    if (m_state != m_next_state) return; // prend en compte le cas d'une transition qui a déjà demandé le changement d'état. Dans ce cas, lui laisse la priorité et ignore cet autre demande de changement d'état. Du coup, celle qui est écrite en première est la plus prioritaire.
    m_old_state = m_state;
    m_next_state = next_state;
    m_timeout = 0;
}
// ________________________________________________
void SM_StateMachineBase::gotoStateAfter(unsigned short next_state, long timeout)
{
    if (timeout == NO_TIMEOUT) return;  // pas de gestion de timeout dans ce cas.
    m_timeout += (inputs()->TE_Modele * 1000);
    if (m_timeout>timeout) {
        gotoState(next_state);
    }
}
// ________________________________________________
void SM_StateMachineBase::gotoStateIfTrue(unsigned short next_state, bool condition, long timeout)
{
    if (condition) {
        gotoState(next_state);
    }
    gotoStateAfter(next_state, timeout);
}
// ________________________________________________
void SM_StateMachineBase::gotoStateIfConvergence(unsigned short next_state, long timeout)
{
    gotoStateIfTrue(next_state, m_ia->m_inputs_interface.FrontM_Convergence, timeout);
}

// ________________________________________________
void SM_StateMachineBase::gotoStateIfConvergenceRapide(unsigned short next_state, long timeout)
{
    gotoStateIfTrue(next_state, m_ia->m_inputs_interface.FrontM_Convergence_rapide, timeout);
}

// ________________________________________________
void SM_StateMachineBase::gotoStateIfConvergenceRack(unsigned short next_state, long timeout)
{
    gotoStateIfTrue(next_state, m_ia->m_inputs_interface.FrontM_ConvergenceRack, timeout);
}

// ________________________________________________
bool SM_StateMachineBase::onEntry()
{
    bool result = (m_state != m_old_state);
    m_old_state = m_state;
    if (result && m_ia->m_sm_debug) {
        m_ia->m_sm_debug->SM_EntryState(this, m_state, internals()->TempsMatch);
    }
    return result;
}
// ________________________________________________
bool SM_StateMachineBase::onExit()
{
    // Fin de l'état si changement d'état demandé ou si la machine est arrêtée
    bool result = (m_state != m_next_state) || (!isActive());
    if (result && m_ia->m_sm_debug) {
        m_ia->m_sm_debug->SM_ExitState(this, m_state, internals()->TempsMatch);
    }
    m_state = m_next_state;
    return result;
}

// ________________________________________________
SM_DatasInterface* SM_StateMachineBase::internals()
{
    return &m_ia->m_datas_interface;
}

// ________________________________________________
SM_OutputsInterface* SM_StateMachineBase::outputs()
{
    return &m_ia->m_outputs_interface;
}

// ________________________________________________
SM_InputsInterface* SM_StateMachineBase::inputs()
{
    return &m_ia->m_inputs_interface;
}

// ________________________________________________
bool SM_StateMachineBase::isSucces()
{
    return m_succes;
}

// ________________________________________________
bool SM_StateMachineBase::isMainMission()
{
    return m_main_mission_type;
}

// ________________________________________________
bool SM_StateMachineBase::isEnabled()
{
    return m_enabled;
}

// ________________________________________________
void SM_StateMachineBase::setEnabled(bool state)
{
    m_enabled = state;
}

