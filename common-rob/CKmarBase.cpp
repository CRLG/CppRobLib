#include "CKmarBase.h"

// ===========================================================

// ________________________________________________
void CKmarMouvement::start()
{
    m_old_state = -2;
    m_state = MOVEMENT_START;
    m_next_state = MOVEMENT_START;
    m_timeout = 0;
}

void CKmarMouvement::stop()
{
    gotoState(MOVEMENT_FINISHED);
}

bool CKmarMouvement::isFinished()
{
    return (m_state==MOVEMENT_FINISHED);
}

// ________________________________________________
void CKmarMouvement::gotoState(unsigned short next_state)
{
    if (m_state != m_next_state) return; // prend en compte le cas d'une transition qui a déjà demandé le changement d'état. Dans ce cas, lui laisse la priorité et ignore cet autre demande de changement d'état. Du coup, celle qui est écrite en première est la plus prioritaire.
    m_old_state = m_state;
    m_next_state = next_state;
    m_timeout = 0;
}
// ________________________________________________
void CKmarMouvement::gotoStateAfter(unsigned short next_state, long timeout)
{
    if (timeout == NO_TIMEOUT) return;  // pas de gestion de timeout dans ce cas.
    m_timeout += REFRESH_PERIOD;
    if (m_timeout>timeout) {
        gotoState(next_state);
    }
}
// ________________________________________________
void CKmarMouvement::gotoStateIfTrue(unsigned short next_state, bool condition, long timeout)
{
    if (condition) {
        gotoState(next_state);
    }
    gotoStateAfter(next_state, timeout);
}
// ________________________________________________
void CKmarMouvement::gotoStateIfConvergence(unsigned short next_state, long timeout)
{
    // TODO : àvoir si on lit l'état des axes
}

// ________________________________________________
void CKmarMouvement::gotoStateIfNear(unsigned short next_state, int axis, int target, unsigned char percent, long timeout)
{
    bool condition=false;

    int epsilon = ((100-percent)/100.) * target;
    int pos = m_kmar->getPosition(axis);
    if ( (pos <= (target+epsilon)) && (pos >= (target-epsilon)) ) condition= true;
    gotoStateIfTrue(next_state, condition, timeout);
}

// ________________________________________________
void CKmarMouvement::gotoNextState()
{
    gotoState(m_state+1);
}

// ________________________________________________
void CKmarMouvement::gotoFinish()
{
    gotoState(MOVEMENT_FINISHED);
}

// ________________________________________________
bool CKmarMouvement::onEntry()
{
    bool result = (m_state != m_old_state);
    m_old_state = m_state;
    return result;
}
// ________________________________________________
bool CKmarMouvement::onExit()
{
    // Fin de l'état si changement d'état demandé ou si la machine est arrêtée
    bool result = (m_state != m_next_state);
    m_state = m_next_state;
    return result;
}

// ===========================================================
CKmarBase::CKmarBase() :
    m_speed_factor(1.0),
    m_mouvement_en_cours(nullptr)
{
}

CKmarBase::~CKmarBase()
{
}

// __________________________________________________
//!
//! \brief CKmarBase::getPosition
//! \param pos_axis1
//! \param pos_axis2
//! \param pos_axis3
//! \param pos_axis4
//!
void CKmarBase::getPosition(int *pos_axis1, int *pos_axis2, int *pos_axis3, int *pos_axis4)
{

}

// __________________________________________________
bool CKmarBase::autotest()
{
    //! TODO
    return true;
}

// __________________________________________________
void CKmarBase::arm()
{
    for (unsigned int i=0; i<getAxisCount(); i++) {
        arm(i);
    }
}

// __________________________________________________
void CKmarBase::disarm()
{
    for (unsigned int i=0; i<getAxisCount(); i++) {
        disarm(i);
    }
}

// __________________________________________________
// Renvoie vrai si au moin un des axes est en mouvement
// Renvoie faux si tous les axes ont convergés
bool CKmarBase::isMoving()
{
    bool moving = false;
    for (unsigned int i=0; i<getAxisCount(); i++) {
        moving |= isMoving(i);
    }
    return moving;
}

// __________________________________________________
void CKmarBase::setSpeedFactor(float factor)
{
    if (factor < 0) return;
    if (factor > 20) return;

    m_speed_factor = factor;
}

void CKmarBase::stop()
{
    if (m_mouvement_en_cours) m_mouvement_en_cours->stop();
    m_mouvement_en_cours = nullptr;
    disarm();
}

// __________________________________________________
void CKmarBase::compute()
{
    if (m_mouvement_en_cours) {
        m_mouvement_en_cours->step();
        if (m_mouvement_en_cours->isFinished()) m_mouvement_en_cours = nullptr;
    }
}
