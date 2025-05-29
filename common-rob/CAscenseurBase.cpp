#include "CAscenseurBase.h"
#include "CGlobale.h"
#include <stdio.h>

CAscenseurBase::CAscenseurBase()
    : m_target(TARGET_STOP),
      m_speed_up(0),
      m_speed_down(0)
{
}
CAscenseurBase::~CAscenseurBase()
{
}

// _________________________________________
void CAscenseurBase::up()
{
    set_position(TARGET_UP);
}

// _________________________________________
void CAscenseurBase::down()
{
    set_position(TARGET_DOWN);
}

// _________________________________________
void CAscenseurBase::set_position(unsigned char target)
{
    m_target = target;
}

// _________________________________________
void CAscenseurBase::set_speeds(int speed_up, int speed_down)
{
    m_speed_up = speed_up;
    m_speed_down = speed_down;
}


// _________________________________________
void CAscenseurBase::stop()
{
    m_target = TARGET_STOP;
    command_motor(0);
}

// _________________________________________
unsigned int CAscenseurBase::get_position()
{
    if (is_sensor_low() && !is_sensor_high()) return POSITION_LOW;
    else if (is_sensor_high() && !is_sensor_low()) return POSITION_HIGH;
    else return POSITION_UNKNOWN;
}

// _________________________________________
// Fonctionnement de l'ascenseur :
//   -> On donne une consigne cible haut, bas ou arrêt (target)
// L'ascenseur maintien la consigne, même si par gravité l'ascenseur redescend et quitte la butée haute,
//  le moteur sera de nouveau piloté (pour rester en haut)
void CAscenseurBase::periodicCall()
{
    if ((is_sensor_low()) && (m_target==TARGET_DOWN))       command_motor(0);
    else if ((is_sensor_high()) && (m_target==TARGET_UP))   command_motor(0);
    else if (m_target==TARGET_DOWN)                         command_motor(m_speed_down);
    else if (m_target==TARGET_UP)                           command_motor(m_speed_up);
    else                                                    command_motor(0);
}
