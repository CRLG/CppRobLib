/*! \file CLedBase.cpp
    \brief Classe de base pour la gestion d'une LED
*/
#include "CLedBase.h"


// ============================================================
//        Gestion d'une LED
// ============================================================
CLedBase::CLedBase()
{
    m_mode = LEDMODE_MANUAL;
}

CLedBase::~CLedBase()
{
}


//___________________________________________________________________________
 /*!
   \brief Fixe l'état d'une LED.
   \return --
   \remark en utilisant cette méthode, la LED passe en mode manuel
*/
void CLedBase::setState(bool state)
{
    _write(state);
    m_mode = LEDMODE_MANUAL;
}

//___________________________________________________________________________
 /*!
   \brief Change l'état de la LED
   \return --
*/
void CLedBase::toggle()
{
    setState( !_read() );
}

//___________________________________________________________________________
 /*!
   \brief Lit l'état de la LED
   \return l'état On/Off de la LED
*/
bool CLedBase::read()
{
    return _read();
}

//___________________________________________________________________________
 /*!
   \brief Passe la LED en mode impulsion
   \param on_duration : la durée à l'état ON de la LED [msec]
   \param off_duration : la durée à l'état OFF de la LED [msec]
   \param num_cycle : le nombre d'impulsions à jouer (INFINIT = indéfiniement)
   \return --
*/
void CLedBase::setPulse(unsigned short on_duration, unsigned short off_duration, unsigned short num_cycle)
{
    m_on_duration = on_duration;
    m_off_duration = off_duration;
    m_num_cycle = num_cycle;

    m_time = 0;
    m_count = 0;

    m_mode = LEDMODE_PULSE;
}

//___________________________________________________________________________
 /*!
   \brief Passe la LED en mode pattern
   \param pattern : le tableau de valeur à jouer
   \param pattern_mask : le masque à appliquer à la valeur pour définir l'état de la LED (car la même valeur peut représenter l'état de plusieurs LED)
   \param sample_duration : la durée entre 2 échantillons du pattern
   \param num_cycle : le nombre d'impulsions à jouer (INFINIT = indéfiniement)
   \remark dans ce mode, un tableau est joué
   \remark cette fonction ne fait que mémoriser les paramètres du pattern. Il n'est réellement joué que dans la fonction compute
   \return --
*/
void CLedBase::setPattern(const unsigned char *pattern, unsigned char pattern_mask, unsigned short sample_duration, unsigned short num_cycle)
{
    // Attention : le tableau de valeur n'est pas recopié. On suppose qu'il n'est pas détruit
    // durant le déroulement du pattern
    m_pattern = pattern;
    m_on_duration = sample_duration;   // on utilise m_on_duration pour définir la durée d'un échantillon
    m_pattern_mask = pattern_mask;
    m_num_cycle = num_cycle;

    m_time = 0;
    m_count = 0;
    m_index_array = 0;

    m_mode = LEDMODE_PATTERN;
}

//___________________________________________________________________________
 /*!
   \brief Gestion périodique de la LEDm_led1
   \return --
   \remark pour gérer l'état de la LED dans les modes PULSE et PATTERN, il est
            nécessaire d'appeler périodiquement cette fonction
*/
void CLedBase::compute()
{
    switch(m_mode)
    {
        // ________________________________________________
        case LEDMODE_PULSE :
            //   |-------------|__________________________|-------------|___________
            //   < on_duration ><       off_duration      >
            if (m_time < m_on_duration) _write(1);
            else _write(0);
            if (m_time >= (m_on_duration + m_off_duration)) {
                if (m_num_cycle != INFINITE) m_count++;
                if (m_count >= m_num_cycle) {
                    setState(0);  // repasse en mode manuel LED éteinte
                }
                else {  // Recommence un cycle
                    m_time = 0;
                }
            }
            else {
                m_time += LED_REFRESH_PERIOD;
            }
        break;
        // ________________________________________________ Joue le pattern passé en paramètre
        case LEDMODE_PATTERN:
            if (m_time == 0) {
                _write(m_pattern[m_index_array] & m_pattern_mask);  // la LED prend la valeur du tableau à l'index courant
            }
            m_time += LED_REFRESH_PERIOD;
            if (m_time > m_on_duration) {  // il est l'heure de changer d'index
                m_time = 0;
                if (m_pattern[m_index_array+1] != 0xFF) m_index_array++;
                else {
                    m_index_array = 0;
                    if (m_num_cycle != INFINITE) m_count++;  // Refait le cycle ou termine
                    if (m_count >= m_num_cycle) {
                        setState(0);  // repasse en mode manuel LED éteinte
                    }
                }
            }
        break;
        // ________________________________________________
        default :
            // ne rien faire
        break;
    }
}
