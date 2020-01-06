/*! \file CLeds.cpp
	\brief Classe qui contient toute l'application
*/
#include "CLeds.h"

//___________________________________________________________________________
 /*!
   \brief Constructeur

   \param --
   \return --
*/
CLeds::CLeds(CLedBase *led1, CLedBase *led2, CLedBase *led3, CLedBase *led4)
  : m_led1(led1),
    m_led2(led2),
    m_led3(led3),
    m_led4(led4)
{
}

//___________________________________________________________________________
 /*!
   \brief Destructeur

   \param --
   \return --
*/
CLeds::~CLeds()
{

}

//___________________________________________________________________________
 /*!
   \brief Allume ou éteint une ou des LED

   \param led numéro de la LED (ALL_LED pour affecter toutes les LEDs)
   \param state souhaité état de la LED
   \return --
*/
void CLeds::setState(tLed led, bool state)
{
    switch(led) {
        case LED_1 : m_led1->setState(state); break;
        case LED_2 : m_led2->setState(state); break;
        case LED_3 : m_led3->setState(state); break;
        case LED_4 : m_led4->setState(state); break;

        case ALL_LED :
            m_led1->setState(state);
            m_led2->setState(state);
            m_led3->setState(state);
            m_led4->setState(state);
        break;
    }
}

//___________________________________________________________________________
 /*!
   \brief Change l'état d'une ou de toutes les LED

   \param led numéro de la LED (ALL_LED pour affecter toutes les LEDs)
   \return --
*/
void CLeds::toggle(tLed led)
{
    switch(led) {
        case LED_1 : m_led1->toggle(); break;
        case LED_2 : m_led2->toggle(); break;
        case LED_3 : m_led3->toggle(); break;
        case LED_4 : m_led4->toggle(); break;

        case ALL_LED :
            m_led1->toggle();
            m_led2->toggle();
            m_led3->toggle();
            m_led4->toggle();
        break;
    }
}

//___________________________________________________________________________
 /*!
   \brief Traitement périodiques nécessaires à la gestion des LED
   \return --
*/
void CLeds::compute()
{
    m_led1->compute();
    m_led2->compute();
    m_led3->compute();
    m_led4->compute();
}

//___________________________________________________________________________
 /*!
   \brief Initie un pattern sur toutes les LED

   \param pattern_id : le numéro de pattern à jouer parmi ceux prédéfinis
   \param sample_duration : la durée entre 2 échantillons du pattern
   \param num_cylce : le nombre de fois où le pattern doit être répété avant de s'arrêter (INIFINITE) pour jouer le pattern indéfiniment
   \return --
*/
void CLeds::setPattern(tLedPattern pattern_id, unsigned short sample_duration_ms, unsigned short num_cycle)
{
    if (pattern_id >= PATTERNS_MAX_SIZE) return;

    m_led1->setPattern(m_patterns[pattern_id], 0x1, sample_duration_ms, num_cycle);
    m_led2->setPattern(m_patterns[pattern_id], 0x2, sample_duration_ms, num_cycle);
    m_led3->setPattern(m_patterns[pattern_id], 0x4, sample_duration_ms, num_cycle);
    m_led4->setPattern(m_patterns[pattern_id], 0x8, sample_duration_ms, num_cycle);
}


