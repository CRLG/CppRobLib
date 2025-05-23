/*! \file CLeds.h
    \brief Classe de base qui contient la gestion des 4 LED CPU
*/

#ifndef _LEDS_H_
#define _LEDS_H_

#include "CLedBase.h"

// Dans ce module, toutes les durées sont en [msec]

// Comment l'utiliser :
// --------------------
// Par héritage de CLedBase, créer une classe CLed en lien avec le hardware et ré-implémenter les méthode spécifiques au hardware.
// Instancer 4 fois la classe CLed dans l'applicatif.
// Instancier la classe CLeds de gestion des 4 leds en passant un pointeur sur ces 4 instances à son constructeur
// Instancier la classe CLeds et appeler périodiquement la méthode compute()
//      CLed led1, led2, led3, led4;
//      CLeds leds(&led1, &led2, &led3, &led4);
//      ...
//      (toutes les 50msec)
//      leds.compute();

// Exemple d'utilisation de l'API du module :
// ------------------------------------------
// Possibilité de s'adresser à une LED en particulier :
//      leds.m_led1.toggle();
//  ou
//      leds.toggle(LED_1);
// Possibilité de s'adresser à toutes les leds d'une seul coup :
//      leds.toggle(ALL_LED);
// Possibilité jouer une impulsion sur une LED
//      leds.m_led1.setPulse(50, 100, 10);
//          -> Joue 10 impulsions de 50msec à l'état ON puis 100msec à l'état OFF
// Possibilité jouer un motif sur toutes les LED
//      leds.setPattern(PATTERN_K2000); // Joue le pattern avec des valeurs par défaut
//  ou
//      leds.setPattern(PATTERN_K2000, 200); // Joue le pattern en spéciant la vitesse de défilement (nombre de msec entre 2 échantillons du pattern)
//  ou
//      leds.setPattern(PATTERN_K2000, 200, 5); // Idem précédent, mais définit le nombre de cycles du pattern à jouer avant de s'arrêter

// Attention : la fonction setPattern ne doit être appelée qu'une seule fois au moment où le début du pattern est souhaité (et pas appelé périodiquement car cela recommencerait du début le pattern à chaque appel)


// ============================================================
//        Gestion des LEDs
// ============================================================
typedef enum {
    LED_1 = 1,
    LED_2,
    LED_3,
    LED_4,
    ALL_LED = 0xFFFF
}tLed;

// Attention : cet enum doit être dans le même ordre que m_patterns
typedef enum {
    PATTERN_CLIGNO_1234 = 0,
    PATTERN_CLIGNO_12,
    PATTERN_CLIGNO_13,
    PATTERN_CLIGNO_14,
    PATTERN_CLIGNO_24,
    PATTERN_CLIGNO_34,
    PATTERN_CLIGNO_12_34,
    PATTERN_CLIGNO_13_24,
    PATTERN_CLIGNO_14_23,
    PATTERN_K2000,
    PATTERN_CHENILLE,
    // __________________ à laisser en dernier
    PATTERNS_MAX_SIZE
}tLedPattern;

//! Classe de gestion des LED
class CLeds
{
public :
    CLeds(CLedBase *led1, CLedBase *led2, CLedBase *led3, CLedBase *led4);
    ~CLeds();

    void compute();
    // API
    void setState(tLed led, bool state);
    void toggle(tLed led);
    void setOn(tLed led);
    void setOff(tLed led);
    void setPattern(tLedPattern pattern, unsigned short sample_duration_ms=100, unsigned short num_cycle=INFINITE);

    CLedBase *m_led1;
    CLedBase *m_led2;
    CLedBase *m_led3;
    CLedBase *m_led4;

private :
    // Les patterns
    // Chaque valeur du tableau correspond à un champ de bits pour les 4 LED
    //      0x1 allume la LED n°1 uniquement
    //      0x3 allume les LED n°1 et n°2
    // Un pattern doit toujours se terminer par 0xFF
    const unsigned char m_pattern_cligno1234[3] =   { 0xF, 0x0, 0xFF};
    const unsigned char m_pattern_cligno12[3] =     { 0x3, 0x0, 0xFF};
    const unsigned char m_pattern_cligno13[3] =     { 0x5, 0x0, 0xFF};
    const unsigned char m_pattern_cligno14[3] =     { 0x9, 0x0, 0xFF};
    const unsigned char m_pattern_cligno24[3] =     { 0xA, 0x0, 0xFF};
    const unsigned char m_pattern_cligno34[3] =     { 0xC, 0x0, 0xFF};
    const unsigned char m_pattern_cligno12_34[3] =  { 0x3, 0xC, 0xFF};
    const unsigned char m_pattern_cligno13_24[3] =  { 0x5, 0xA, 0xFF};
    const unsigned char m_pattern_cligno14_23[3] =  { 0x9, 0x6, 0xFF};
    const unsigned char m_pattern_k2000[9] =        { 0x0, 0x1, 0x3, 0x7, 0xF, 0x7, 0x3, 0x1, 0xFF};
    const unsigned char m_pattern_chenille[5] =     { 0x1, 0x2, 0x4, 0x8, 0xFF};

    // Attention : ce tableau doit être dans le même ordre que l'enum tLedPattern
    const unsigned char *m_patterns[PATTERNS_MAX_SIZE] =
    {       m_pattern_cligno1234,
            m_pattern_cligno12,
            m_pattern_cligno13,
            m_pattern_cligno14,
            m_pattern_cligno24,
            m_pattern_cligno34,
            m_pattern_cligno12_34,
            m_pattern_cligno13_24,
            m_pattern_cligno14_23,
            m_pattern_k2000,
            m_pattern_chenille
    };
};
#endif


