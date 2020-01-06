/*! \file CLedBase.h
    \brief Classe de base pour la gestion d'une LED
*/

#ifndef _LED_BASE_H_
#define _LED_BASE_H_

#define LED_REFRESH_PERIOD 50 // msec : période d'appel de la fonction compute depuis le séquenceur

// ============================================================
//        Gestion d'une LED
// ============================================================
//! Classe de gestion d'une LED
//! Virtuelle pure (méthodes en lien avec le hardware à ré-impléménter sur la cible)
class CLedBase
{
#define INFINITE (unsigned short)0xFFFFFFFF
public :
    CLedBase();
    virtual ~CLedBase();

    void compute();
    // API
    void setState(bool state);
    void toggle();
    bool read();
    void setPulse(unsigned short on_duration=500, unsigned short off_duration=500, unsigned short num_cycle=INFINITE);
    void setPattern(const unsigned char *pattern, unsigned char pattern_mask=0x1, unsigned short sample_duration=100, unsigned short num_cycle=INFINITE);

protected :
    typedef enum {
        LEDMODE_MANUAL = 0,
        LEDMODE_PULSE,
        LEDMODE_PATTERN
    }tLedMode;

    tLedMode m_mode;
    unsigned short m_on_duration;
    unsigned short m_off_duration;
    unsigned short m_num_cycle;
    unsigned char m_pattern_mask;
    const unsigned char *m_pattern;

    unsigned long m_count;
    unsigned long m_time;
    unsigned char m_index_array;
protected :
    // Méthodes vrituelles pures à ré-implémenter pour faire le lien avec le hardware
    virtual void _write(bool val) = 0;
    virtual bool _read() = 0;
};

#endif


