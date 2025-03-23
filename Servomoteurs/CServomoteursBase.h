/*! \file CServomoteursBase.h
    \brief Classe qui contient la gestion des roues motrices gauche et droite
*/

#ifndef _SERVOMOTEURS_BASE_H_
#define _SERVOMOTEURS_BASE_H_


// -----------------------------
//! Classe de gestion des options d'exécution passees en ligne de commande
class CServomoteursBase {
public :

    static const unsigned int RELACHE_SERVO_OFF = 0xFFFFFFFF;
    static const unsigned int TIMER_RELACHE_SERVO_OFF = 0xFFFFFFFF;
    static const unsigned short BUTEE_SERVO_OFF =0xFFFF;
    static const unsigned short PERIODE_APPEL_TACHE_GESTION = 20; // [msec]
    static const unsigned short NBRE_SERVOS = 7;
    static const unsigned short VMAX = 0xFFFF;

    typedef enum {
        SERVO_1 = 1,
        SERVO_2,
        SERVO_3,
        SERVO_4
    }tServoNum;

    CServomoteursBase();
    virtual ~CServomoteursBase();

    virtual void Init(void);
    virtual void CommandePosition(unsigned char numServo, unsigned short position);
    virtual void CommandeVitesse(unsigned char numServo, unsigned short vitesse);
    virtual void CommandePositionVitesse(unsigned char numServo, unsigned short position, unsigned short vitesse);
    virtual void setDureeAvantRelache(unsigned char numServo, unsigned int duree_ms);
    virtual void setButeesMinMaxPosition(unsigned char numServo, unsigned short butee_min, unsigned short butee_max);
    virtual void setButeeMinPosition(unsigned char numServo, unsigned short butee_min);
    virtual void setButeeMaxPosition(unsigned char numServo, unsigned short butee_max);
    virtual void relacheServo(unsigned char numServo);

    void periodicCall(void);

protected :
    virtual void CommandeServo(unsigned char numServo, unsigned short position_usec) = 0;
    virtual unsigned short saturePositionButees(unsigned char idx_servo, unsigned short position);

    //! Memorise la position relle envoyee au servo
    unsigned short m_current_position[NBRE_SERVOS];
    //! Memorise la consigne de position demandée par l'applicatif
    unsigned short m_target_position[NBRE_SERVOS];
    //! Memorise la vitesse
    unsigned short m_vitesse[NBRE_SERVOS];
    //! Butée min sur la position
    unsigned short m_pos_butee_min[NBRE_SERVOS];
    //! Butée max sur la position
    unsigned short m_pos_butee_max[NBRE_SERVOS];
    //! Timer de relâché de la commande du servo
    unsigned int m_timer_relache[NBRE_SERVOS];
    //! Duree avant relâchée de la commande du servo (unité = nombre de passage dans la boucle de gestion)
    unsigned int m_duree_relache[NBRE_SERVOS];
};

#endif  // _SERVOMOTEURS_BASE_H_


