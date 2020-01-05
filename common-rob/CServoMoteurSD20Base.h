/*! \file CServoMoteurSD20Base.h
    \brief Classe qui contient la gestion des roues motrices gauche et droite
*/

#ifndef _SERVO_MOTEUR_SD20_BASE_H_
#define _SERVO_MOTEUR_SD20_BASE_H_


// -----------------------------
//! Classe de gestion des options d'exécution passees en ligne de commande
class CServoMoteurSD20Base {
#define ADRESSE_I2C_SD20 	0xC2
#define NBRE_SERVOS_SD20	20

#define RELACHE_SERVO_OFF           0xFFFF
#define BUTEE_SERVO_OFF             0xFFFF
#define PERIODE_APPEL_TACHE_GESTION 10 // [msec]

public :
    CServoMoteurSD20Base();
    virtual ~CServoMoteurSD20Base();

    virtual void Init(void);
    virtual void CommandePosition(unsigned char numServo, unsigned int position);
    virtual void CommandeVitesse(unsigned char numServo, unsigned int vitesse);
    virtual void CommandePositionVitesse(unsigned char numServo, unsigned int position, unsigned int vitesse);
    virtual void setDureeAvantRelache(unsigned char numServo, unsigned int duree_ms);
    virtual void setButeesMinMaxPosition(unsigned char numServo, unsigned int butee_min, unsigned int butee_max);
    virtual void setButeeMinPosition(unsigned char numServo, unsigned int butee_min);
    virtual void setButeeMaxPosition(unsigned char numServo, unsigned int butee_max);
    virtual unsigned int saturePositionButees(unsigned char numServo, unsigned int position, unsigned int butee_min, unsigned int butee_max);

private :
    //! FONCTIONS SPECIFIQUES
    //void readEEPROM(void);
    //void setDureesMinMaxPulseSD20(unsigned int duree_min_us, unsigned int duree_max_us);
    //void GestionTransfert(void) = 0;


protected :
    //! Memorise la position relle envoyee au servo
    unsigned int m_position[NBRE_SERVOS_SD20];
    //! Memorise la consigne de position demandée par l'applicatif
    unsigned int m_position_utilisateur[NBRE_SERVOS_SD20];
    //! Memorise la vitesse
    unsigned int m_vitesse[NBRE_SERVOS_SD20];
    //! Butée min sur la position
    unsigned int m_pos_butee_min[NBRE_SERVOS_SD20];
    //! Butée max sur la position
    unsigned int m_pos_butee_max[NBRE_SERVOS_SD20];
    //! Flag qui indique s'il faut renvoyer la valeur au SD20 pour ce servo
    bool m_update[NBRE_SERVOS_SD20];
    //! Timer de relâché de la commande du servo
    unsigned int m_timer_relache[NBRE_SERVOS_SD20];
    //! Duree avant relâchée de la commande du servo (unité = nombre de passage dans la boucle de gestion)
    unsigned int m_duree_relache[NBRE_SERVOS_SD20];
};

#endif  // _SERVO_MOTEUR_SD20_BASE_H_


