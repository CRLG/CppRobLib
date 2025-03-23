/*! \file CServomoteursBase.cpp
    \brief Classe qui contient les méthodes pour le dialogue avec ANACONBOT
*/
//#include "RessourcesHardware.h"
#include "CServomoteursBase.h"

// Il reste un bug :
//      1. Consigne position 1500
//      2. Consigne position=1000 / vitesse = 5000
//      => La consigne repart en arrière à partir de 4294963796

// Lorsque le servo a été relâché
// La prochaine consigne de position n'est pas envoyée si c'est la même que la dernière
// Ex:
//      1. Relaché activé avec vitesse < VMAX
//      2. Consigne de 1300
//      3. Attente relâché
//      4. Consigne de 1300
//___________________________________________________________________________
/*!
   \brief Constructeur

   \param --
   \return --
*/
CServomoteursBase::CServomoteursBase()
{
    unsigned char i=0;

    // Initialisation des données
    for (i=0; i<NBRE_SERVOS; i++) {
        m_current_position[i] = 1500;  // valeur neutre 1500 plutôt pour repartir du centre la première fois ??
        m_target_position[i] = 0;
        m_vitesse[i] = VMAX;
        m_timer_relache[i] = TIMER_RELACHE_SERVO_OFF;
        m_duree_relache[i] = RELACHE_SERVO_OFF;
        m_pos_butee_min[i] = 0;         // Pas de saturation basse par défaut
        m_pos_butee_max[i] = 0xFFFF;    // Pasde saturation haute par défaut
    }
}



//___________________________________________________________________________
/*!
   \brief Destructeur

   \param --
   \return --
*/
CServomoteursBase::~CServomoteursBase()
{

}

//___________________________________________________________________________
/*!
   \brief Initialisation des servos pour la configuration matérielle courante
   \return --
*/
void CServomoteursBase::Init(void)
{
    // Si besoin dans l'application (comportement par défaut sans action)
}



//___________________________________________________________________________
/*!
   \brief Commande d'un servo moteur

   \param numServo le servomoteur a piloter  (1 à 20)
   \param pos position du servomoteur
   \param vitesse vitesse a laquelle se deplace le servo

   \return --
*/
void CServomoteursBase::CommandePositionVitesse(unsigned char numServo, unsigned short position, unsigned short vitesse)
{
    CommandeVitesse(numServo, vitesse);
    CommandePosition(numServo, position);
}

//___________________________________________________________________________
/*!
   \brief Commande d'un servo moteur

   \param numServo le servomoteur a piloter
   \param pos position du servomoteur
   \remarks valeur particulière "0" pour la position pour relâcher la commande PWM du servo

   \return --
*/
void CServomoteursBase::CommandePosition(unsigned char numServo, unsigned short position)
{
    if (numServo>NBRE_SERVOS) { return; }
    if (numServo == 0) { return; }

    unsigned char idx_servo = numServo - 1;  // -1 car numServo varie entre 1 et 20 (et pas entre 0 et 19)

    // cas particulier d'une demande de relâché (= servo libre):
    //    1. On force le PWM à "0"
    //    2. m_current_position conserve sa dernière valeur pour savoir de quelle valeur repartir pour la prochaine consigne qui sera demandée
    if (position == 0) {
        CommandeServo(numServo, 0);
    }
    else {
        // gestion des butées min et max
        position = saturePositionButees(idx_servo, position);
        if ( (m_vitesse[idx_servo] == VMAX) ||                  // Si la vitesse est au max, on applique directement la consigne souhaitée
             (m_current_position[idx_servo] == position) ) {    // Cas où on redemande la même position (sortie de relâché avec la même commande que la dernière avant le relâché)
            CommandeServo(numServo, position);
            m_current_position[idx_servo] = position;
        }
    }
    m_target_position[idx_servo] = position;

    if (m_duree_relache[idx_servo] != RELACHE_SERVO_OFF) {
        m_timer_relache[idx_servo] = 0;  // Relance la tempo avant relâché de la commande
    }
}

//___________________________________________________________________________
/*!
   \brief Commande de vitesse

   \param numServo le servomoteur a piloter  (1 à n)
   \param vitesse vitesse a laquelle se deplace le servo

   \return --
*/
void CServomoteursBase::CommandeVitesse(unsigned char numServo, unsigned short vitesse)
{
    if (numServo>NBRE_SERVOS) { return; }
    if (numServo == 0) { return; }
    m_vitesse[numServo - 1] = vitesse;
}


//___________________________________________________________________________
/*!
 * \brief Gestion des déplacements et du relâché
 */
void CServomoteursBase::periodicCall(void)
{
    unsigned char i=0;

    for (i=0; i<NBRE_SERVOS; i++) {
        // Rampe la position jusqu'à la consigne cible
        if (m_target_position[i] == 0) continue; // le servo est relâché -> rien à faire sur ce servo

        // ---------------------------------------------
        // La position du servo n'est pas encore à la consigne, il faut ramper vers la position target
        // Mais attention, plusieurs cas particuliers à traiter :
        //     (1) Le calcul de la prochaine position théorique dépasse la position target          => on sature à la position target
        //     (2) Le calcul de la prochaine position théorique provoque un débordement de calcul   => on sature à la position target
        if (m_current_position[i] != m_target_position[i]) {
            if (m_current_position[i] < m_target_position[i]) {
                unsigned short next_pos = m_current_position[i] + m_vitesse[i];
                if ( (next_pos > m_target_position[i]) ||       // (1) cas de la somme qui dépasserait la target -> on sature à la valeur target
                     (next_pos < m_current_position[i]) ) {     // (2) cas du débordement de calcul (la somme a debordé et fait rebouclé le compteur 16 bits) : on détecte que la somme a debordé si le résultat du calcul de la somme donne une valeur plus petite que la valeur de départ
                    m_current_position[i] = m_target_position[i];
                }
                else {
                    m_current_position[i] = next_pos;
                }
            } // if la position courante est inférieure à la target
            else {
                unsigned short next_pos = m_current_position[i] - m_vitesse[i];
                if ( (next_pos < m_target_position[i]) ||       // (1)
                     (next_pos > m_current_position[i]) ) {     // (2)
                    m_current_position[i] = m_target_position[i];
                }
                else {
                    m_current_position[i] = next_pos;
                }
            } // if la position courante est supérieure à la target
            CommandeServo(i+1, m_current_position[i]);
            if (m_duree_relache[i] != RELACHE_SERVO_OFF) m_timer_relache[i] = 0;  // Relance la tempo avant relâché de la commande pour qu'elle ne commence qu'une fois le servo arrivé à sa position finale
        } // if la position courante n'a pas encore atteint la target

        // ---------------------------------------------
        // Gestion de l'arrêt automatique des servos au bout de quelques instants pour éviter les vibrations
        // Faut il stopper automatiquement la commande appliquée à un servo ?
        if (m_timer_relache[i] < TIMER_RELACHE_SERVO_OFF) {
            m_timer_relache[i]++;
            if (m_timer_relache[i] > m_duree_relache[i]) {
                m_timer_relache[i] = TIMER_RELACHE_SERVO_OFF; // Pour ne plus incrémenter le timer et arrêter le traitement
                m_target_position[i] = 0; // La valeur "0" permet de relâcher la commande du servo (il reste à sa place, mais ne bouge plus)
                // m_current_position[i] conserve sa dernière valeur pour savoir d'où repartir à la prochaine consigne
                CommandeServo(i+1, 0);
            } // if il est temps de relâcher le servo
        } // if le timer de relâchement est à gérer
    } // for tous les servos
}



//___________________________________________________________________________
/*!
   \brief Configure la duree avant de relâcher la commande du servo
   \description la relâche de la commande permet de preserver la mécanique
                en arrêtant la consigne de position au servo.
                Dans certains cas d'utilisation, il ne faut pas relâcher la commande
   \param numServo le servomoteur a piloter
   \param duree la duree avant relach [msec] (ou RELACHE_SERVO_OFF pour que le servo ne
                soit jamais relâché automatiquement mais uniquement sur demande de l'applicatif)
   \return --
*/
void CServomoteursBase::setDureeAvantRelache(unsigned char numServo, unsigned int duree_ms)
{
    if (numServo>NBRE_SERVOS) { return; }
    if (numServo == 0) { return; }

    numServo = numServo - 1;  // -1 car numServo varie entre 1 et NBRE_SERVOS

    if (duree_ms != RELACHE_SERVO_OFF) {
        m_duree_relache[numServo] = (duree_ms / PERIODE_APPEL_TACHE_GESTION);  // Convertit les [msec] en nombre de passage dans la fonction principale de gestion des servos
        m_timer_relache[numServo] = 0; // active le timer
    }
    else {
        m_duree_relache[numServo] = RELACHE_SERVO_OFF;
        m_timer_relache[numServo] = TIMER_RELACHE_SERVO_OFF; // bloque le timer
    }
}

//___________________________________________________________________________
/*!
   \brief Configure les butées min et max a ne pas dépasser
   \description gère les butées pour préserver le matériel
   \param numServo le servomoteur a piloter
   \param butee_min la butée min à ne pas dépasser (ou BUTEE_SERVO_OFF pour le pas gérer la butée)
   \param butee_max la butée max à ne pas dépasser (ou BUTEE_SERVO_OFF pour le pas gérer la butée)
   \return --
*/
void CServomoteursBase::setButeesMinMaxPosition(unsigned char numServo, unsigned short butee_min, unsigned short butee_max)
{
    setButeeMinPosition(numServo, butee_min);
    setButeeMaxPosition(numServo, butee_max);
}

//___________________________________________________________________________
/*!
   \brief Configure les butées min a ne pas dépasser
   \description gère les butées pour préserver le matériel
   \param numServo le servomoteur a piloter  (1 à 20)
   \param butee_min la butée min à ne pas dépasser (ou BUTEE_SERVO_OFF pour le pas gérer la butée)
   \return --
*/
void CServomoteursBase::setButeeMinPosition(unsigned char numServo, unsigned short butee_min)
{
    if (numServo>NBRE_SERVOS) { return; }
    if (numServo == 0) { return; }

    numServo = numServo - 1;  // -1 car numServo varie entre 1 et NBRE_SERVOS
    m_pos_butee_min[numServo] = butee_min;
}
//___________________________________________________________________________
/*!
   \brief Configure les butées min et max a ne pas dépasser
   \description gère les butées pour préserver le matériel
   \param numServo le servomoteur a piloter  (1 à 20)
   \param butee_min la butée max à ne pas dépasser (ou BUTEE_SERVO_OFF pour le pas gérer la butée)
   \return --
*/
void CServomoteursBase::setButeeMaxPosition(unsigned char numServo, unsigned short butee_max)
{
    if (numServo>NBRE_SERVOS) { return; }
    if (numServo == 0) { return; }

    numServo = numServo - 1;  // -1 car numServo varie entre 1 et NBRE_SERVOS
    m_pos_butee_max[numServo] = butee_max;
}

//___________________________________________________________________________
/*!
 * \brief Libère la consigne du servomoteur
 * \param numServo le numéro de servo ou 0xFF pour libérer tous les servos
 */
void CServomoteursBase::relacheServo(unsigned char numServo)
{
    if (numServo == 0xFF) {
        for (int i=1; i<=NBRE_SERVOS; i++) {
            CommandePosition(i, 0);
        }
    }
    else {
        CommandePosition(numServo, 0);
    }
}


//___________________________________________________________________________
/*!
   \brief Configure la duree min et max des impulsions du signal PMW.
   \param idx_servo index du servo (0, 1, ...)
   \param duree_min_us duree minimum de l'impulsion en [µsec]
   \param duree_max_us duree maximum de l'impulsion en [µsec]
   \return --
*/
unsigned short CServomoteursBase::saturePositionButees(unsigned char idx_servo, unsigned short position)
{
    if (position == 0) return 0; // valeur spéciale pour le relâché de la commande servo -> on laisse passer la valeur
    if (idx_servo>=NBRE_SERVOS) { return 0; }

    unsigned short ret_pos;
    if (position < m_pos_butee_min[idx_servo]) {
        ret_pos = m_pos_butee_min[idx_servo];
    }
    else if (position > m_pos_butee_max[idx_servo]) {
        ret_pos = m_pos_butee_max[idx_servo];
    }
    else {
        ret_pos = position;
    }
    return(ret_pos);
}


