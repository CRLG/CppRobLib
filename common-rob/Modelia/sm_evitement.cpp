#include "CGlobale.h"
#include "sm_evitement.h"
#include "ia.h"

SM_Evitement::SM_Evitement()
{
/* TODO : plantage
    internals()->evitementEnCours=false;
    internals()->inhibeObstacle=false;
    internals()->evit_nombre_tentatives=0;
    internals()->evit_nombre_max_tentatives=0;
    internals()->evit_detection_obstacle_bitfield=0;
    internals()->evit_strategie_evitement_en_cours=false;
    internals()->evit_sgn_dist_eloigne=1;
    internals()->evit_sgn_angle_pivote=1;
    internals()->evit_sgn_dist_evacue=1;
    internals()->evit_toggle_signe=1;
    internals()->evit_debug_etape=0;
    internals()->evit_choix_strategie=STRATEGIE_EVITEMENT_ATTENDRE;
    internals()->evit_seuil_detection_obstacle=0;
*/
}

const char* SM_Evitement::getName()
{
    return "SM_Evitement";
}

const char* SM_Evitement::stateToName(unsigned short state)
{
    switch(state)
    {
    //case ALLER_VERS_PHARE :         return "ALLER_VERS_PHARE";
    }
    return "UNKNOWN_STATE";
}


void SM_Evitement::step()
{
    internals()->evit_detection_obstacle_bitfield = 0; //Application.m_match.m_obstacle_detecte_bitfield;

    switch(m_state)
    {
    // ___________________________________
    case EVITEMENT_INIT :
        if (onEntry()) {
/*
            // Mémorisation du contexte (paramètres de l'asserv)
            evit_memo_cde_min = Application.m_asservissement.cde_min;
            evit_memo_cde_max = Application.m_asservissement.cde_max;
            evit_memo_idx_sportiv = Application.m_asservissement.Ind_perfo;
            evit_memo_vitesse_avance = Application.m_asservissement.vitesse_avance_max;
            evit_memo_vitesse_angle = Application.m_asservissement.vitesse_rotation_max;
            evit_sens_avant_detection = IN_sens_deplacement;
            evit_memo_force_obstacle = forceObstacle;
            forceObstacle = true;

            evit_debug_etape = 0;
            evit_nombre_tentatives = 0;
            evit_toggle_signe = evit_toggle_signe * -1;  // permet de changer certains sens lorsqu'on revient dans l'évitement pour donner un caractère aléatoire

            // Arrêt du robot : l'arrêt se fait en donnant une consigne de vitesse nulle en avance et en angle
            // cela permet un arrêt plus rapide qu'une consigne manuelle à "0"
            //Application.m_asservissement.CommandeManuelle(0,0)
            //Application.m_asservissement.CommandeVitesseMouvement(0, 0)
            Application.m_asservissement.CommandeMouvementXY_TETA(IN_x_pos as float, IN_y_pos as float, IN_teta_pos as float)
*/
        }
        gotoStateAfter(EVITEMENT_INIT_CHOICE, 1000);

        if (onExit()) {
/*
            // Fixe la vitesse pour toute la procédure d'évitement (on remet la valeur par défaut, le "tout doux" se fait par cde_min et cde_max
            //Application.m_asservissement.CommandeVitesseMouvement(evit_memo_vitesse_avance as float, evit_memo_vitesse_angle as float);
            // Modifie les paramètres de l'asservissement pour que les mouvements se fassent "tout doux" pendant l'évitement
            Application.m_asservissement.setCdeMinCdeMax(-10, 10)
 */

        }
        break;
    // ___________________________________
    case EVITEMENT_INIT_CHOICE :
        if (onEntry()) {
        }

        if ( (inputs()->evit_choix_strategie == 0) || internals()->evit_detection_obstacle_bitfield==0) {
            // Choix d'une stratégie neutre
            // ou
            // Plus aucun obstacle -> fin de l'évitement
            gotoState(SORTIE_EVITEMENT);
        }
        else {
            // Obstacle détecté
            gotoState(STRATEGIE_EVITEMENT);
        }

        if (onExit()) { }
        break;
    // ___________________________________
    case STRATEGIE_EVITEMENT :
        if (onEntry()) {
            internals()->evit_strategie_evitement_en_cours = true;
        }

        gotoStateAfter(EVITEMENT_INIT_CHOICE, 2000);

        if (onExit()) {
            internals()->evit_nombre_tentatives +=1;
        }
        break;
    // ___________________________________
    case EVITEMENT_ATTENTE :
        if (onEntry()) {
            //Application.m_asservissement.CommandeMouvementXY_TETA(IN_x_pos as float, IN_y_pos as float, IN_teta_pos as float);
            internals()->evit_debug_etape = 30;
        }
        gotoStateAfter(EVITEMENT_INIT_CHOICE, 2000);

        if (onExit()) {
        }
        break;
    // ___________________________________
    case SORTIE_EVITEMENT :
        if (onEntry()) {
            internals()->evitementEnCours=false;

            // Assure qu'aucun mouvement n'est en cours avant de restituer les paramètres de l'asserv
            // (risque d'accèl brutale sinon)
            Application.m_asservissement.CommandeManuelle(0,0); // Laisse en commande libre pour éviter de bourriner en cas de blocage

            // Restitution du contexte de l'asservissement avant de rendre la main
            Application.m_asservissement.setCdeMinCdeMax(internals()->evit_memo_cde_min, internals()->evit_memo_cde_max);
            Application.m_asservissement.setIndiceSportivite(internals()->evit_memo_idx_sportiv);
            Application.m_asservissement.CommandeVitesseMouvement(internals()->evit_memo_vitesse_avance, internals()->evit_memo_vitesse_angle);
            //forceObstacle = evit_memo_force_obstacle;
            internals()->evit_debug_etape = 99;
            stop();
            gotoState(EVITEMENT_INIT); // pour la prochaine fois qu'on entre dans l'évitement
        }


        if (onExit()) {
        }
        break;

    }
}
