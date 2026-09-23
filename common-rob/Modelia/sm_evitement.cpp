#include "CGlobale.h"
#include "sm_evitement.h"
#include "ia.h"

SM_Evitement::SM_Evitement()
{
}

const char* SM_Evitement::getName()
{
    return "SM_Evitement";
}

const char* SM_Evitement::stateToName(unsigned short state)
{
    switch(state)
    {
    case EVITEMENT_INIT :                       return "EVITEMENT_INIT";
    case EVITEMENT_INIT_CHOICE :                return "EVITEMENT_INIT_CHOICE";
    case STRATEGIE_EVITEMENT_FIN :              return "STRATEGIE_EVITEMENT_FIN";
    case SORTIE_EVITEMENT :                     return "SORTIE_EVITEMENT";
    case EVITEMENT_ATTENTE :                    return "EVITEMENT_ATTENTE";
    case STRATEGIE_ATTENDRE :                   return "STRATEGIE_ATTENDRE";
    case STRATEGIE_CONTOURNEMENT :              return "STRATEGIE_CONTOURNEMENT";
    case STRATEGIE_CONTOURNEMENT_ELOIGNEMENT :  return "STRATEGIE_CONTOURNEMENT_ELOIGNEMENT";
    case STRATEGIE_CONTOURNEMENT_ROTATION :     return "STRATEGIE_CONTOURNEMENT_ROTATION";
    case STRATEGIE_CONTOURNEMENT_EVACUE :       return "STRATEGIE_CONTOURNEMENT_EVACUE";
    case STRATEGIE_CONTOURNEMENT_REDRESSE :     return "STRATEGIE_CONTOURNEMENT_REDRESSE";
    case STRATEGIE_CONTOURNEMENT_EVACUE_2 :     return "STRATEGIE_CONTOURNEMENT_EVACUE_2";
#ifdef SM_DATASINTERFACE_EVITEMENT_AE
    case STRATEGIE_AE_EVAL :                    return "STRATEGIE_AE_EVAL";
    case STRATEGIE_AE_ARRET :                   return "STRATEGIE_AE_ARRET";
    case STRATEGIE_AE_GENTLEMAN :               return "STRATEGIE_AE_GENTLEMAN";
    case STRATEGIE_AE_ESQUIVE :                 return "STRATEGIE_AE_ESQUIVE";
    case STRATEGIE_AE_BLOCAGE :                 return "STRATEGIE_AE_BLOCAGE";
#endif
    }
    return "UNKNOWN_STATE";
}


// _____________________________________________________________
void SM_Evitement::step()
{
    switch(m_state)
    {
    // ___________________________________
    case EVITEMENT_INIT :
        if (onEntry()) {
            internals()->evitementEnCours=true;
            saveContext();
            internals()->evit_sens_avant_detection = Application.m_asservissement.getSensDeplacement();
            internals()->evit_force_obstacle = true;

            internals()->evit_debug_etape = 0;
            internals()->evit_nombre_tentatives = 0;
            internals()->evit_toggle_signe *= -1;  // permet de changer certains sens lorsqu'on revient dans l'évitement pour donner un caractère aléatoire

            // Arrêt du robot : l'arrêt se fait en donnant une consigne de vitesse nulle en avance et en angle
            // cela permet un arrêt plus rapide qu'une consigne manuelle à "0"
            Application.m_asservissement.CommandeMouvementXY_TETA(inputs()->X_robot, inputs()->Y_robot, inputs()->angle_robot);
        }
        gotoStateAfter(EVITEMENT_INIT_CHOICE, 1000);

        if (onExit()) {
            // Fixe la vitesse pour toute la procédure d'évitement (on remet la valeur par défaut, le "tout doux" se fait par cde_min et cde_max
            //Application.m_asservissement.CommandeVitesseMouvement(evit_memo_vitesse_avance as float, evit_memo_vitesse_angle as float);
            // Modifie les paramètres de l'asservissement pour que les mouvements se fassent "tout doux" pendant l'évitement
            Application.m_asservissement.setCdeMinCdeMax(CDE_MIN_TOUT_DOUX, CDE_MAX_TOUT_DOUX);
        }
        break;
    // ___________________________________
    case EVITEMENT_INIT_CHOICE :
        if (onEntry()) {
        }

#ifdef SM_DATASINTERFACE_EVITEMENT_AE
        // Strategie AE : le champ de bits de detection (issu des capteurs US ou de leur emulation)
        // n'est pas representatif de la situation tactique -> on entre directement dans l'echelle
        // de phases, qui decide sur le verdict de l'evaluation tactique (couche 3).
        if (internals()->evit_choix_strategie == SM_DatasInterface::STRATEGIE_EVITEMENT_AE) {
            internals()->evit_strategie_evitement_en_cours = true;
            gotoState(STRATEGIE_AE_EVAL);
        }
        else
#endif
        // Plus aucun obstacle -> fin de l'évitement sans attendre
        if (internals()->evit_detection_obstacle_bitfield==0) {
            gotoState(SORTIE_EVITEMENT);
        }
        else { // Obstacle toujours présent -> commence une stratégie d'évitement
            internals()->evit_strategie_evitement_en_cours = true;
            if (internals()->evit_choix_strategie == SM_DatasInterface::STRATEGIE_EVITEMENT_CONTOURNER) {
                gotoState(STRATEGIE_CONTOURNEMENT);
            }
            // TODO : mettre ici les autres stratégies d'évitement possibles s'il y en a
            else {  // Ici comme il n'y a qu'une seule stratégie d'évitement (contournement), on revient dessus dans tous les cas
                gotoState(STRATEGIE_ATTENDRE);
            }
        }

        if (onExit()) { }
        break;
    // ___________________________________
    // On revient ici quand la stratégie d'évitement sélectionnée est terminée
    case STRATEGIE_EVITEMENT_FIN :
        if (onEntry()) {
            internals()->evit_nombre_tentatives++;
            internals()->evit_strategie_evitement_en_cours = false;
        }
        // Si le nombre de tentatives autorisés est atteint, termine l'évitement
        // Sinon, refait une nouvelle tentative d'évitement
        if (internals()->evit_nombre_tentatives >= internals()->evit_nombre_max_tentatives) {
            gotoState(SORTIE_EVITEMENT);
        }
        else {
            gotoState(EVITEMENT_INIT_CHOICE);
        }

        if (onExit()) { }
        break;
    // ___________________________________
    // TODO : cet état n'est plus utilisé.
    // A confirmer avec des tests complémerntaires avant de le supprimer
    case EVITEMENT_ATTENTE :
        if (onEntry()) {
            // Reste sur place
            Application.m_asservissement.CommandeMouvementXY_TETA(inputs()->X_robot, inputs()->Y_robot, inputs()->angle_robot);
            internals()->evit_debug_etape = 30;
        }
        gotoStateAfter(EVITEMENT_INIT_CHOICE, 1000);

        if (onExit()) {
        }
        break;
    // ___________________________________
    case SORTIE_EVITEMENT :
        if (onEntry()) {

            // Assure qu'aucun mouvement n'est en cours avant de restituer les paramètres de l'asserv
            // (risque d'accèl brutale sinon)
            Application.m_asservissement.CommandeManuelle(0,0); // Laisse en commande libre pour éviter de bourriner en cas de blocage

            // Restitution du contexte de l'asservissement avant de rendre la main
            restoreContext();
            internals()->evit_debug_etape = 99;
            stop();
            gotoState(EVITEMENT_INIT); // pour la prochaine fois qu'on entre dans l'évitement
        }


        if (onExit()) {
            internals()->evitementEnCours=false;
            internals()->evit_debug_etape = 0;
        }
        break;

    // ===================================================
    // STRATEGIE D'EVITEMENT : ATTENDRE
    // Dans cette stratégie, le robot attend sans rien faire un certain temps
    // ===================================================
    case STRATEGIE_ATTENDRE :
        if (onEntry()) { }
        gotoStateAfter(STRATEGIE_EVITEMENT_FIN, 1000);
        if (onExit()) { }
        break;
    // ===================================================
    // STRATEGIE D'EVITEMENT : CONTOURNEMENT
    // ===================================================
    // Variable "evit_detection_obstacle_bitfield" :
    // Champ de bit représentant la présence d'un obstacle sur chaque capteur US :
    //  ARG   ARD  AVG  AVD
    //   0     0    0    0   =0 -> Aucun obstacle
    //   1     0    1    0   =6 -> Obstacle ARG et AVG
    //   ...
    // Stratégie d'évitement "Contournement" en 5 actions :
    //    1. Eloignement
    //    2. Rotation 45°
    //    3. Evacuation
    //    4. Redressement
    //    5. Contournement
    case STRATEGIE_CONTOURNEMENT :
        if (onEntry()) {
            if (internals()->evit_detection_obstacle_bitfield == 1) {       // AVD
                internals()->evit_sgn_dist_eloigne  = -1;
                internals()->evit_sgn_angle_pivote  = +1;
                internals()->evit_sgn_dist_evacue   = +1;
                gotoState(STRATEGIE_CONTOURNEMENT_ELOIGNEMENT);
            }
            else if (internals()->evit_detection_obstacle_bitfield == 2) {  // AVG
                internals()->evit_sgn_dist_eloigne  = -1;
                internals()->evit_sgn_angle_pivote  = -1;
                internals()->evit_sgn_dist_evacue   = +1;
                gotoState(STRATEGIE_CONTOURNEMENT_ELOIGNEMENT);
            }
            else if (internals()->evit_detection_obstacle_bitfield == 3) {  // AGD-AVG
                internals()->evit_sgn_dist_eloigne  = -1;
                internals()->evit_sgn_angle_pivote  = internals()->evit_toggle_signe;
                internals()->evit_sgn_dist_evacue   = +1;
                gotoState(STRATEGIE_CONTOURNEMENT_ELOIGNEMENT);
            }
            else if (internals()->evit_detection_obstacle_bitfield == 4) {  // ARD
                internals()->evit_sgn_dist_eloigne  = +1;
                internals()->evit_sgn_angle_pivote  = -1;
                internals()->evit_sgn_dist_evacue   = -1;
                gotoState(STRATEGIE_CONTOURNEMENT_ELOIGNEMENT);
            }
            else if (internals()->evit_detection_obstacle_bitfield == 8) {  // ARG
                internals()->evit_sgn_dist_eloigne  = +1;
                internals()->evit_sgn_angle_pivote  = +1;
                internals()->evit_sgn_dist_evacue   = -1;
                gotoState(STRATEGIE_CONTOURNEMENT_ELOIGNEMENT);
            }
            else if (internals()->evit_detection_obstacle_bitfield == 12) { // ARD-ARG
                internals()->evit_sgn_dist_eloigne  = +1;
                internals()->evit_sgn_angle_pivote  = internals()->evit_toggle_signe;
                internals()->evit_sgn_dist_evacue   = -1;
                gotoState(STRATEGIE_CONTOURNEMENT_ELOIGNEMENT);
            }
            else {  // Cas d'un blocage par l'avant ET par l'arrière -> ne rien faire (rend la main)
                gotoState(STRATEGIE_EVITEMENT_FIN);
            }
        }

        if (onExit()) { }
        break;
    // ___________________________________
    case STRATEGIE_CONTOURNEMENT_ELOIGNEMENT :
        if (onEntry()) {
            Application.m_asservissement.CommandeMouvementDistanceAngle(internals()->evit_sgn_dist_eloigne * (15 + 3*internals()->evit_nombre_tentatives), inputs()->angle_robot);
            internals()->evit_debug_etape = 21;
        }
        gotoStateIfConvergenceRapide(STRATEGIE_CONTOURNEMENT_ROTATION, 5000);
        if (onExit()) { }
        break;
    // ___________________________________
    case STRATEGIE_CONTOURNEMENT_ROTATION :
        if (onEntry()) {
            Application.m_asservissement.CommandeMouvementDistanceAngle(0, inputs()->angle_robot + internals()->evit_sgn_angle_pivote * 1.1f);
            internals()->evit_debug_etape = 22;
        }
        gotoStateIfConvergenceRapide(STRATEGIE_CONTOURNEMENT_EVACUE, 3000);
        if (onExit()) { }
        break;
    // ___________________________________
    case STRATEGIE_CONTOURNEMENT_EVACUE :
        if (onEntry()) {
            Application.m_asservissement.CommandeMouvementDistanceAngle(internals()->evit_sgn_dist_evacue * (40 + 4*internals()->evit_nombre_tentatives), inputs()->angle_robot);
            internals()->evit_debug_etape = 23;
        }
        gotoStateIfConvergenceRapide(STRATEGIE_CONTOURNEMENT_REDRESSE, 5000);
        // Si un obstacle est détecté pendant cette phase, recommence l'évitement
        if (inputs()->obstacleDetecte) {
            gotoState(STRATEGIE_EVITEMENT_FIN);
        }
        if (onExit()) { }
        break;
    // ___________________________________
    case STRATEGIE_CONTOURNEMENT_REDRESSE :
        if (onEntry()) {
            Application.m_asservissement.CommandeMouvementDistanceAngle(0, inputs()->angle_robot - internals()->evit_sgn_angle_pivote * 0.9f);
            internals()->evit_debug_etape = 24;
        }
        gotoStateIfConvergenceRapide(STRATEGIE_CONTOURNEMENT_EVACUE_2, 3000);
        if (onExit()) { }
        break;
    // ___________________________________
    case STRATEGIE_CONTOURNEMENT_EVACUE_2 :
        if (onEntry()) {
            Application.m_asservissement.CommandeMouvementDistanceAngle(internals()->evit_sgn_dist_evacue * (50 + 4*internals()->evit_nombre_tentatives), inputs()->angle_robot);
            internals()->evit_debug_etape = 25;
        }
        gotoStateIfConvergenceRapide(STRATEGIE_EVITEMENT_FIN, 2000);
        // Si un obstacle est détecté pendant cette phase, recommence l'évitement
        if (inputs()->obstacleDetecte) {
            gotoState(STRATEGIE_EVITEMENT_FIN);
        }
        if (onExit()) { }
        break;

#ifdef SM_DATASINTERFACE_EVITEMENT_AE
    // ===================================================
    // STRATEGIE D'EVITEMENT : AE (atelier evitement 2027)
    // Echelle de phases REENTRANTE : chaque entree dans l'evitement ne joue qu'UNE phase, puis rend
    // la main a la mission. La marche atteinte est memorisee dans "evit_ae_state" ; si l'adversaire
    // est toujours la a l'entree suivante, on monte d'une marche. Quand la voie se degage, IA remet
    // l'echelle a zero. C'est ce qui evite la manoeuvre interminable de la strategie CONTOURNEMENT.
    //
    // Les marches PRUDENCE et RALENTI ne sont PAS des etats de cette machine : elles ne sont qu'un
    // plafond de vitesse applique en continu par IA, sans manoeuvre ni interruption de la mission.
    // L'echelle de la machine a etats commence donc a l'arret.
    //
    // Cette machine est un SEQUENCEUR PUR : toute la geometrie (recul possible, esquive possible et
    // son cap) est calculee par IA et lue ici sous forme de decisions deja prises. C'est ce qui
    // permet a ce fichier, partage par tous les robots du club, de ne rien connaitre du lidar.
    // ===================================================
    case STRATEGIE_AE_EVAL :
        if (onEntry()) {
            internals()->evit_debug_etape = 40;
        }
        // La voie s'est degagee entre-temps : on ne manoeuvre pas pour rien
        if (internals()->evit_menace < MENACE_ARRET) {
            gotoState(STRATEGIE_EVITEMENT_FIN);
        }
        else if (internals()->evit_ae_state < SM_DatasInterface::ETAT_AE_ARRET) {
            gotoState(STRATEGIE_AE_ARRET);
        }
        else if (internals()->evit_ae_state == SM_DatasInterface::ETAT_AE_ARRET) {
            gotoState(STRATEGIE_AE_GENTLEMAN);
        }
        else if (internals()->evit_ae_state == SM_DatasInterface::ETAT_AE_GENTLEMAN) {
            gotoState(STRATEGIE_AE_ESQUIVE);
        }
        else {
            gotoState(STRATEGIE_AE_BLOCAGE);
        }
        if (onExit()) { }
        break;
    // ___________________________________
    // Premiere marche : on s'arrete sur place et on laisse sa chance a l'adversaire de passer.
    case STRATEGIE_AE_ARRET :
        if (onEntry()) {
            internals()->evit_debug_etape = 41;
            internals()->evit_ae_state = SM_DatasInterface::ETAT_AE_ARRET;
            // Memorise la pose : elle sert de consigne d'arret ici, et de cap de reference pour les
            // manoeuvres des marches suivantes
            internals()->evit_ae_memo_X     = inputs()->X_robot;
            internals()->evit_ae_memo_Y     = inputs()->Y_robot;
            internals()->evit_ae_memo_Theta = inputs()->angle_robot;
            // Arret par consigne de position (plus rapide qu'une consigne manuelle a zero)
            Application.m_asservissement.CommandeMouvementXY_TETA(internals()->evit_ae_memo_X,
                                                                  internals()->evit_ae_memo_Y,
                                                                  internals()->evit_ae_memo_Theta);
        }
        gotoStateAfter(STRATEGIE_EVITEMENT_FIN, internals()->evit_ae_tempo_arret_ms);
        // Sortie anticipee si la voie se degage : place apres le gotoStateAfter pour etre prioritaire
        if (internals()->evit_menace < MENACE_ARRET) {
            gotoState(STRATEGIE_EVITEMENT_FIN);
        }
        if (onExit()) { }
        break;
    // ___________________________________
    // Deuxieme marche : le "gentleman move". Un petit recul dans l'axe, qui ouvre le passage sans
    // engager le robot ailleurs, et qui suffit souvent a debloquer un face-a-face.
    case STRATEGIE_AE_GENTLEMAN :
        if (onEntry()) {
            internals()->evit_debug_etape = 42;
            internals()->evit_ae_state = SM_DatasInterface::ETAT_AE_GENTLEMAN;
            if (internals()->evit_recul_possible) {
                // Recule dans le sens oppose a celui ou l'on allait, cap conserve
                Application.m_asservissement.CommandeMouvementDistanceAngle(
                            -internals()->evit_sens_avant_detection * DISTANCE_AE_GENTLEMAN_CM,
                            internals()->evit_ae_memo_Theta);
            }
        }
        // Robot accule a la bordure : la marche est sautee, on passera a l'esquive la prochaine fois
        if (!internals()->evit_recul_possible) {
            gotoState(STRATEGIE_EVITEMENT_FIN);
        }
        gotoStateIfConvergenceRapide(STRATEGIE_EVITEMENT_FIN, TIMEOUT_AE_MANOEUVRE_MS);
        if (onExit()) { }
        break;
    // ___________________________________
    // Troisieme marche : l'esquive. On se decale du cote libre, juste assez pour sortir du couloir
    // de l'adversaire. Le cap d'esquive est calcule par IA a partir de la geometrie de la menace.
    case STRATEGIE_AE_ESQUIVE :
        if (onEntry()) {
            internals()->evit_debug_etape = 43;
            internals()->evit_ae_state = SM_DatasInterface::ETAT_AE_ESQUIVE;
            if (internals()->evit_esquive_possible) {
                Application.m_asservissement.CommandeMouvementDistanceAngle(
                            internals()->evit_sens_avant_detection * DISTANCE_AE_ESQUIVE_CM,
                            internals()->evit_esquive_cap_rad);
            }
        }
        // Aucun cote libre ou pas la place : marche sautee, on tombera en blocage a la prochaine entree
        if (!internals()->evit_esquive_possible) {
            gotoState(STRATEGIE_EVITEMENT_FIN);
        }
        gotoStateIfConvergenceRapide(STRATEGIE_EVITEMENT_FIN, TIMEOUT_AE_MANOEUVRE_MS);
        if (onExit()) { }
        break;
    // ___________________________________
    // Derniere marche : le blocage. Toutes les manoeuvres ont echoue -> on tient la position plutot
    // que de s'acharner (c'est la que l'ancienne strategie partait en mouvements erratiques). Au
    // bout d'un certain temps on redescend l'echelle pour retenter les manoeuvres : l'adversaire a
    // pu bouger, et rester fige jusqu'a la fin du match ne rapporte rien.
    case STRATEGIE_AE_BLOCAGE :
        if (onEntry()) {
            internals()->evit_debug_etape = 44;
            internals()->evit_ae_state = SM_DatasInterface::ETAT_AE_BLOCAGE;
            internals()->evit_ae_chrono_blocage_ms += TEMPO_AE_BLOCAGE_PAS_MS;
            // Reste sur place, sur la pose memorisee a l'arret
            Application.m_asservissement.CommandeMouvementXY_TETA(internals()->evit_ae_memo_X,
                                                                  internals()->evit_ae_memo_Y,
                                                                  internals()->evit_ae_memo_Theta);
            if (internals()->evit_ae_chrono_blocage_ms >= TIMEOUT_AE_BLOCAGE_MS) {
                internals()->evit_ae_chrono_blocage_ms = 0;
                internals()->evit_ae_state = SM_DatasInterface::ETAT_AE_ARRET;  // on retentera le recul
            }
        }
        gotoStateAfter(STRATEGIE_EVITEMENT_FIN, TEMPO_AE_BLOCAGE_PAS_MS);
        if (internals()->evit_menace < MENACE_ARRET) {
            gotoState(STRATEGIE_EVITEMENT_FIN);
        }
        if (onExit()) { }
        break;
#endif // SM_DATASINTERFACE_EVITEMENT_AE

    // ===================================================
    // ===================================================
    default :
        m_state = EVITEMENT_INIT;
        break;
    }
}



// _____________________________________________________________
void SM_Evitement::saveContext()
{
    internals()->evit_memo_cde_min          = Application.m_asservissement.cde_min;
    internals()->evit_memo_cde_max          = Application.m_asservissement.cde_max;
    internals()->evit_memo_idx_sportiv      = Application.m_asservissement.Ind_perfo;
    internals()->evit_memo_vitesse_avance   = Application.m_asservissement.vitesse_avance_max;
    internals()->evit_memo_vitesse_angle    = Application.m_asservissement.vitesse_rotation_max;
    internals()->evit_memo_force_obstacle   = internals()->evit_force_obstacle;
    internals()->evit_memo_choix_strategie  = internals()->evit_choix_strategie;
}

void SM_Evitement::restoreContext()
{
    Application.m_asservissement.setCdeMinCdeMax(internals()->evit_memo_cde_min, internals()->evit_memo_cde_max);
    Application.m_asservissement.setIndiceSportivite(internals()->evit_memo_idx_sportiv);
    Application.m_asservissement.CommandeVitesseMouvement(internals()->evit_memo_vitesse_avance, internals()->evit_memo_vitesse_angle);
    internals()->evit_force_obstacle = internals()->evit_memo_force_obstacle;
    internals()->evit_choix_strategie = internals()->evit_memo_choix_strategie;

}
