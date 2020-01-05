/*! \file CAsservissementBase.h
    \brief Classe qui contient l'asservissement vitesse/position du robot
*/

#ifndef _ASSERVISSEMENT_BASE_H_
#define _ASSERVISSEMENT_BASE_H_

// -----------------------------
//! Classe de gestion des options d'exécution passees en ligne de commande
class CAsservissementBase {
public :
    // Les 3 constantes spécifiques au robot
    // (à redéfinir dans le CAsservissement.cpp de chaque robot)
    static const float DISTANCE_PAR_PAS_CODEUR_G;   // Roue gauche
    static const float DISTANCE_PAR_PAS_CODEUR_D;   // Roue droite
    static const float VOIE_ROBOT;                  // Distance entre les 2 roues [cm]
#define FACTEUR_CONV_DELTA_DIST_VERS_ANGLE		(1 / VOIE_ROBOT)	// (2 * Pi) / ( 2 * Pi * VOIE_ROBOT) => Resultat en rad

    // Les cartos spécifiques au robot
#define NBRE_POINTS_CARTO_ERREUR 13
    static const float ini_conv_erreur_dist_vitesse_cur_x[NBRE_POINTS_CARTO_ERREUR];    // [cm]
    static const float ini_conv_erreur_dist_vitesse_1_cur[NBRE_POINTS_CARTO_ERREUR];    // [cm/s] Carto perfo 0% environ 30cm/s2
    static const float ini_conv_erreur_dist_vitesse_2_cur[NBRE_POINTS_CARTO_ERREUR];

    static const float ini_conv_erreur_angle_vitesse_cur_x[NBRE_POINTS_CARTO_ERREUR];   // [rad]
    static const float ini_conv_erreur_angle_vitesse_1_cur[NBRE_POINTS_CARTO_ERREUR];   // [rad/s] Carto perfo 0% environ 5rad/s2
    static const float ini_conv_erreur_angle_vitesse_2_cur[NBRE_POINTS_CARTO_ERREUR];   // [rad/s] Carto perfo 100% environ 20rad/s2

    typedef enum {
        cMODE_MANUEL = 0,
        cMODE_XY_AUTO,
        cMODE_XY_AUTO_A,
        cMODE_XY_AUTO_B,
        cMODE_DISTANCE_ANGLE,
        cMODE_XY_TETA
    } tTypeAsservissement;

    float te;  // [s] Temps entre chaque appel de la tache asservissement
    // Les paramètres de l'asservissement
    int cde_max;					// %	Commande maximum normalisée pour saturer la régulation
    int cde_min;					// %	Commande minimum normalisée pour saturer la régulation
    float kp_distance;				// 		Gain proportionnel pour la régulation en distance
    float ki_distance;				// 		Gain intégral pour la régulation en distance
    float kp_angle;					// 		Gain proportionnel pour la régulation en angle
    float ki_angle;					// 		Gain intégral pour la régulation en angle
    float k_angle;					//		Coeff de filtrage pour le terme dérivé
    float seuil_conv_distance;		// cm	Erreur en dessous de laquelle on considère que le robot est en position sur la distance
    float seuil_conv_angle;			// rad	Erreur en dessous de laquelle on considère que le robot est en position sur l'angle
    unsigned int compteur_max;		// 		Nombre de coups d'horloge (N*te) avant de confirmer que le robot est en position
    int zone_morte_D;
    int zone_morte_G;

    // Les variables de l'asservissement
    float distance_consigne;
    float angle_consigne;
    float TETA_consigne;
    float distance_roue_D;
    float distance_roue_G;
    float erreur_distance;
    float erreur_angle;
    float erreur_vitesse_avance;
    float erreur_vitesse_rotation;
    float int_terme_prec_distance;
    float int_terme_distance;
    float cde_distance_ulim;
    float cde_distance_recal;
    float cde_distance;
    float int_terme_prec_angle;
    float int_terme_angle;
    float cde_angle_ulim;
    float cde_angle;
    float cde_moteur_D;
    float cde_moteur_D_ulim;
    float cde_moteur_G;
    float cde_moteur_G_ulim;
    float angle_robot_prec;
    float distance_roue_D_prec;
    float distance_roue_G_prec;
    float X_robot_prec;
    float Y_robot_prec;
    float ddistance_roue_D;
    float ddistance_roue_G;
    float ddistance_robot;
    float distance_robot;
    float dangle_robot;
    float angle_robot;
    float dX_robot;
    float dY_robot;
    float X_robot;
    float X_consigne;
    float Y_robot;
    //float Y_Consigne;
    float X_erreur;
    float Y_erreur;
    float Y_consigne;
    float temp_cos_angle_robot;
    float temp_sin_angle_robot;
    float Xa;
    float Ya;
    float Xb;
    float Yb;
    float Offset_a;
    float Offset_b;
    float cde_offset_min_D;
    float cde_offset_min_G;
    //float tension_batt_mes;   			// multiplier par le facteur du pont diviseur pour la mesure en volts
    float ubatt_nominale;
    float k_batt;
    float correction_ubatt;
    //float m_tension_batterie;
    float m_tension_batterie_filt;
    float m_tension_batterie_filt_prec;
    float vitesse_avance_robot;
    float vitesse_rotation_robot;
    float vitesse_avance_robot_filt_prec;
    float vitesse_avance_robot_filt;
    float k_filt_avance;
    float vitesse_rotation_robot_filt_prec;
    float vitesse_rotation_robot_filt;
    float k_filt_rotation;
    float consigne_vitesse_avance;
    float consigne_vitesse_rotation;
    float consigne_vitesse_avance_filt;
    float consigne_vitesse_avance_recal;
    float consigne_vitesse_rotation_cm;
    float consigne_vitesse_rotation_filt;
    float facteur_correction_avance_angle;
    float seuil_vitesse_diag_blocage;		// cm/s
    float commande_min_diag_blocage;		// En cas de blocage l'intégrateur charge rapidement à des valeurs importantes
    float seuil_vitesse_diag_rotation;	// rad/s
    int seuil_max_compteur_diag_blocage;	// 25 * te,5s
    int inc_diag_blocage;
    int dec_diag_blocage;
    int compteur_diag_blocage;
    char diag_blocage;
    char convergence_rapide;
    char convergence;
    char convergence_conf;						// 0: Mvt en cours, 1: Convergence ok, 2: Blocage détecté
    char saturation_distance;
    char saturation_angle;
    char saturation_moteur_D;
    char saturation_moteur_G;
    char verouillage_approche;
    unsigned int compteur_convergence;
    unsigned char ModeAsservissement;
    float consigne_vitesse_avance_base;
    float consigne_vitesse_avance_perfo;
    float consigne_vitesse_rotation_base;
    float consigne_vitesse_rotation_perfo;

    float offset_vitesse_avance;				//	[cm/s-2]
    float offset_vitesse_rotation;				//	[rad/s-2]
    float vitesse_avance_max;					//	[cm/s]
    float vitesse_rotation_max;					//	[rad/s]
    float Ind_perfo;							//	coeff entre 0 et 1 (possibilité d'extrapoler mais attention car pas de limitation)

    float conv_erreur_dist_vitesse_cur_x[NBRE_POINTS_CARTO_ERREUR];		// [cm]
    float conv_erreur_dist_vitesse_1_cur[NBRE_POINTS_CARTO_ERREUR];		// [cm/s]
    float conv_erreur_dist_vitesse_2_cur[NBRE_POINTS_CARTO_ERREUR];		// [cm/s]

    float conv_erreur_angle_vitesse_cur_x[NBRE_POINTS_CARTO_ERREUR];	// [rad]
    float conv_erreur_angle_vitesse_1_cur[NBRE_POINTS_CARTO_ERREUR];	// [rad/s]
    float conv_erreur_angle_vitesse_2_cur[NBRE_POINTS_CARTO_ERREUR];	// [rad/s]

    //float facteur_cor_vitesse_avance_cur_x[NBRE_POINTS_CARTO_ERREUR];
    //float facteur_cor_vitesse_avance_cur[NBRE_POINTS_CARTO_ERREUR];

    //! Constructeur / destructeur
    CAsservissementBase();
    virtual ~CAsservissementBase();

    //! Réinitilise tous les paramètres et valeurs
    virtual void Init(void);

    // Prototype des fonctions
    virtual void CommandeMouvementXY(float x, float y);
    virtual void CommandeMouvementDistanceAngle(float distance, float angle);
    virtual void CommandeManuelle(float cde_G, float cde_D);
    virtual void CommandeMouvementXY_A(float x, float y);
    virtual void CommandeMouvementXY_B(float x, float y);
    virtual void CommandeMouvementXY_TETA(float x, float y, float teta);
    virtual void CommandeVitesseMouvement(float vit_avance, float vit_angle);
    virtual void CalculsMouvementsRobots(void);
    virtual void setPosition_XYTeta(float x, float y, float teta);
    virtual void setIndiceSportivite(float idx);
    virtual void setCdeMinCdeMax(int min, int max);

protected :
    float Calc_carto_cur(float entree_x, float axe_x[], float axe_z[], int nb_point_x);
    void Initialisation_PID(void);
    void Reset_Pas_Cumules(void);
    void Stop_robot(void);
    float saturation(float valeur_ulim, float valeur_min, float valeur_max);
    void LectureCodeurCalculDistance(void);
    void asservissement(void);
    void CalculXY (void);
    void Generateur_trajectoire (void);
    float max(float Inp1, float Inp2);
    float min(float Inp1, float Inp2);
    float SAT(float Inp_max, float Inp_min, float Inp);
    float BornageAngle(float angle);     //! Bornage de l'angle pour les mouvement distance/angle
};


#endif


