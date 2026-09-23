/*! \file CTacticalEvaluator.h
    \brief Evaluation tactique des pistes suivies (couche 3 de l'evitement)

    Repond a une seule question, a chaque pas de modele : parmi les objets suivis, lequel me gene,
    et a quel point ? Elle ne commande rien -- c'est la machine a etats d'evitement qui decide.

    Deux criteres, repris du document "Nouvelle strategie d'evitement" du club :
      - proximite : la distance D a l'objet ;
      - trajectoire : l'objet est dans notre couloir si |phi| <= asin((R0+R1)/D).
    Le suivi temporel en ajoute un troisieme, qu'une mesure instantanee ne peut pas donner : le
    point d'approche minimale et le temps qui nous en separe (TTC). Deux objets a la meme distance
    et au meme angle appellent des decisions opposees selon qu'ils s'eloignent ou qu'ils foncent.

    Cette classe ne connait ni l'application, ni l'asservissement : on lui donne des pistes, une
    pose, un cap de trajectoire et une vitesse.
*/
#ifndef _CTACTICAL_EVALUATOR_H_
#define _CTACTICAL_EVALUATOR_H_

#include "CObstacleTracker.h"

//! Niveaux de menace, dans l'ordre croissant : la machine a etats s'en sert comme d'une echelle
typedef enum {
    MENACE_LIBRE = 0,   //!< rien sur la route
    MENACE_PRUDENCE,    //!< quelque chose au loin sur la route : on leve le pied
    MENACE_RALENTI,     //!< il se rapproche : on ralentit franchement
    MENACE_ARRET        //!< il faut s'arreter
}eNiveauMenace;

//! Verdict de l'evaluation, et la piste qui l'a motive
typedef struct {
    unsigned char niveau;       //!< eNiveauMenace
    bool  piste_retenue;        //!< false si aucune piste ne gene
    float D_cm;                 //!< distance de la piste retenue
    float phi_rad;              //!< angle relatif au cap de trajectoire (+ a gauche)
    float ttc_s;                //!< temps avant approche minimale ; negatif : l'objet s'eloigne
    float dmin_cm;              //!< distance d'approche minimale, a vitesses constantes
    signed char cote_libre;     //!< cote ou s'ecarter : +1 gauche, -1 droite, 0 aucun
    bool  statique;             //!< la piste retenue est immobile
    bool  forme_douteuse;       //!< la piste retenue ne ressemble pas a un mat balise
}tMenace;

class CTacticalEvaluator
{
public :
    CTacticalEvaluator();
    void init();

    /*! \brief Evalue les pistes et rend un niveau de menace
        \param suivi pistes de la couche 2
        \param x_robot_cm, y_robot_cm pose du robot, repere terrain
        \param cap_trajectoire_rad cap dans lequel le robot va -- PAS son cap geometrique : en
               marche arriere, c'est le cap oppose. Le couloir de detection suit la trajectoire.
        \param vitesse_robot_cms vitesse d'avance du robot le long de ce cap (positive)
    */
    void evaluer(const CObstacleTracker &suivi,
                 float x_robot_cm, float y_robot_cm,
                 float cap_trajectoire_rad, float vitesse_robot_cms);

    const tMenace& menace() const { return m_menace; }

    // ---- parametres de reglage
    float m_R0_cm;                  //!< rayon englobant de notre robot
    float m_R1_cm;                  //!< rayon estime du robot adverse
    float m_seuil_prudence_cm;      //!< au-dela, aucune reaction
    float m_seuil_ralenti_cm;
    float m_seuil_arret_cm;
    float m_hysteresis_cm;          //!< marge ajoutee aux seuils pour redescendre d'un niveau
    float m_ttc_arret_s;            //!< arret anticipe si le contact est imminent
    unsigned char m_confirmations_detente;  //!< evaluations consecutives avant de detendre
    float m_terrain_x_min_cm, m_terrain_x_max_cm;   //!< bornes du terrain : au-dela, on ignore
    float m_terrain_y_min_cm, m_terrain_y_max_cm;
    float m_marge_bord_cm;          //!< marge de bord pour choisir le cote d'esquive

private :
    static float moduloPi(float angle_rad);
    bool horsTerrain(float x_cm, float y_cm) const;
    signed char choisirCoteLibre(float x_robot_cm, float y_robot_cm,
                                 float cap_trajectoire_rad, float phi_piste_rad) const;

    tMenace m_menace;
    unsigned char m_compteur_detente;
};

#endif // _CTACTICAL_EVALUATOR_H_
