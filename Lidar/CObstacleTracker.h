/*! \file CObstacleTracker.h
    \brief Suivi temporel des objets detectes par le lidar (couche 2 de l'evitement)

    Transforme une liste d'objets vus par le filtre -- des taches, dans le repere du robot -- en une
    liste de PISTES persistantes, positionnees dans le repere du TERRAIN, avec leur vitesse.

    C'est le changement de repere qui cree l'information : vu du robot, un objet fixe et un objet
    mobile se deplacent tous les deux, puisque c'est nous qui bougeons. Projetes sur le terrain avec
    la pose du robot, l'objet fixe reste en place et l'adversaire dessine une trajectoire.

    Cette classe ne connait ni l'application, ni le terrain, ni la strategie : on lui donne des
    objets, une pose et une date, elle rend des pistes. Le tri (hors terrain, statique, menacant)
    appartient a la couche tactique.
*/
#ifndef _COBSTACLE_TRACKER_H_
#define _COBSTACLE_TRACKER_H_

#include "lidar_blob.h"

//! Une piste : un objet suivi d'un tour de balayage a l'autre
typedef struct {
    bool  valide;
    float X_cm;             //!< position, repere TERRAIN
    float Y_cm;
    float Vx_cms;           //!< vitesse, repere TERRAIN [cm/s]
    float Vy_cms;
    float largeur_deg;      //!< largeur angulaire apparente, recopiee du dernier objet associe
    unsigned short age_ms;  //!< duree depuis la creation de la piste (sature a 65000)
    unsigned char confirmations;  //!< tours de balayage consecutifs avec une mesure
    unsigned char sans_mesure;    //!< tours de balayage consecutifs sans mesure
    bool  statique;         //!< immobile de facon confirmee (avec hysteresis)
    bool  forme_douteuse;   //!< dernier objet associe hors des courbes enveloppes du filtre
}tObstacleTrack;

class CObstacleTracker
{
public :
    static const int NBRE_MAX_TRACKS = 12;

    CObstacleTracker();

    //! Vide toutes les pistes (debut de match, rechargement, perte du lidar)
    void init();

    /*! \brief Un nouveau tour de balayage : associe, cree, vieillit, et estime les vitesses
        \param objets objets rendus par le filtre, dans le repere du robot
        \param x_robot_cm, y_robot_cm position du robot dans le repere terrain
        \param cap_robot_rad cap du robot dans le repere terrain
        \param date_ms date de ce tour de balayage [ms]

        Les objets marques douteux par le filtre sont suivis comme les autres : un objet est
        declasse, jamais supprime (a l'homologation, le lidar voit le mat du robot factice et le
        bras de l'arbitre qui le pousse).
    */
    void nouveauScan(const CLidarBlobs &objets, float x_robot_cm, float y_robot_cm,
                     float cap_robot_rad, unsigned long date_ms);

    /*! \brief Entre deux tours de balayage : avance les pistes de leur vitesse
        Le lidar tourne a ~8 Hz et le modele a 50 Hz : le meme balayage sert a plusieurs pas.
    */
    void extrapoler(unsigned long date_ms);

    int count() const;                                  //!< nombre de pistes valides
    const tObstacleTrack* tracks() const { return m_tracks; }
    //! Piste valide la plus proche d'un point du terrain, ou 0 s'il n'y en a aucune
    const tObstacleTrack* plusProche(float x_cm, float y_cm) const;

    // ---- parametres de reglage
    float m_porte_association_cm;           //!< au-dela, la mesure cree une piste au lieu d'en nourrir une
    float m_seuil_statique_cms;             //!< en dessous, la piste est candidate a l'etat statique
    float m_seuil_mobile_cms;               //!< au-dessus, la piste redevient mobile
    unsigned char m_scans_sans_mesure_max;  //!< tours sans mesure avant de perdre la piste
    unsigned char m_scans_confirmation;     //!< tours consecutifs avant de confirmer statique/mobile
    float m_filtre_vitesse;                 //!< poids de l'ancienne vitesse dans le filtre du 1er ordre

private :
    int  chercherPisteLibre();
    void vieillir(tObstacleTrack &piste, float dt_s);

    tObstacleTrack m_tracks[NBRE_MAX_TRACKS];
    // Position de la piste AU DERNIER TOUR DE BALAYAGE, distincte de la position courante, qui est
    // extrapolee entre deux tours. La vitesse se mesure sur le deplacement d'un tour a l'autre : la
    // calculer depuis la position extrapolee ne mesurerait que l'ecart residuel, et l'estimation
    // s'effondrerait vers zero pour un objet a vitesse constante.
    float m_x_dernier_scan[NBRE_MAX_TRACKS];
    float m_y_dernier_scan[NBRE_MAX_TRACKS];
    //! Date de la derniere mesure associee a la piste : une piste qui saute un tour doit voir sa
    //! vitesse calculee sur la duree reellement ecoulee, sinon elle est surestimee d'autant.
    unsigned long m_date_derniere_mesure[NBRE_MAX_TRACKS];
    unsigned char  m_compteur_statique[NBRE_MAX_TRACKS];
    unsigned char  m_compteur_mobile[NBRE_MAX_TRACKS];
    unsigned long  m_date_dernier_scan_ms;
    unsigned long  m_date_extrapolation_ms;
    bool           m_premier_scan;
};

#endif // _COBSTACLE_TRACKER_H_
