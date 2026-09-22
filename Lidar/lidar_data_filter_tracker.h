#ifndef _LIDAR_DATA_FILTER_TRACKER_H_
#define _LIDAR_DATA_FILTER_TRACKER_H_

#include "lidar_data_filter_base.h"
#include "lidar_blob.h"

class CLidarDataFilterTracker : public CLidarDataFilterBase
{
public:
    CLidarDataFilterTracker();

    /*virtual*/void filter(const CLidarData *data_in, CLidarData *data_out) override;

    //! Objets decoupes lors du dernier appel a filter(), avec leur largeur angulaire apparente.
    //! Les objets rejetes par le facteur de forme y figurent, marques forme_douteuse : un objet
    //! est declasse, jamais supprime. Seuls les objets retenus sont ecrits dans data_out.
    const CLidarBlobs& blobs() const { return m_blobs; }

    //Paramétrage
    double m_d_dist_offset;           // mm
    int m_i_MAX_BLANK;                // discontinuité possible du blob
    int m_i_MIN_COUNT;                // taille minimum du blob
    int m_i_MAX_COUNT;                // taille maximum du blob
    double m_d_MAX_dist;              // zone max de détection
    double m_d_MIN_dist;              // zone min de détection
    double m_d_seuil_filtrage_dist;   // pour fusionner les doublons trops proches en distance
    int m_i_seuil_filtrage_angle;     // pour fusionner les doublons trops proches en angle
    static const int m_i_MAX_SAMPLES_THRESHOLD=5;
    double m_d_seuil_gradient;
    // Courbes enveloppes du facteur de forme : demi-diagonales [mm] du plus petit et du plus grand
    // mat balise admis. Un objet est retenu si sa distance mesuree tient entre R/tan(theta/2)
    // evalue pour ces deux rayons. Remplacent l'ancienne marge unique m_d_seuil_facteur_forme, qui
    // ne bornait que d'un cote : un mur vu de loin passait le test.
    double m_d_R_mini;                // mm (demi-diagonale du plus petit mat admis)
    double m_d_R_maxi;                // mm (demi-diagonale du plus grand mat admis)
    int m_dot_size;

private :
    CLidarBlobs m_blobs;
};

#endif // _LIDAR_DATA_FILTER_TRACKER_H_
