/*! \file lidar_blob.h
    \brief Objets detectes par un filtre lidar (sortie "blobs")

    Un filtre lidar rend deux choses : le balayage filtre (CLidarData, interface historique) et la
    liste des objets qu'il a decoupes. La largeur angulaire apparente d'un objet, connue du filtre
    et jusqu'ici perdue a sa sortie, est ce qui permet de distinguer un mat balise d'un mur : elle
    est publiee ici, avec le verdict du facteur de forme.
*/
#ifndef _LIDAR_BLOB_H_
#define _LIDAR_BLOB_H_

typedef struct {
    float angle_deg;        //!< centroide de l'objet [degres], meme convention d'origine que le balayage
    float distance_mm;      //!< distance moyenne [mm], offset de compensation deja applique
    float largeur_deg;      //!< largeur angulaire apparente [degres]
    bool  forme_douteuse;   //!< true si le couple (largeur, distance) sort des courbes enveloppes
}tLidarBlob;

//! Liste des objets rendus par un filtre pour un tour de balayage
class CLidarBlobs
{
public :
    static const int NBRE_MAX_BLOBS = 20;

    CLidarBlobs() { clear(); }

    void clear() { m_count = 0; }

    //! Ajoute un objet ; rend false si la liste est pleine (l'objet est alors perdu, jamais ecrase)
    bool append(float angle_deg, float distance_mm, float largeur_deg, bool forme_douteuse)
    {
        if (m_count >= NBRE_MAX_BLOBS) return false;
        m_blobs[m_count].angle_deg      = angle_deg;
        m_blobs[m_count].distance_mm    = distance_mm;
        m_blobs[m_count].largeur_deg    = largeur_deg;
        m_blobs[m_count].forme_douteuse = forme_douteuse;
        m_count++;
        return true;
    }

    int         m_count;
    tLidarBlob  m_blobs[NBRE_MAX_BLOBS];
};

#endif // _LIDAR_BLOB_H_
