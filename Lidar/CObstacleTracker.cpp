#include <math.h>
#include "CObstacleTracker.h"

CObstacleTracker::CObstacleTracker()
{
    // Valeurs de depart, a calibrer sur enregistrements reels.
    // La porte d'association tolere, a 125 ms entre deux tours, un deplacement relatif de 1,2 m/s :
    // tres au-dessus de ce que font deux robots de coupe.
    m_porte_association_cm = 15.f;
    m_seuil_statique_cms   = 10.f;
    m_seuil_mobile_cms     = 15.f;
    m_scans_sans_mesure_max = 5;
    m_scans_confirmation    = 5;
    m_filtre_vitesse        = 0.6f;
    init();
}

// _____________________________________________________________
void CObstacleTracker::init()
{
    for (int i=0; i<NBRE_MAX_TRACKS; i++) {
        m_tracks[i].valide = false;
        m_tracks[i].X_cm = 0.f;
        m_tracks[i].Y_cm = 0.f;
        m_tracks[i].Vx_cms = 0.f;
        m_tracks[i].Vy_cms = 0.f;
        m_tracks[i].largeur_deg = 0.f;
        m_tracks[i].age_ms = 0;
        m_tracks[i].confirmations = 0;
        m_tracks[i].sans_mesure = 0;
        m_tracks[i].statique = false;
        m_tracks[i].forme_douteuse = false;
        m_compteur_statique[i] = 0;
        m_compteur_mobile[i] = 0;
        m_x_dernier_scan[i] = 0.f;
        m_y_dernier_scan[i] = 0.f;
        m_date_derniere_mesure[i] = 0;
    }
    m_date_dernier_scan_ms = 0;
    m_date_extrapolation_ms = 0;
    m_premier_scan = true;
}

// _____________________________________________________________
int CObstacleTracker::count() const
{
    int n = 0;
    for (int i=0; i<NBRE_MAX_TRACKS; i++) if (m_tracks[i].valide) n++;
    return n;
}

// _____________________________________________________________
const tObstacleTrack* CObstacleTracker::plusProche(float x_cm, float y_cm) const
{
    const tObstacleTrack *piste = 0;
    float distance2_min = 0.f;
    for (int i=0; i<NBRE_MAX_TRACKS; i++) {
        if (!m_tracks[i].valide) continue;
        const float dx = m_tracks[i].X_cm - x_cm;
        const float dy = m_tracks[i].Y_cm - y_cm;
        const float distance2 = dx*dx + dy*dy;
        if ((piste == 0) || (distance2 < distance2_min)) {
            piste = &m_tracks[i];
            distance2_min = distance2;
        }
    }
    return piste;
}

// _____________________________________________________________
int CObstacleTracker::chercherPisteLibre()
{
    for (int i=0; i<NBRE_MAX_TRACKS; i++) if (!m_tracks[i].valide) return i;
    return -1;      // liste pleine : la mesure est perdue, aucune piste suivie n'est sacrifiee
}

// _____________________________________________________________
void CObstacleTracker::vieillir(tObstacleTrack &piste, float dt_s)
{
    const unsigned long age = (unsigned long)piste.age_ms + (unsigned long)(dt_s * 1000.f);
    piste.age_ms = (age > 65000UL) ? 65000 : (unsigned short)age;
}

// _____________________________________________________________
void CObstacleTracker::nouveauScan(const CLidarBlobs &objets, float x_robot_cm, float y_robot_cm,
                                   float cap_robot_rad, unsigned long date_ms)
{
    // Toutes les pistes sont d'abord amenees a la date de ce tour : l'association se fait alors sur
    // la position attendue de chaque piste, et non sur celle du tour precedent.
    extrapoler(date_ms);

    // Duree depuis le tour precedent. Bornee : un ecart aberrant (reprise apres perte du lidar)
    // ferait diverger l'estimation de vitesse.
    float dt_s = 0.f;
    if (!m_premier_scan && (date_ms > m_date_dernier_scan_ms)) {
        dt_s = (float)(date_ms - m_date_dernier_scan_ms) / 1000.f;
        if (dt_s > 1.f) dt_s = 1.f;
    }
    m_date_dernier_scan_ms = date_ms;
    m_date_extrapolation_ms = date_ms;
    m_premier_scan = false;

    bool piste_nourrie[NBRE_MAX_TRACKS];
    for (int i=0; i<NBRE_MAX_TRACKS; i++) piste_nourrie[i] = false;

    // ---- association : chaque mesure nourrit la piste valide la plus proche, si elle est dans la porte
    for (int n=0; n<objets.m_count; n++) {
        // projection de la mesure dans le repere terrain
        const float distance_cm = objets.m_blobs[n].distance_mm / 10.f;
        const float cap_mesure = cap_robot_rad + objets.m_blobs[n].angle_deg * (float)M_PI / 180.f;
        const float x_mesure = x_robot_cm + distance_cm * cosf(cap_mesure);
        const float y_mesure = y_robot_cm + distance_cm * sinf(cap_mesure);

        int indice = -1;
        float distance2_min = m_porte_association_cm * m_porte_association_cm;
        for (int i=0; i<NBRE_MAX_TRACKS; i++) {
            if (!m_tracks[i].valide || piste_nourrie[i]) continue;
            const float dx = m_tracks[i].X_cm - x_mesure;
            const float dy = m_tracks[i].Y_cm - y_mesure;
            const float distance2 = dx*dx + dy*dy;
            if (distance2 <= distance2_min) {
                distance2_min = distance2;
                indice = i;
            }
        }

        if (indice < 0) {
            // aucune piste dans la porte : c'est un objet nouveau
            indice = chercherPisteLibre();
            if (indice < 0) continue;
            m_tracks[indice].valide = true;
            m_tracks[indice].X_cm = x_mesure;
            m_tracks[indice].Y_cm = y_mesure;
            m_tracks[indice].Vx_cms = 0.f;
            m_tracks[indice].Vy_cms = 0.f;
            m_tracks[indice].age_ms = 0;
            m_tracks[indice].confirmations = 1;
            m_tracks[indice].statique = false;   // tant qu'on ne l'a pas vu deux fois, on ne sait pas
            m_compteur_statique[indice] = 0;
            m_compteur_mobile[indice] = 0;
            m_x_dernier_scan[indice] = x_mesure;
            m_y_dernier_scan[indice] = y_mesure;
            m_date_derniere_mesure[indice] = date_ms;
        }
        else {
            // vitesse par difference filtree (premier ordre) : pas besoin d'un Kalman pour savoir
            // si l'objet bouge, dans quel sens et a peu pres a quelle allure
            // duree depuis la DERNIERE MESURE de cette piste, et non depuis le tour precedent :
            // une piste qui a saute un tour a parcouru deux fois plus de chemin
            float dt_piste_s = dt_s;
            if (date_ms > m_date_derniere_mesure[indice]) {
                dt_piste_s = (float)(date_ms - m_date_derniere_mesure[indice]) / 1000.f;
                if (dt_piste_s > 1.f) dt_piste_s = 1.f;
            }
            if (dt_piste_s > 0.f) {
                const float vx_mesuree = (x_mesure - m_x_dernier_scan[indice]) / dt_piste_s;
                const float vy_mesuree = (y_mesure - m_y_dernier_scan[indice]) / dt_piste_s;
                m_tracks[indice].Vx_cms = m_filtre_vitesse * m_tracks[indice].Vx_cms
                                        + (1.f - m_filtre_vitesse) * vx_mesuree;
                m_tracks[indice].Vy_cms = m_filtre_vitesse * m_tracks[indice].Vy_cms
                                        + (1.f - m_filtre_vitesse) * vy_mesuree;
            }
            m_tracks[indice].X_cm = x_mesure;
            m_tracks[indice].Y_cm = y_mesure;
            if (m_tracks[indice].confirmations < 255) m_tracks[indice].confirmations++;
        }

        m_x_dernier_scan[indice] = x_mesure;
        m_y_dernier_scan[indice] = y_mesure;
        m_date_derniere_mesure[indice] = date_ms;
        m_tracks[indice].largeur_deg = objets.m_blobs[n].largeur_deg;
        m_tracks[indice].forme_douteuse = objets.m_blobs[n].forme_douteuse;
        m_tracks[indice].sans_mesure = 0;
        piste_nourrie[indice] = true;
    }

    // ---- pistes sans mesure a ce tour : extrapolees, puis perdues au bout de N tours
    for (int i=0; i<NBRE_MAX_TRACKS; i++) {
        if (!m_tracks[i].valide || piste_nourrie[i]) continue;
        // la position a deja ete avancee par l'extrapolation en tete de fonction ; la piste garde
        // sa derniere position mesuree comme reference de vitesse, pour ne pas se figer si elle
        // reapparait au tour suivant
        if (m_tracks[i].sans_mesure < 255) m_tracks[i].sans_mesure++;
        m_tracks[i].confirmations = 0;
        if (m_tracks[i].sans_mesure > m_scans_sans_mesure_max) {
            m_tracks[i].valide = false;
            m_compteur_statique[i] = 0;
            m_compteur_mobile[i] = 0;
        }
    }

    // ---- classement statique / mobile, avec hysteresis : un seul tour bruite ne doit pas
    //      faire basculer une piste d'un etat a l'autre
    for (int i=0; i<NBRE_MAX_TRACKS; i++) {
        if (!m_tracks[i].valide) continue;
        const float vitesse = sqrtf(m_tracks[i].Vx_cms * m_tracks[i].Vx_cms
                                  + m_tracks[i].Vy_cms * m_tracks[i].Vy_cms);
        if (vitesse < m_seuil_statique_cms) {
            if (m_compteur_statique[i] < 255) m_compteur_statique[i]++;
            m_compteur_mobile[i] = 0;
            if (m_compteur_statique[i] >= m_scans_confirmation) m_tracks[i].statique = true;
        }
        else if (vitesse > m_seuil_mobile_cms) {
            if (m_compteur_mobile[i] < 255) m_compteur_mobile[i]++;
            m_compteur_statique[i] = 0;
            if (m_compteur_mobile[i] >= 2) m_tracks[i].statique = false;
        }
        // entre les deux seuils : on ne change rien (zone morte de l'hysteresis)
    }
}

// _____________________________________________________________
void CObstacleTracker::extrapoler(unsigned long date_ms)
{
    if (m_premier_scan) return;
    if (date_ms <= m_date_extrapolation_ms) return;

    float dt_s = (float)(date_ms - m_date_extrapolation_ms) / 1000.f;
    if (dt_s > 1.f) dt_s = 1.f;
    m_date_extrapolation_ms = date_ms;

    for (int i=0; i<NBRE_MAX_TRACKS; i++) {
        if (!m_tracks[i].valide) continue;
        m_tracks[i].X_cm += m_tracks[i].Vx_cms * dt_s;
        m_tracks[i].Y_cm += m_tracks[i].Vy_cms * dt_s;
        vieillir(m_tracks[i], dt_s);
    }
}
