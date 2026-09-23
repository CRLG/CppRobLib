#include <math.h>
#include "CTacticalEvaluator.h"

static const float TTC_INFINI_S = 1000.f;

CTacticalEvaluator::CTacticalEvaluator()
{
    // Valeurs de depart, a calibrer sur table.
    m_R0_cm = 18.f;                 // rayon englobant de notre robot
    m_R1_cm = 18.f;                 // rayon estime du robot adverse
    m_seuil_prudence_cm = 80.f;
    m_seuil_ralenti_cm  = 50.f;
    m_seuil_arret_cm    = 30.f;
    m_hysteresis_cm     = 8.f;
    m_ttc_arret_s       = 1.5f;     // n'agit que sur les objets mobiles, cf. evaluer()
    m_confirmations_detente = 3;
    m_terrain_x_min_cm = 10.f;
    m_terrain_x_max_cm = 290.f;
    m_terrain_y_min_cm = 10.f;
    m_terrain_y_max_cm = 190.f;
    m_marge_bord_cm    = 20.f;
    init();
}

// _____________________________________________________________
void CTacticalEvaluator::init()
{
    m_menace.niveau = MENACE_LIBRE;
    m_menace.piste_retenue = false;
    m_menace.D_cm = 0.f;
    m_menace.phi_rad = 0.f;
    m_menace.ttc_s = TTC_INFINI_S;
    m_menace.dmin_cm = 0.f;
    m_menace.cote_libre = 0;
    m_menace.statique = false;
    m_menace.forme_douteuse = false;
    m_compteur_detente = 0;
}

// _____________________________________________________________
float CTacticalEvaluator::moduloPi(float angle_rad)
{
    while (angle_rad > (float)M_PI)  angle_rad -= 2.f*(float)M_PI;
    while (angle_rad < -(float)M_PI) angle_rad += 2.f*(float)M_PI;
    return angle_rad;
}

// _____________________________________________________________
bool CTacticalEvaluator::horsTerrain(float x_cm, float y_cm) const
{
    return (x_cm <= m_terrain_x_min_cm) || (x_cm >= m_terrain_x_max_cm)
        || (y_cm <= m_terrain_y_min_cm) || (y_cm >= m_terrain_y_max_cm);
}

// _____________________________________________________________
/*!
 * \brief Demi-ouverture du couloir a la distance D
 * Meme calcul que dans evaluer() : un objet a moins de cet angle du cap est dans le couloir.
 */
float CTacticalEvaluator::angleCritiqueRad(float D_cm) const
{
    if (D_cm < 0.1f) return (float)M_PI / 2.f;   // colle au robot : tout le demi-plan gene
    float rapport = (m_R0_cm + m_R1_cm) / D_cm;
    if (rapport > 1.f) rapport = 1.f;
    return asinf(rapport);
}

// _____________________________________________________________
/*!
 * \brief true si la pose reste dans le terrain, marge de bord comprise
 */
bool CTacticalEvaluator::poseManoeuvrable(float x_cm, float y_cm) const
{
    // La marge est le rayon englobant du robot, et non la marge de bord tactique (m_marge_bord_cm,
    // qui sert a choisir un cote d'esquive). La question posee ici est physique : le chassis
    // tiendrait-il a cette pose ? Prendre la marge tactique refuserait toute manoeuvre depuis la
    // zone de depart, qui est elle-meme adossee a une bordure.
    return (x_cm >= (m_terrain_x_min_cm + m_R0_cm))
        && (x_cm <= (m_terrain_x_max_cm - m_R0_cm))
        && (y_cm >= (m_terrain_y_min_cm + m_R0_cm))
        && (y_cm <= (m_terrain_y_max_cm - m_R0_cm));
}

// _____________________________________________________________
/*!
 * \brief Cote ou s'ecarter : l'oppose de la piste, sauf si le bord du terrain l'interdit
 */
signed char CTacticalEvaluator::choisirCoteLibre(float x_robot_cm, float y_robot_cm,
                                                 float cap_trajectoire_rad, float phi_piste_rad) const
{
    // On s'ecarte du cote oppose a la piste
    const signed char cote_naturel = (phi_piste_rad > 0.f) ? -1 : +1;
    const signed char cotes[2] = { cote_naturel, (signed char)(-cote_naturel) };

    for (int i=0; i<2; i++) {
        // point atteint en s'ecartant lateralement de la marge de bord
        const float cap_lateral = cap_trajectoire_rad + (float)cotes[i] * (float)M_PI / 2.f;
        const float x = x_robot_cm + m_marge_bord_cm * cosf(cap_lateral);
        const float y = y_robot_cm + m_marge_bord_cm * sinf(cap_lateral);
        if (!horsTerrain(x, y)) return cotes[i];
    }
    return 0;   // acule : aucun cote ne degage
}

// _____________________________________________________________
void CTacticalEvaluator::evaluer(const CObstacleTracker &suivi,
                                 float x_robot_cm, float y_robot_cm,
                                 float cap_trajectoire_rad, float vitesse_robot_cms)
{
    // vitesse de notre robot dans le repere terrain, le long de sa trajectoire
    const float vrx = vitesse_robot_cms * cosf(cap_trajectoire_rad);
    const float vry = vitesse_robot_cms * sinf(cap_trajectoire_rad);

    const tObstacleTrack *retenue = 0;
    float ttc_retenu = TTC_INFINI_S;
    float D_retenu = 0.f, phi_retenu = 0.f, dmin_retenu = 0.f;

    const tObstacleTrack *pistes = suivi.tracks();
    for (int i=0; i<CObstacleTracker::NBRE_MAX_TRACKS; i++) {
        if (!pistes[i].valide) continue;

        // Hors terrain : un point derriere une bordure n'est pas un obstacle. C'est aussi ce qui
        // evite de prendre les murs pour des adversaires quand le filtre en laisse passer un bout.
        if (horsTerrain(pistes[i].X_cm, pistes[i].Y_cm)) continue;

        const float dx = pistes[i].X_cm - x_robot_cm;
        const float dy = pistes[i].Y_cm - y_robot_cm;
        const float D = sqrtf(dx*dx + dy*dy);
        if (D < 0.1f) continue;
        const float phi = moduloPi(atan2f(dy, dx) - cap_trajectoire_rad);

        // Critere de trajectoire : |phi| <= asin((R0+R1)/D). Au-dela de cet angle critique,
        // l'objet ne peut pas se trouver dans le couloir du robot.
        float rapport = (m_R0_cm + m_R1_cm) / D;
        if (rapport > 1.f) rapport = 1.f;
        const float angle_critique = asinf(rapport);
        const bool dans_couloir = (fabsf(phi) <= angle_critique);

        // Point d'approche minimale, a vitesses constantes
        const float vx = pistes[i].Vx_cms - vrx;
        const float vy = pistes[i].Vy_cms - vry;
        const float v2 = vx*vx + vy*vy;
        float ttc = TTC_INFINI_S;
        float dmin = D;
        if (v2 > 1.f) {                       // au moins 1 cm/s de vitesse relative
            ttc = -(dx*vx + dy*vy) / v2;
            if (ttc < 0.f) {
                ttc = -1.f;                   // on s'eloigne deja : l'approche minimale est passee
                dmin = D;
            }
            else {
                const float ex = dx + vx*ttc;
                const float ey = dy + vy*ttc;
                dmin = sqrtf(ex*ex + ey*ey);
            }
        }

        // Une piste gene-t-elle ?
        //  - si rien ne bouge l'un par rapport a l'autre, seule la geometrie parle : la piste gene
        //    si elle est dans le couloir (cas du face-a-face immobile, celui du blocage) ;
        //  - sinon c'est l'approche minimale qui tranche : elle gene si les trajectoires se
        //    rapprochent a moins de R0+R1. Un adversaire qui croise en degageant ne gene pas, meme
        //    s'il est droit devant a cet instant ; un adversaire qui s'eloigne non plus.
        //  - dans tous les cas, ce qui est deja tres pres gene, quoi qu'il fasse.
        bool gene;
        if (v2 <= 1.f) gene = dans_couloir;
        else gene = ((ttc >= 0.f) && (dmin <= (m_R0_cm + m_R1_cm))) || (D <= m_seuil_arret_cm);
        if (!gene) continue;

        // La piste la plus menacante est celle de TTC minimal, et non la plus proche : un pied de
        // table a 30 cm sur le cote ne doit pas masquer l'adversaire a 60 cm droit devant.
        const float ttc_classement = (ttc < 0.f) ? TTC_INFINI_S : ttc;
        if ((retenue == 0) || (ttc_classement < ttc_retenu)) {
            retenue = &pistes[i];
            ttc_retenu = ttc_classement;
            D_retenu = D;
            phi_retenu = phi;
            dmin_retenu = dmin;
            m_menace.ttc_s = ttc;
        }
    }

    // ---- niveau brut. Les seuils sont elargis de l'hysteresis tant qu'on est deja au niveau
    //      correspondant : sinon une mesure qui oscille autour d'un seuil fait vibrer la machine
    //      a etats d'evitement.
    unsigned char niveau = MENACE_LIBRE;
    if (retenue) {
        const float marge_prudence = (m_menace.niveau >= MENACE_PRUDENCE) ? m_hysteresis_cm : 0.f;
        const float marge_ralenti  = (m_menace.niveau >= MENACE_RALENTI)  ? m_hysteresis_cm : 0.f;
        const float marge_arret    = (m_menace.niveau >= MENACE_ARRET)    ? m_hysteresis_cm : 0.f;

        if (D_retenu <= m_seuil_prudence_cm + marge_prudence) niveau = MENACE_PRUDENCE;
        if (D_retenu <= m_seuil_ralenti_cm  + marge_ralenti)  niveau = MENACE_RALENTI;
        if (D_retenu <= m_seuil_arret_cm    + marge_arret)    niveau = MENACE_ARRET;
        // Contact imminent avec un objet MOBILE : on s'arrete sans attendre que la distance
        // descende. Le critere est volontairement reserve aux objets mobiles : applique a un objet
        // immobile, il boucle sur lui-meme (on ralentit, donc le temps avant contact augmente, donc
        // on reaccelere) ; pour un objet immobile, c'est l'echelle des distances qui commande, et
        // elle est stable.
        if ((ttc_retenu >= 0.f) && (ttc_retenu <= m_ttc_arret_s) && !retenue->statique) {
            niveau = MENACE_ARRET;
        }
    }

    // ---- aggravation immediate, detente confirmee
    if (niveau >= m_menace.niveau) {
        m_menace.niveau = niveau;
        m_compteur_detente = 0;
    }
    else {
        if (m_compteur_detente < 255) m_compteur_detente++;
        if (m_compteur_detente >= m_confirmations_detente) {
            m_menace.niveau = niveau;
            m_compteur_detente = 0;
        }
    }

    // ---- publication de la piste retenue
    m_menace.piste_retenue = (retenue != 0);
    if (retenue) {
        m_menace.D_cm = D_retenu;
        m_menace.phi_rad = phi_retenu;
        m_menace.dmin_cm = dmin_retenu;
        m_menace.statique = retenue->statique;
        m_menace.forme_douteuse = retenue->forme_douteuse;
        m_menace.cote_libre = choisirCoteLibre(x_robot_cm, y_robot_cm, cap_trajectoire_rad, phi_retenu);
    }
    else {
        m_menace.ttc_s = TTC_INFINI_S;
        m_menace.dmin_cm = 0.f;
        m_menace.cote_libre = 0;
        m_menace.statique = false;
        m_menace.forme_douteuse = false;
    }
}
