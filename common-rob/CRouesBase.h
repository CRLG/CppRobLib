/*! \file CRouesBase.h
    \brief Classe qui contient la gestion des roues motrices gauche et droite
*/

#ifndef _ROUES_BASE_H_
#define _ROUES_BASE_H_

// -----------------------------
//! Classe de gestion des options d'exécution passees en ligne de commande
class CRouesBase {
public :
    //! Memorise la commande des moteurs
    float m_cde_roue_G;
    float m_cde_roue_D;

    CRouesBase();
    virtual ~CRouesBase();

    virtual void AdapteCommandeMoteur_G(float vitesse) = 0;
    virtual void AdapteCommandeMoteur_D(float vitesse) = 0;
    virtual void AdapteCommandeMoteurs(float vit_G, float vit_D);

    virtual int getCodeurG(void) = 0;
    virtual int getCodeurD(void) = 0;

    virtual void resetCodeurs(void) = 0;
};


#endif


