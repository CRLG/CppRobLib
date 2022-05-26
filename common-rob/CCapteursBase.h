/*! \file CCapteursBase.h
    \brief Classe de base pour la virtualisation des capteurs
*/

#ifndef _CAPTEURS_BASE_H_
#define _CAPTEURS_BASE_H_


// -----------------------------
//! Classe de base pour les capteurs (commun robot réel et simulation)
//! Classe à dériver pour un robot donné ou la simulation pour faire le lien avec le hardware
//! Mettre une valeur par défaut pour chaque méthode si le capteur n'est pas utilisé sur le robot
class CCapteursBase
{
public :
    CCapteursBase();
    virtual ~CCapteursBase();

    //! Initialisation des capteurs
    virtual void Init(void);
    //! Traitement des capteurs (aqcuisition, filtrage)
    virtual void Traitement(void);

    // ____________________________________________
    // Liste tous les capteurs possibles (pour tous les robots)
    virtual bool getTirette();

    virtual bool getContactRecalageAVG();
    virtual bool getContactRecalageAVD();
    virtual bool getContactRecalageARG();
    virtual bool getContactRecalageARD();

    virtual bool getAscenseurButeeHaute();
    virtual bool getAscenseurButeeBasse();

    virtual float getCapteurPressionKmar();
};
#endif


