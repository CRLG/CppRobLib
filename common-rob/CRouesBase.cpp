/*! \file CRouesBase.cpp
    \brief Classe
*/
#include "CRouesBase.h"

//___________________________________________________________________________
/*!
   \brief Constructeur

   \param --
   \return --
*/
CRouesBase::CRouesBase() 
{
}

//___________________________________________________________________________
/*!
   \brief Destructeur

   \param --
   \return --
*/
CRouesBase::~CRouesBase() 
{
}

//___________________________________________________________________________
/*!
   \brief Applique la puissance aux 2 moteurs

   \param vit_G la vitesse signee en pourcentage [-100%;+100] pour le moteur gauche
   \param vit_D la vitesse signee en pourcentage [-100%;+100] pour le moteur droit
   \return --
*/
void CRouesBase::AdapteCommandeMoteurs(float vit_G, float vit_D)
{
    AdapteCommandeMoteur_G(vit_G);
    AdapteCommandeMoteur_D(vit_D);
}
