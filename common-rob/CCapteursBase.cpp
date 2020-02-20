/*! \file CCapteursBase.cpp
	\brief Classe qui contient toute l'application
*/
#include "CCapteursBase.h"

//___________________________________________________________________________
CCapteursBase::CCapteursBase()
{
}
CCapteursBase::~CCapteursBase()
{
}

// ____________________________________________
void CCapteursBase::Init(void)
{
}

// ____________________________________________
void CCapteursBase::Traitement(void)
{
}

// ____________________________________________
// Liste tous les capteurs possibles (pour tous les robots)
bool CCapteursBase::getTirette()
{
    return false;
}

// ____________________________________________
bool CCapteursBase::getContactRecalageAVG()
{
    return false;
}
bool CCapteursBase::getContactRecalageAVD()
{
    return false;
}
bool CCapteursBase::getContactRecalageARG()
{
    return false;
}
bool CCapteursBase::getContactRecalageARD()
{
    return false;
}

// ____________________________________________
bool CCapteursBase::getAscenseurButeeHaute()
{
    return false;
}
bool CCapteursBase::getAscenseurButeeBasse()
{
    return false;
}

