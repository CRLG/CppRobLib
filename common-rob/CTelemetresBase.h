/*! \file CTelemetresBase.h
    \brief Classe de base qui contient la gestion des 4 LED MBED
*/

#ifndef _TELEMETRES_BASE_H
#define _TELEMETRES_BASE_H

//! Classe de base pour la gestion des télémètres
class CTelemetresBase
{
public :
    CTelemetresBase();
    virtual ~CTelemetresBase();

    virtual float getDistanceAVG() = 0;
    virtual float getDistanceAVD() = 0;
    virtual float getDistanceARG() = 0;
    virtual float getDistanceARD() = 0;

};
#endif


