/*! \file CDetectionObstaclesBase.h
    \brief Classe de base de détection d'obstacles
*/

#ifndef _DETECTION_OBSTACES_BASE_H
#define _DETECTION_OBSTACES_BASE_H

//! Classe de base pour la gestion des télémètres
class CDetectionObstaclesBase
{
public :
    CDetectionObstaclesBase();
    ~CDetectionObstaclesBase();

    virtual void Init();

    virtual void setSeuilDetectionObstacle(float seuil_cm);

    virtual bool isObstacle();
    virtual bool isObstacleAVG();
    virtual bool isObstacleAVD();
    virtual bool isObstacleARG();
    virtual bool isObstacleARD();
    virtual bool isObstacleAV();
    virtual bool isObstacleAR();

    virtual void inhibeDetectionAVG(bool state);
    virtual void inhibeDetectionAVD(bool state);
    virtual void inhibeDetectionARG(bool state);
    virtual void inhibeDetectionARD(bool state);
    virtual void inhibeDetectionAV(bool state);
    virtual void inhibeDetectionAR(bool state);
    virtual void inhibeDetection(bool state);

    // Inhibe/Autorise la détection d'obstacle lorsque le robot ne bouge pas
    virtual void inhibeDetectionVitesseNulle(bool state);

protected :
    float m_seuil_detection_obstacle;   // [cm]
    bool m_inhibe_detection_AVG;
    bool m_inhibe_detection_AVD;
    bool m_inhibe_detection_ARG;
    bool m_inhibe_detection_ARD;

    bool m_inhibe_detection_a_l_arret;
};
#endif


