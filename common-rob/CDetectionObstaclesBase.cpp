/*! \file CDetectionObstaclesBase.cpp
    \brief Classe de base de détection des obstacles
*/
#include "CDetectionObstaclesBase.h"
#include "CGlobale.h"
#include <math.h>

//___________________________________________________________________________
/*!
   \brief Constructeur

   \param --
   \return --
*/
CDetectionObstaclesBase::CDetectionObstaclesBase()
    : m_seuil_detection_obstacle(35)
{
    Init();
}

//___________________________________________________________________________
/*!
   \brief Destructeur

   \param --
   \return --
*/
CDetectionObstaclesBase::~CDetectionObstaclesBase()
{
}

//___________________________________________________________________________
void CDetectionObstaclesBase::Init()
{
    inhibeDetectionAV(false);
    inhibeDetectionAR(false);
    inhibeDetectionVitesseNulle(true);
}

//___________________________________________________________________________
void CDetectionObstaclesBase::setSeuilDetectionObstacle(float seuil_cm)
{
    m_seuil_detection_obstacle = seuil_cm;
}

//___________________________________________________________________________
void CDetectionObstaclesBase::inhibeDetectionAVG(bool state)
{
    m_inhibe_detection_AVG = state;
}

void CDetectionObstaclesBase::inhibeDetectionAVD(bool state)
{
    m_inhibe_detection_AVD = state;
}

void CDetectionObstaclesBase::inhibeDetectionARG(bool state)
{
    m_inhibe_detection_ARG = state;
}

void CDetectionObstaclesBase::inhibeDetectionARD(bool state)
{
    m_inhibe_detection_ARD = state;
}

void CDetectionObstaclesBase::inhibeDetectionARGCentre(bool state)
{
    m_inhibe_detection_ARGCentre = state;
}

void CDetectionObstaclesBase::inhibeDetectionARDCentre(bool state)
{
    m_inhibe_detection_ARDCentre = state;
}

//___________________________________________________________________________
void CDetectionObstaclesBase::inhibeDetectionAV(bool state)
{
    inhibeDetectionAVG(state);
    inhibeDetectionAVD(state);
}

void CDetectionObstaclesBase::inhibeDetectionAR(bool state)
{
    inhibeDetectionARG(state);
    inhibeDetectionARD(state);
    inhibeDetectionARGCentre(state);
    inhibeDetectionARDCentre(state);
}

void CDetectionObstaclesBase::inhibeDetection(bool state)
{
    inhibeDetectionAV(state);
    inhibeDetectionAR(state);
}

//___________________________________________________________________________
void CDetectionObstaclesBase::inhibeDetectionVitesseNulle(bool state)
{
    m_inhibe_detection_a_l_arret = state;
}

float CDetectionObstaclesBase::modulo_pi(float angle)
{
    float ret=angle;
    while((ret>M_PI) || (ret<=-M_PI))
    {
        if (ret > M_PI){
            ret = ret - (2.0*M_PI);
        }
        else if (ret <= (-M_PI)){
            ret = ret + (2*M_PI);
        }
    }

    return(ret);
}

//___________________________________________________________________________
bool CDetectionObstaclesBase::isObstacleAVG()
{
    return ( (m_inhibe_detection_AVG == false) &&
             (Application.m_telemetres.getDistanceAVG()<=m_seuil_detection_obstacle) );
}

bool CDetectionObstaclesBase::isObstacleAVD()
{
    return ( (m_inhibe_detection_AVD == false) &&
             (Application.m_telemetres.getDistanceAVD()<=m_seuil_detection_obstacle) );
}

bool CDetectionObstaclesBase::isObstacleARG()
{
    return ( (m_inhibe_detection_ARG == false) &&
             (Application.m_telemetres.getDistanceARG()<=m_seuil_detection_obstacle) );
}

bool CDetectionObstaclesBase::isObstacleARD()
{
    return ( (m_inhibe_detection_ARD == false) &&
             (Application.m_telemetres.getDistanceARD()<=m_seuil_detection_obstacle) );
}

bool CDetectionObstaclesBase::isObstacleARGCentre()
{
    return ( (m_inhibe_detection_ARGCentre == false) &&
             (Application.m_telemetres.getDistanceARGCentre()<=m_seuil_detection_obstacle) );
}

bool CDetectionObstaclesBase::isObstacleARDCentre()
{
    return ( (m_inhibe_detection_ARDCentre == false) &&
             (Application.m_telemetres.getDistanceARDCentre()<=m_seuil_detection_obstacle) );
}

//___________________________________________________________________________
bool CDetectionObstaclesBase::isObstacleAV()
{
    return isObstacleAVG() || isObstacleAVD();
}

bool CDetectionObstaclesBase::isObstacleAR()
{
    return isObstacleARG() || isObstacleARD() ||
           isObstacleARGCentre() || isObstacleARDCentre();
}


//___________________________________________________________________________
bool CDetectionObstaclesBase::isObstacle()
{

    bool detection = false;
    float sens = Application.m_asservissement.getSensDeplacement();
    // TODO : à vérifier dans l'asserv pour le sens car l'info n'est pas "0" lorsque le robot est à l'arrêt

    // Version simple
    // TODO : à enrichir
    if (   ((sens>0) && isObstacleAV())     //marche avant
           || ((sens<0) && isObstacleAR()) )   //marche arrière
    {
        detection = true;
    }
    else if ( (sens == 0) && (!m_inhibe_detection_a_l_arret) ) { // vitesse nulle
        detection = isObstacleAV() || isObstacleAR();
    }
    return detection;
    /*
       //calibration
       //TODO: a remplacer par une carto
       int detection=0;
       //int VIOLET=m_iaSCI->get_vIOLET();
       //int JAUNE=m_iaSCI->get_jAUNE();
       int Couleur=m_couleur_equipe;// m_iaSCI->get_couleur();
       //bool b_forceObstacle=m_iaSCI->get_forceObstacle();
       float f_x=Application.m_asservissement.X_robot;
                                       float f_y=Application.m_asservissement.Y_robot;
                                       float f_teta=Application.m_asservissement.angle_robot;


       if (   ((sens>0)&&(m_obstacle_detecte_AVD||m_obstacle_detecte_AVG)) //marche avant
           || ((sens<0)&&(m_obstacle_detecte_ARD||m_obstacle_detecte_ARG)) ) //marche arrière
       {
           if(Couleur==JAUNE)
           {
               if ((f_y>-69)&&(f_y<8)&&(x>35))
                   detection=1;
               else
                   detection=0;
           }
           if(Couleur==VIOLET)
           {
                if ((f_y<69)&&(f_y>-8)&&(x>35))
                   detection=1;
               else
                   detection=0;
           }

           if (b_forceObstacle==true)
               detection=1;
       }
       else
         detection=0;

       //à réactiver si on veut détecter à l'arrêt
       /*if (sens==0)
           ((m_telemetre_AVD<=seuilDistance)||(m_telemetre_AVG<=seuilDistance)||(m_telemetre_ARD<=seuilDistance)||(m_telemetre_ARG<=seuilDistance))? detection=1:detection=0;
       //on retire les alertes si détection trop proche de la bordure*/
    /*if ((((x>100)||(x<-100))||((y>300)||(y<-30)))&&(detection==1))
           detection=0;*/

    //return 0;
    //return detection;
}

//___________________________________________________________________________
bool CDetectionObstaclesBase::isObstacleLIDAR(int distance, float phi, const int seuil_transverse)
{

    bool detection = false;
    float sens = Application.m_asservissement.getSensDeplacement();
    // TODO : à vérifier dans l'asserv pour le sens car l'info n'est pas "0" lorsque le robot est à l'arrêt

    // Version trigo
    if ((phi<(M_PI/2)) && (phi>(-M_PI/2)))
    {
        if((sens>0) && (fabs(sin(phi)*distance)<seuil_transverse))
            detection=true;
    }

    if((phi>(M_PI/2))&&(phi<=M_PI))
    {
            if((sens>0) && (fabs(sin(phi-M_PI)*distance)<seuil_transverse))
                detection=true;
    }

    if((phi<(-M_PI/2))&&(phi>=-M_PI))
    {
            if((sens>0) && (fabs(sin(phi+M_PI)*distance)<seuil_transverse))
                detection=true;
    }

    return detection;


       //à réactiver si on veut détecter à l'arrêt
       /*if (sens==0)
           ((m_telemetre_AVD<=seuilDistance)||(m_telemetre_AVG<=seuilDistance)||(m_telemetre_ARD<=seuilDistance)||(m_telemetre_ARG<=seuilDistance))? detection=1:detection=0;
       //on retire les alertes si détection trop proche de la bordure*/
}
