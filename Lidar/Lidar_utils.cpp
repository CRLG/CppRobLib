/*! \file Lidar_utils.cpp
 *
    \brief Données partagées pour le LIDAR
*/

#include "Lidar_utils.h"
#include "lidar_data.h"

void LidarUtils::copy_tab_obstacles(tLidarObstacle *src, tLidarObstacle *dest, int size)
{
    for (int i=0; i<size; i++) {
        dest[i].angle = src[i].angle;
        dest[i].distance = src[i].distance;
    }
}


// _____________________________________________________________________
/*!
 * \brief Met en forme la structure de donnes d'obstacles a partir des donnees LIDAR
 * \param data les donnees filtrees
 * \return la structure de donnees d'obstacle
*  \return le statut du LIDAR
 */
int LidarUtils::lidar_data_to_obstacles(const CLidarData *in_data, tLidarObstacles out_obstacles)
{
    if (!in_data) return LIDAR_ERROR;
    if (!out_obstacles) return LIDAR_ERROR;

    for (int i=0; i<NBRE_MAX_OBSTACLES; i++) {
         out_obstacles[i].angle = 0;
         out_obstacles[i].distance = NO_OBSTACLE;
     }

     //TODO mettre la limite max d'objets filtrés dans lidarutils
     int tempObstacles[2][NBRE_MAX_OBSTACLES_FILTRES];
     int nbObstaclesFiltres=0;
     for (int i=0; i< (in_data->m_measures_count);i++)
     {
         if(in_data->m_dist_measures[i]!=NO_OBSTACLE)
         {
             //il y a trop d'objets filtrés ce n'est pas normal, on arrête le traitement et on le met en erreur
             if(nbObstaclesFiltres>=NBRE_MAX_OBSTACLES_FILTRES)
                 return LIDAR_ERROR;

             tempObstacles[0][nbObstaclesFiltres]=in_data->m_dist_measures[i];
             tempObstacles[1][nbObstaclesFiltres]=in_data->m_start_angle + i*in_data->m_angle_step_resolution;
             nbObstaclesFiltres++;
         }
     }
     int idx=nbObstaclesFiltres;
     while(idx < NBRE_MAX_OBSTACLES_FILTRES)
     {
         tempObstacles[0][idx]=NO_OBSTACLE;
         tempObstacles[1][idx]=0;
         idx++;
     }

     //à trier
     int minD=0;
     int minPhi=0;
     int minIdx=0;
     for(int i=0; i<(NBRE_MAX_OBSTACLES_FILTRES-1);i++)
     {
         //init min
         minD=tempObstacles[0][i];
         minPhi=tempObstacles[1][i];
         minIdx=i;

         //recherche du min dans le reste du tableau
         for(int j=i+1; j<NBRE_MAX_OBSTACLES_FILTRES ;j++)
         {
             if((tempObstacles[0][j])<minD)
             {
                minD=tempObstacles[0][j];
                minPhi=tempObstacles[1][j];
                minIdx=j;
             }
         }

         //si il y a un minimum dans le reste du tableau, on permute avec l'élement en cours
         if(minIdx!=i)
         {
             int tempMinD=tempObstacles[0][i];
             int tempMinPhi=tempObstacles[1][i];
             tempObstacles[0][i]=tempObstacles[0][minIdx];
             tempObstacles[1][i]=tempObstacles[1][minIdx];
             tempObstacles[0][minIdx]=tempMinD;
             tempObstacles[1][minIdx]=tempMinPhi;
         }
     }

     //on rempli le tableau d'obstacle à hauteur du nombre max de point que l'on souhaite envoyer
     for (int i=0; i<NBRE_MAX_OBSTACLES; i++)
     {
         out_obstacles[i].distance = tempObstacles[0][i];
         out_obstacles[i].angle = tempObstacles[1][i];
     }

     // code laissé pour l'exemple: renseigne une valeur pipo pour le premier obstacle (juste pour tester la communication)
     /*int index=45+60;
     int angle = in_data->m_start_angle + index*in_data->m_angle_step_resolution;
     int distance = in_data->m_dist_measures[index];
     out_obstacles[0].angle = angle;
     out_obstacles[0].distance = distance;*/

     //on est allé au bout du traitement le statut est OK
     return LIDAR_OK;
}
