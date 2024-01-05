/*! \file Lidar_utils.h
    \brief Données partagées pour le LIDAR
*/

#ifndef _LIDAR_UTILS_H_
#define _LIDAR_UTILS_H_

class LidarUtils {
public :
    typedef enum {
        LIDAR_OK    = 0,
        LIDAR_DISCONNECTED,
        LIDAR_ERROR
    }eLidarStatus;

    static const int NBRE_MAX_OBSTACLES = 20;

    typedef struct {
        signed short angle;         // [degres signe / -180;+180]
        unsigned short distance;    // [mm]
    }tLidarObstacle;

    typedef tLidarObstacle tLidarObstacles [NBRE_MAX_OBSTACLES];

    static void copy_tab_obstacles(tLidarObstacle *src, tLidarObstacle *dest, int size=NBRE_MAX_OBSTACLES);
};

#endif
