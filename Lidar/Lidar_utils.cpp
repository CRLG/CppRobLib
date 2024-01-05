/*! \file Lidar_utils.cpp
 *
    \brief Données partagées pour le LIDAR
*/

#include "Lidar_utils.h"

void LidarUtils::copy_tab_obstacles(tLidarObstacle *src, tLidarObstacle *dest, int size)
{
    for (int i=0; i<size; i++) {
        dest[i].angle = src[i].angle;
        dest[i].distance = src[i].distance;
    }
}
