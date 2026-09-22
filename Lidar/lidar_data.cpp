#include "lidar_data.h"
#include "Lidar_utils.h"

CLidarData::CLidarData()
{
    // Initialisation explicite de tous les membres : un CLidarData alloue dynamiquement (logique
    // robot de Simulia, recreee a chaque rechargement a chaud) contenait sinon des valeurs
    // indeterminees, en particulier m_measures_count, sur lequel les utilisateurs bouclent.
    m_timestamp = 0;
    m_start_angle = 0.;
    m_angle_step_resolution = 0.;
    m_measures_count = 0;
    for (int i=0; i<MAX_MEASURES_COUNT; i++) {
        m_dist_measures[i] = LidarUtils::NO_OBSTACLE;
    }
    m_scan_frequency = 0;
    m_scale_factor = 1.;
}
