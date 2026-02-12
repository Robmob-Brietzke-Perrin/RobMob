#include "robmob_auto_explo/explo_helper.hpp"
#include <algorithm>

std::optional<double> ExploHelper::getBestDirection(
    const LaserScan::SharedPtr scan,
    const OccupancyGrid::SharedPtr map,
    double robot_x, double robot_y, double robot_yaw,
    double projection_dist,
    double last_angle) 
{
    int num_sectors = 12; // FIXME: a tune (nb segmentation) -> fine or reliable ?
    int points_per_sector = scan->ranges.size() / num_sectors;
    double best_score = -1.0;
    std::optional<double> best_angle;

    for (int i = 0; i < num_sectors; ++i) {
        // 1ère mesure de score -> distance moyenne donnée par le lidar
        double avg_dist = getAvgDistInSector(scan, i * points_per_sector, (i + 1) * points_per_sector);
        double score = avg_dist;

        // On ignore les secteurs trop proche
        if (avg_dist < 0.8) continue; // FIXME: 0.2 peut-être idéaliste, à tune

        // On extrait l'angle = la direction potentielle d'exploration
        double sector_angle = scan->angle_min + (i * points_per_sector + points_per_sector / 2.0) * scan->angle_increment;

        // On adapte le score selon la complétion de la map = curiosité vers points inconnus.
        // TODO: beware projection dist est crucial, c'est la dist à laquelle on regarde -> compromis finnesse/curiosité
        double target_x = robot_x + projection_dist * std::cos(robot_yaw + sector_angle);
        double target_y = robot_y + projection_dist * std::sin(robot_yaw + sector_angle);
        if (isPointUnknown(map, target_x, target_y)) {
            score *= 3.0; // augmentation forte du score vers l'inconnu
        }

        // Encore une adaptation pour ajouter de l'inertie (on ne veut pas d'un robot trop indécis)
        double diff = std::abs(sector_angle - last_angle);
        if (diff < 0.4) score *= 1.5;

        if (score > best_score) {
            best_score = score;
            best_angle = sector_angle;
        }
    }

    return best_angle;
}

double ExploHelper::getAvgDistInSector(const LaserScan::SharedPtr scan, int start_idx, int end_idx) {
    double sum = 0;
    int count = 0;
    // Pour chaque ray, ajouter la distance et count++
    for (int i = start_idx; i < end_idx; ++i) {
        if (std::isfinite(scan->ranges[i])) {
            sum += scan->ranges[i];
            count++;
        }
    }
    return (count > 0) ? (sum / count) : 0.0;
}

// FIXME: est-ce necessaire d'essayer d'avoir plus de robustesse aux NaN et !isfinite comme ici? les deux à tester, mais de toute façon je crois qu'en lab on n'est pas dans les limites...
// double ExploHelper::getAvgDistInSector(const LaserScan::SharedPtr scan, int start_idx, int end_idx) {
//     double sum = 0;
//     int count = 0;

//     for (int i = start_idx; i < end_idx; ++i) {
//         double r = scan->ranges[i];

//         if (std::isnan(r) || r < scan->range_min) {
//             continue; 
//         }

//         if (!std::isfinite(r) || r > scan->range_max) {
//             // On sature à la distance maximale connue
//             sum += scan->range_max;
//         } else {
//             sum += r;
//         }
//         count++;
//     }

//     // Si tout est vide ou bruité, on peut retourner range_max ?
//     return (count > 0) ? (sum / count) : scan->range_max;
// }

bool ExploHelper::isPointUnknown(const OccupancyGrid::SharedPtr map, double x, double y) {
    int gx = std::floor((x - map->info.origin.position.x) / map->info.resolution);
    int gy = std::floor((y - map->info.origin.position.y) / map->info.resolution);

    if (gx < 0 || gx >= (int)map->info.width || gy < 0 || gy >= (int)map->info.height) return true;
    
    return map->data[gy * map->info.width + gx] == -1;
}