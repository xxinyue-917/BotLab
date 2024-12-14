#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
#include <planning/obstacle_distance_grid.hpp>
// #include <unordered_set>
// #include <queue>
SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(20),
	ray_stride_(7),
	max_ray_range_(1000),
    search_range(10),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}

void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    bfs_offsets_.clear();
    for(int dx = -search_range; dx <= search_range; ++dx) {
        for(int dy = -search_range; dy <= search_range; ++dy) {
            if(dx != 0 || dy != 0) {
                bfs_offsets_.emplace_back(dx, dy);
            }
        }
    }
    Point<int> center(0, 0);
    sort(bfs_offsets_.begin(), bfs_offsets_.end(), [&](const Point<int> &a, const Point<int> &b) {
        return distance_between_points(a, center) <= distance_between_points(b, center);
    });
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    /// TODO: Compute the likelihood of the given particle using the provided laser scan and map.
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;

    for(auto& ray : movingScan){
        // Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta),
        //                        ray.origin.y + ray.range * std::sin(ray.theta));
        // auto rayEnd = global_position_to_grid_position(endpoint, map);
        // if(map.logOdds(rayEnd.x, rayEnd.y) > 0.0){
        //     scanScore += map.logOdds(rayEnd.x, rayEnd.y);
        // }
        scanScore += scoreRay(ray, map);
    }

    return scanScore;
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Compute a score for a given ray based on its end point and the map. 
    // Consider the offset from the nearest occupied cell.  
    Point<float> end_Point = getRayEndPointOnMap(ray, map);
    Point<int> endPoint = global_position_to_grid_cell(end_Point, map);
    Point<int> nearestPoint = gridBFS(endPoint, map);
    int dx = endPoint.x - nearestPoint.x;
    int dy = endPoint.y - nearestPoint.y;

    double distance = std::sqrt(dx * dx + dy * dy);

    return NormalPdf(distance) * offset_quality_weight;
}

double SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point. 
    if (map.logOdds(end_point.x, end_point.y) > occupancy_threshold_) {
        return end_point;
    }
    for (auto & offset : bfs_offsets_) {
        auto next_cell = end_point + offset;
        if (map.logOdds(next_cell.x, next_cell.y) > occupancy_threshold_) {
            return next_cell;
        }
    }

    return Point<int>(end_point.x + 20, end_point.y + 20);
}

Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Calculate the end point of a given ray on the map 
    
    return Point<float>(ray.origin.x + ray.range * std::cos(ray.theta),
                        ray.origin.y + ray.range * std::sin(ray.theta));
}
