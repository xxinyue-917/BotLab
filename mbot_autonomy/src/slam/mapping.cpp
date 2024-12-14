#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
#include <thread>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
    : kMaxLaserDistance_(maxLaserDistance), kHitOdds_(hitOdds), kMissOdds_(missOdds), initialized_(false)
{
}

void Mapping::updateMap(const mbot_lcm_msgs::lidar_t &scan,
                        const mbot_lcm_msgs::pose2D_t &pose,
                        OccupancyGrid &map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.

    // For the laser terminate -- It is just like for ray in movingScan in Python
    for(auto& ray : movingScan){
        scoreEndpoint(ray, map);
    }

    // For the laser can go through
    for(auto& ray : movingScan){
        scoreRay(ray, map);
    }

    initialized_ = true;
    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t &ray, OccupancyGrid &map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits
    if(ray.range < kMaxLaserDistance_){
        Point<int> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x;
        rayCell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y;

        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
    
}

void Mapping::scoreRay(const adjusted_ray_t &ray, OccupancyGrid &map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through
    if(ray.range < kMaxLaserDistance_){
        std::vector<Point<int>> points;
        points = bresenham(ray, map);
        // std::cout << "Points have been recorded";

        for(auto& point : points){
            if(map.isCellInGrid(point.x, point.y)){
                decreaseCellOdds(point.x, point.y, map);
            }
        }
    }
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t &ray, const OccupancyGrid &map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    std::vector<Point<int>> points;
    Point<int> rayStart = global_position_to_grid_position(ray.origin, map);
    Point<int> rayCell;

    rayCell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x;
    rayCell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y;

    // Calculate the distance between start and end
    int dx = rayCell.x - rayStart.x;
    int dy = rayCell.y - rayStart.y;

    // Calculate the sign of end point - return 1 for positive, -1 for negative
    int sx = (dx > 0) ? 1 : (dx < 0 ? -1 : 0);
    int sy = (dy > 0) ? 1 : (dy < 0 ? -1 : 0);

    // Add absolute value on dx and dy
    dx = std::abs(dx);
    dy = std::abs(dy);

    int err = dx - dy;
    int x = rayStart.x;
    int y = rayStart.y;
    points.push_back(Point<int>(x, y));
    // std::cout << "sx: " << sx << ", sy: " << sy << std::endl;
    while(x != rayCell.x || y != rayCell.y){
        int e2 = err * 2;
        if(e2 > -dy){
            err -= dy;
            x += sx;
        }
        if(e2 <= dx){
            err += dx;
            y += sy;
        }
        points.push_back(Point<int>(x, y));
        // std::cout << "x: " << x << ", y: " << y << ", err: " << err << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return points; // Placeholder
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t &ray, const OccupancyGrid &map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray.
    
    return {}; // Placeholder
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid &map)
{
    /// TODO: Increase the odds of the cell at (x,y)
    if(!initialized_){
        //do nothing
    }
    else if(127 - map(x,y) > kHitOdds_){
        map(x,y) += kHitOdds_;
    }
    else{
        map(x,y) = 127;
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid &map)
{
    /// TODO: Decrease the odds of the cell at (x,y)
    if(!initialized_){
        //do nothing
    }
    else if(map(x,y) + 128 > kMissOdds_){
        map(x,y) -= kMissOdds_;
    }
    else{
        map(x,y) = -128;
    }
}
