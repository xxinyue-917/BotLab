#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    // printf("start position(search): %f, %f \n", start.x, start.y);
    // printf("goal position(search): %f, %f \n", goal.x, goal.y);

    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;

    // printf("start position(search_cell): %d, %d \n", startCell.x, startCell.y);
    // printf("goal position(search_cell): %d, %d \n", goalCell.x, goalCell.y);
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    std::vector<Node*> interior;
    PriorityQueue frontier;
    
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);

    // printf("The start point is %d, %d \n", startNode->cell.x, startNode->cell.y);
    // printf("The goal point is %d, %d \n", goalNode->cell.x, goalNode->cell.y);
    // printf("distance: %f", distances);

    // Check if start/goal already in the barrier
    float startDistance = distances(startCell.x, startCell.y);
    float goalDistance = distances(goalCell.x, goalCell.y);
    // printf("startDist: %f\n", startDistance);
    // printf("goalDist: %f\n", goalDistance);

    // if (startDistance < 1.01 * params.minDistanceToObstacle || goalDistance < 1.01 * params.minDistanceToObstacle) {
    //     printf("[A*] Start or goal is too close to obstacle\n");
    //     mbot_lcm_msgs::path2D_t path;
    //     path.utime = start.utime;
    //     path.path_length = 0;
    //     return path;
    // }

    int width = distances.widthInCells();
    int height = distances.heightInCells();
    if (goalCell.x < 0 || goalCell.x >= width || goalCell.y < 0 || goalCell.y >= height) {
        printf("[A*] Goal is outside the map boundaries\n");
        // Handle the invalid goal appropriately
        mbot_lcm_msgs::path2D_t path;
        path.utime = start.utime;
        path.path_length = 0;
        return path;
    }

    startNode->g_cost = 0;
    startNode->h_cost = h_cost(startNode, goalNode, distances);

    frontier.push(startNode);

    while (!frontier.empty())
{
    // Sort the node with smallest cost and extract it as currentNode
    // std::sort(frontier.begin(), frontier.end(), [](Node* n1, Node* n2) {
    //     return n1->f_cost() < n2->f_cost();
    // });
    // printf("Expand\n");
    Node* currentNode = frontier.pop();

    // Check if reached the goal
    if (*currentNode == *goalNode)
    {
        found_path = true;
        goalNode = currentNode;
        break;
    }

    // Append the node to path
    // frontier.erase(frontier.begin());
    interior.push_back(currentNode);

    // Expand the node
    auto children = expand_node(currentNode, distances, params);

    // Travser all the children
    for (auto& child : children)
    {
        // Check if the child already inside interior
        if (is_in_list(child, interior)){
            continue;   // skip the rest of code
        }
        
        // Calculate g_cost of this child
        double tentative_g_cost = currentNode->g_cost + g_cost(currentNode, child, distances, params);

        // Check if the child already in the frontier
        Node* existingNode = frontier.get_member(child);

        // Delete child with higher g_cost
        if (existingNode != NULL)
        {
            if (tentative_g_cost >= existingNode->g_cost)
            {
                continue;   // skip
            }
            else
            {
                // Update child's cost
                existingNode->g_cost = tentative_g_cost;
                existingNode->parent = currentNode;
            }
        }
        else
        {
            child->g_cost = tentative_g_cost;
            child->h_cost = h_cost(child, goalNode, distances);
            child->parent = currentNode;
            frontier.push(child);
        }
    }
}

    // Extract the path
    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;

    if (found_path)
    {
        auto nodePath = extract_node_path(goalNode, startNode);
        auto new_path = prune_node_path(nodePath);
        path.path = extract_pose_path(new_path, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }

    else printf("[A*] Didn't find a path\n");
    path.path_length = path.path.size();
    // for (auto node : frontier) {
    //     delete node;
    // }
    // // Clear interior
    // for (auto node : interior) {
    //     delete node;
    // }
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    ////////////////// TODO: Implement your heuristic //////////////////////////
    int dx = goal->cell.x - from->cell.x;
    int dy = goal->cell.y - from->cell.y;
    return sqrt(dx * dx + dy * dy);
}
double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    int dx = abs(to->cell.x - from->cell.x);
    int dy = abs(to->cell.y - from->cell.y);

    if (dx == 1 && dy == 1)
        g_cost = sqrt(2);
    else if ((dx == 1 && dy == 0) || (dx == 0 && dy == 1))
        g_cost = 1.0;
    else
        g_cost = 1.0E16;    // Add penalty to obstacle

    float distanceToObstacle = distances(to->cell.x, to->cell.y);
    if (distanceToObstacle < params.maxDistanceWithCost)
        g_cost += pow(params.maxDistanceWithCost - distanceToObstacle, params.distanceCostExponent);
    else if (distanceToObstacle < 1.5 * params.minDistanceToObstacle) {
        return 1.0E16;
    }
        
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> children;
    ////////////////// TODO: Implement your expand node algorithm //////////////////////////
    // Difine 4 possible moving directions
    std::vector<std::pair<int, int>> moves = {
        {0, -1},  // up
        {0, 1},   // down
        {-1, 0},  // left
        {1, 0},   // right
        {-1, -1}, // upleft
        {1, -1},  // upright
        {-1, 1},  // downleft
        {1, 1}    // downright
    };

    for (auto move : moves){
        int newX = node->cell.x + move.first;
        int newY = node->cell.y + move.second;

        if (newX >= 0 && newX < distances.widthInCells() && newY >= 0 && newY < distances.heightInCells()) {
            // Only include children have enough distance between obstacle
            float distance = distances(newX, newY);
            if (distance > 1.01 * params.minDistanceToObstacle){
                    Node* child = new Node(newX, newY);
                    children.push_back(child);
            }
        }
    }

    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    ////////////////// TODO: Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector
    Node* current = goal_node;
    while (current != NULL)
    {
        path.push_back(current);
        current = current->parent;
    }

    // Reverse path
    std::reverse(path.begin(), path.end());
    return path;
}
// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller
    if (nodes.empty())
        return path;

    // From grid position to global position
    for (auto& node : nodes)
    {
        Point<double> global_pos = grid_position_to_global_position(Point<double>(node->cell.x, node->cell.y), distances);

        mbot_lcm_msgs::pose2D_t pose;
        pose.x = global_pos.x;
        pose.y = global_pos.y;

        if (!path.empty()){
            pose.theta = std::atan2(pose.y - path[-1].y, pose.x - path[-1].x);
        }
        pose.theta = 0;

        path.push_back(pose);
    }
    
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath)
{
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line

    std::vector<Node*> newPath;
    newPath.push_back(nodePath.front());

    for (size_t i = 1; i < nodePath.size() - 1; ++i)
    {
        Node* prev = newPath.back();
        Node* curr = nodePath[i];
        Node* next = nodePath[i + 1];

        int dx1 = curr->cell.x - prev->cell.x;
        int dy1 = curr->cell.y - prev->cell.y;
        int dx2 = next->cell.x - curr->cell.x;
        int dy2 = next->cell.y - curr->cell.y;

        if (dx1 * dy2 != dy1 * dx2) // Check if the direction changes
        {
            newPath.push_back(curr);
        }
    }

    newPath.push_back(nodePath.back());
    return newPath;
}

std::vector<mbot_lcm_msgs::pose2D_t> prune_pose_path(std::vector<mbot_lcm_msgs::pose2D_t> path)
{
    std::vector<mbot_lcm_msgs::pose2D_t> newPath;
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    for (auto &&pose : path)
    {
        if (newPath.empty())
        {
            newPath.push_back(pose);
        }
        else
        {
            mbot_lcm_msgs::pose2D_t& lastPose = newPath.back();
            if (lastPose.theta == pose.theta)
            {
                continue;
            }
            newPath.push_back(pose);
        }
    }
    return newPath;

}
