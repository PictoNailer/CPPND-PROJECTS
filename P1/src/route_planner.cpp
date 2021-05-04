#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TD-2:: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates. Store the nodes you find in the RoutePlanner's start_node and end_node attributes.                               
    RoutePlanner::start_node = &m_Model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &m_Model.FindClosestNode(end_x, end_y);

    std::cout << "Start node: \n";
    std::cout << start_node->x << ", " << start_node->y << "\n";

    std::cout << "End node: \n";
    std::cout << end_node->x << ", " << end_node->y << "\n";
    // EOTD-2:: DONE. 
}

// TD-3:: Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Calculate distance between current node and goal & return.
    return node->distance(*end_node);
}
// EOTD-3:: DONE. 

// TD-4:: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();

    // For each node in current_node.neighbors, set the parent, the h_value, the g_value.
    for (RouteModel::Node *node : current_node->neighbors)
    {
        node->parent = current_node;
        node->h_value = RoutePlanner::CalculateHValue(node);
        node->g_value = current_node->g_value + node->distance(*current_node); 

        open_list.push_back(node);
        node->visited = true;
    }
}
// EOTD-4:: DONE. 

// TD-5:: Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {

    // Sort the open_list according to the sum of the h value and g value.
    std::sort(open_list.begin(), open_list.end(), RoutePlanner::CompareFValue);

    // Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node* opt_node = open_list.back();

    // Remove that node from the open_list.
    open_list.pop_back();

    // Return the pointer.
    return opt_node; 
}
// Compare function for sorting.
bool RoutePlanner::CompareFValue(RouteModel::Node *nodeA, RouteModel::Node *nodeB)
{
    return ((nodeA->h_value + nodeA->g_value) > (nodeB->h_value + nodeB->g_value));
}
// EOTD-5:: DONE. 


// TD-6:: Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    // Create path_found vector.
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // From end_node to start_node.
    while (current_node != start_node)
    {
        // For each node in the chain, add the distance from the node to its parent to the distance variable.
        distance = distance + current_node->distance(*(current_node->parent)); // *** Attention to * and (*). ***

        // Add current node to path. 
        path_found.push_back(*current_node);

        // Point current_node pointer to father. 
        current_node = current_node->parent;
    }

    // Include the start node. 
    path_found.push_back(*current_node); // *** Fail test if list is incomplete. ***

    // The returned vector should be in the correct order: the start node should be the first element of the vector, the end node should be the last element.
    std::reverse(path_found.begin(), path_found.end());

    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale(); 
    return path_found;
}
// EOTD-6:: DONE. 

// TD-7:: The A* Search algorithm.
void RoutePlanner::AStarSearch() {

    // Initialize the current_node pointer, point to start_node. 
    RouteModel::Node *current_node = start_node;
    start_node->visited = true; // *** Stuck if forget this. ***

    while(current_node != end_node)
    {
        // Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
        AddNeighbors(current_node);

        // Use the NextNode() method to sort the open_list and return the next node.
        RouteModel::Node *next_node = NextNode();
        current_node = next_node;
    }

    open_list.push_back(current_node);

    // Use the ConstructFinalPath method to return the final path that was found. Store the final path in the m_Model.path attribute before the method exits.
    m_Model.path = ConstructFinalPath(current_node);
}
// EOTD-7:: DONE. 