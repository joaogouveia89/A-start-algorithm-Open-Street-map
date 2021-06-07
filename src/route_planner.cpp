#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &model.FindClosestNode(start_x, start_y);
    start_node->visited = true;
    this->end_node = &model.FindClosestNode(end_x, end_y);

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes. DONE!!! BUT NOT SURE

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node. //DONE !!

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true. // DONE

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    current_node->FindNeighbors();
    int currentNodeNeighborsCount = current_node->neighbors.size(); // to avoid calculating all the time this size inside the for loop

    for(int i = 0; i < currentNodeNeighborsCount; i++){
        auto neighbor = current_node->neighbors[i];
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;
        open_list.push_back(neighbor);
    }
}



bool RoutePlanner::Compare(const RouteModel::Node* n1, const RouteModel::Node* n2){
    float f1 = n1->h_value + n1->g_value;
    float f2 = n2->h_value + n2->g_value;
    return f1 > f2;
}

void RoutePlanner::CellSort(std::vector<RouteModel::Node*>* v) {
    sort(v->begin(), v->end(), Compare);
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer. DONE

RouteModel::Node *RoutePlanner::NextNode() {
    CellSort(&open_list);
    RouteModel::Node* current = open_list.back();
    open_list.pop_back();
    return current;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element. //DONE

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // TODO: Implement your solution here.
    while (current_node != start_node){
        distance += current_node->distance(*current_node->parent); // not sure if it is the best solution as I think I can avoid this calculation, in some part of the code, maybe on AddNeighbors method. If I had more time I would study this more carefully
        path_found.insert(path_found.begin(), *current_node);
        current_node = current_node->parent;
    }
    //adding the start node
    path_found.insert(path_found.begin(), *current_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
     // TODO: Implement your solution here.
    RouteModel::Node *current_node = this->start_node;
    while (current_node != this->end_node){
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    std::vector<RouteModel::Node> final_path = ConstructFinalPath(current_node);
    m_Model.path = final_path;
}