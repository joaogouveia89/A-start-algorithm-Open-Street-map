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
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}

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

RouteModel::Node *RoutePlanner::NextNode() {
    CellSort(&open_list);
    RouteModel::Node* current = open_list.back();
    open_list.pop_back();
    return current;
}


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

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;
    while (current_node != this->end_node){
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    std::vector<RouteModel::Node> final_path = ConstructFinalPath(current_node);
    m_Model.path = final_path;
}