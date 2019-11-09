#include "route_planner.h"
#include <algorithm>
#include <iostream>

static     int count = 1;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) 
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(model.FindClosestNode(end_x, end_y));
    open_list.clear(); 
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
  return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
    // std::cout << "AddNeighbors: open_list size BEFORE " << open_list.size() << std::endl;
    current_node->FindNeighbors();
    for (auto aNeighbor : current_node->neighbors)
    {
        if (aNeighbor->visited == false)
        {
            aNeighbor->parent = current_node;
            aNeighbor->g_value = aNeighbor->distance(*start_node);
            float tenative_gScore = aNeighbor->distance(*current_node);
            if (tenative_gScore < aNeighbor->g_value)
            {
                aNeighbor->g_value = tenative_gScore;
            }
            aNeighbor->h_value = CalculateHValue(aNeighbor);
            aNeighbor->visited = true;
            open_list.push_back(aNeighbor);
        }
        else
        {
            // std::cout << "AddNeighbors: neighbor " << aNeighbor << " already visited, check to see if closer\n";
        }
    }
    // std::cout << "AddNeighbors openlist size AFTER " << open_list.size() << "\n";
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
bool Compare(const RouteModel::Node *a, const RouteModel::Node * b)
{
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() 
{
    std::sort(open_list.begin(), open_list.end(),Compare);
    // std::cout << "NextNode: Entry sorted open_list size of " << open_list.size() << std::endl;
    if ((count > 24) && (count < 27))
    {
        for (auto tOLE : open_list)
        {
            std::cout << "\tnode " << tOLE << " g " << tOLE->g_value << " h " << tOLE->h_value << "\n";
        } 
    }
    auto nextNode = open_list.back();
    open_list.pop_back();
    std::cout << "NextNode: returning node " << nextNode << " open_list size now " << open_list.size() << "\n";
    return nextNode;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) 
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node * active_node = current_node;
    bool goal_path_found = false;

    //std::cout << "CFP: start\n";

    while ((goal_path_found == false) && (active_node != nullptr))
    {
        //std::cout << "CFP: while active_node: " << active_node << " start_node " << start_node << " end_node: " << end_node << " start_node: " << start_node << "\n";
        //std::cout << "CFP: path size " << path_found.size() << "\n\n";
        if (active_node == start_node)
        {
            // std::cout << "CFP: back at start\n";
            goal_path_found = true;
            //std::cout << "CFP: before final push_back\n";
            path_found.push_back(*active_node);
        }
        else
        {
            //std::cout << "CFP: going to next node calculating distance...\n";
            distance += active_node->distance(*(active_node->parent));
            //std::cout << "CFP: going to next node, pushing back active node to path_found\n";
            path_found.push_back(*active_node);
            active_node = active_node->parent;
            // std::cout << "CFP: looping to parent node, address of " << active_node << " distance of " << distance <<"\n";
        }
    }

    // std::cout << "CFP: out of while loop...\n";

    if ((goal_path_found == false) || (active_node == nullptr))
    {
        distance = 0.0f;
        path_found.clear();
    }
    else
    {
        std::reverse(path_found.begin(),path_found.end());
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::cout << " path_found.size() " << path_found.size() << "\n";

    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    current_node = start_node;
    current_node->g_value = 0.0f;
    current_node->h_value = CalculateHValue(current_node);
    current_node->visited = true;

    open_list.push_back(current_node);


    while (open_list.size() > 0) // >= because very last node pulled from list below could be end_node
    {
        // std::cout << "AStarSearch: Enter while loop for the " << debugWhileControl << " time.\n";
        current_node = NextNode();
        std::cout << "AStarSearch: While: " << count++ << " CN " << current_node << " EN " << end_node << " D " << current_node->distance(*end_node) <<"\n";

        if (current_node == end_node) 
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        else
        {
            AddNeighbors(current_node);
        }
    }
    std::cout << "ASearch (2) OLsize = " << open_list.size() << "\n";
    m_Model.path = std::vector<RouteModel::Node>{};
    return;
}
