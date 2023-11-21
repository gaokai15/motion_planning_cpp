#include "rrt_star.h"


double RRTStar::Distance(const State &a, const State &b) {
    return sqrt(pow(a.joint[0] - b.joint[0], 2) + pow(a.joint[1] - b.joint[1], 2) + pow(a.joint[2] - b.joint[2], 2));
}

State RRTStar::RandomPoint() {
    State p;
    for(int i=0;i<3;i++){
        p.joint[i] = lower_bound.joint[i] + (upper_bound.joint[i] - lower_bound.joint[i]) * ((double)rand() / (double)RAND_MAX);
    }
    return p;
}

Node* RRTStar::Nearest(const State &point) {
    Node* nearest = nullptr;
    double nearest_dist = std::numeric_limits<double>::max();
    
    for (auto node : nodes) {
        double dist = Distance(node->position, point);
        if (dist < nearest_dist) {
            nearest_dist = dist;
            nearest = node;
        }
    }
    
    return nearest;
}

State RRTStar::Steer(const State &nearest, const State &random_point) {
    // Calculate the direction from the nearest node to the target state
    State direction(
        random_point.joint[0] - nearest.joint[0],
        random_point.joint[1] - nearest.joint[1],
        random_point.joint[2] - nearest.joint[2]
    );

    // Normalize the direction
    double norm = sqrt(
        direction.joint[0] * direction.joint[0] +
        direction.joint[1] * direction.joint[1] +
        direction.joint[2] * direction.joint[2]
    );
    
    if(norm>step_size){
        return State(
            nearest.joint[0] + step_size * direction.joint[0] / norm,
            nearest.joint[1] + step_size * direction.joint[1] / norm,
            nearest.joint[2] + step_size * direction.joint[2] / norm
        );
    }
    else{
        return random_point;
    }
}

std::vector<Node*> RRTStar::Near(const State &point) {
    std::vector<Node*> near_nodes;
    for (auto node : nodes) {
        if (Distance(node->position, point) < search_radius) {
            near_nodes.push_back(node);
        }
    }
    return near_nodes;
}

void RRTStar::AttachNewNode(Node* new_node, std::vector<Node*>& near_nodes) {
    Node* min_node = new_node->parent;
    double min_cost = new_node->parent->cost + Distance(new_node->position, new_node->parent->position);

    for (auto near_node : near_nodes) {
        double cost = near_node->cost + Distance(new_node->position, near_node->position);
        if (cost < min_cost) {
            min_node = near_node;
            min_cost = cost;
        }
    }

    new_node->parent = min_node;
    new_node->cost = min_cost;
    min_node->children.push_back(new_node);
}

void RRTStar::Rewire(Node* new_node, std::vector<Node*>& near_nodes) {
    for (auto near_node : near_nodes) {
        double new_cost = new_node->cost + Distance(new_node->position, near_node->position);
        if (new_cost < near_node->cost) {
            Node* old_parent = near_node->parent;
            old_parent->children.erase(std::remove(old_parent->children.begin(), old_parent->children.end(), near_node), old_parent->children.end());

            near_node->parent = new_node;
            near_node->cost = new_cost;
            new_node->children.push_back(near_node);
        }
    }
}

bool RRTStar::Reached() {
    Node* last_node = nodes.back();
    double dist_to_goal = Distance(last_node->position, goal);
    return dist_to_goal <= goal_max_dist;
}


RRTStar::RRTStar( State& start, State& goal, unsigned int iter, double step, double radius, CollisionChecker &collisionChecker)
    : start(start), goal(goal), max_iter(iter), step_size(step), search_radius(radius), goal_max_dist(step), collisionChecker(collisionChecker) {
    nodes.push_back(new Node(start));
}

RRTStar::~RRTStar() {
    for (auto node : nodes) {
        delete node;
    }
}

void RRTStar::Generate() {
    srand(time(nullptr));
    for (unsigned int i = 0; i < max_iter; i++) {
        if(i%1000==0){
            std::cout << "Iteration: " << i << std::endl;
        }
        State random_point = RandomPoint();
        Node* nearest_node = Nearest(random_point);
        State new_point = Steer(nearest_node->position, random_point);
        if (!collisionChecker.isCollisionFree(new_point)) {
            continue;
        }

        Node* new_node = new Node(new_point, nearest_node);
        std::vector<Node*> near_nodes = Near(new_point);
        
        AttachNewNode(new_node, near_nodes);
        Rewire(new_node, near_nodes);
        
        nodes.push_back(new_node);

        if (Reached()) {
            std::cout << "Goal reached." << std::endl;
            nearest_to_goal = nodes.back();
            // break;
        }
    }
}

std::vector<State> RRTStar::GetPath() {
    std::vector<State> path;
    if (nearest_to_goal==nullptr) {
        std::cout << "Goal not reached, no path to return." << std::endl;
        return path;
    }

    Node* current = nearest_to_goal;
    while (current->parent != nullptr) {
        path.push_back(current->position);
        current = current->parent;
    }
    path.push_back(start); // Finally, add the start position
    
    std::reverse(path.begin(), path.end());
    return path;
}
