#ifndef RRTSTAR_H
#define RRTSTAR_H

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <ctime>

#include "env.h"
#include "collision.h"

// struct Point {
//     double q1, q2, q3;
// };

struct Node {
    State position;
    Node* parent;
    double cost;
    std::vector<Node*> children;
    
    Node() : position({0, 0, 0}), parent(nullptr), cost(0.0) {}
    Node(State pos, Node* p = nullptr) : position(pos), parent(p), cost(0.0) {}
};

class RRTStar {
private:
    CollisionChecker collisionChecker;
    std::vector<Node*> nodes;
    State lower_bound = State(-M_PI/2,-M_PI/2,-M_PI/2), upper_bound=State(M_PI/2,M_PI/2,M_PI/2), start, goal;
    unsigned int max_iter;
    double goal_max_dist;
    double step_size;
    double search_radius;
    Node* nearest_to_goal=nullptr;

    double Distance(const State &a, const State &b);

    State RandomPoint();

    Node* Nearest(const State &point);

    State Steer(const State &nearest, const State &random_point);

    std::vector<Node*> Near(const State &point);

    void AttachNewNode(Node* new_node, std::vector<Node*>& near_nodes);

    void Rewire(Node* new_node, std::vector<Node*>& near_nodes);

    bool Reached();

public:
    RRTStar( State& start, State& goal, unsigned int iter, double step, double radius, CollisionChecker &collisionChecker);

    ~RRTStar();

    void Generate();

    std::vector<State> GetPath();

};

#endif // RRTSTAR_H