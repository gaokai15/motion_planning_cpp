#ifndef RRTCONNECT_H
#define RRTCONNECT_H
#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <algorithm>
#include <memory>  

#include "collision.h"


class TreeNode {
public:
    State state;
    TreeNode *parent;

    TreeNode(const State &state, TreeNode *parent = nullptr): state(state), parent(parent) {};

};

class RRTTree {
private:
    
public:
    CollisionChecker collisionChecker;
    std::vector<TreeNode *> nodes;
    // Define step size
    static constexpr double stepSize = 0.05; // RRT step size
    RRTTree(const State &rootState, CollisionChecker &collisionChecker);

    ~RRTTree();

    TreeNode *nearest(const State &targetState);

    double distance(const State &a, const State &b);

    TreeNode *extend(const State &targetState);

    TreeNode *connect(const State &targetState, bool &Connected);

    // Additional methods as necessary
};

class RRTConnectPlanner {
private:
    RRTTree *treeA;
    RRTTree *treeB;
    CollisionChecker collisionChecker;

    // Assuming you have some predefined limits for your joints
    static constexpr double minJointLimits[3] = {-M_PI/2, -M_PI/2, -M_PI/2}; // Replace with your actual limits
    static constexpr double maxJointLimits[3] = {M_PI/2, M_PI/2, M_PI/2}; // Replace with your actual limits

    // Random device and generator for random state generation
    std::random_device rd;
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<> dist[3]{
        std::uniform_real_distribution<>(minJointLimits[0], maxJointLimits[0]),
        std::uniform_real_distribution<>(minJointLimits[1], maxJointLimits[1]),
        std::uniform_real_distribution<>(minJointLimits[2], maxJointLimits[2])
    };

public:
    RRTConnectPlanner(const State &start, const State &goal, CollisionChecker &collisionChecker);

    std::vector<State> plan();

    State randomState();

    std::vector<State> extractPath(TreeNode *endNodeA, TreeNode *endNodeB);

};

#endif // RRTCONNECT_H