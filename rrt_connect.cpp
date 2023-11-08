// #include <vector>
// #include <cmath>
// #include <limits>
// #include <random>
// #include <algorithm>
// #include <memory>  

// #include "collision.h"
#include "rrt_connect.h"


RRTTree::RRTTree(const State &rootState) {
    nodes.push_back(new TreeNode(rootState));
}

RRTTree::~RRTTree() {
    for (auto *node : nodes) {
        delete node;
    }
}

TreeNode* RRTTree::nearest(const State &targetState) {
    TreeNode *nearestNode = nullptr;
    double minDistance = std::numeric_limits<double>::max();

    for (auto *node : nodes) {
        double dist = distance(node->state, targetState);
        if (dist < minDistance) {
            minDistance = dist;
            nearestNode = node;
        }
    }

    return nearestNode;
}

double RRTTree::distance(const State &a, const State &b) {
    // Euclidean distance in joint space
    double d0 = a.joint[0] - b.joint[0];
    double d1 = a.joint[1] - b.joint[1];
    double d2 = a.joint[2] - b.joint[2];
    return sqrt(d0 * d0 + d1 * d1 + d2 * d2);
}

TreeNode *RRTTree::extend(const State &targetState) {
    
    // Find the nearest node in the tree to the target state
    TreeNode *nearestNode = nearest(targetState);

    // Calculate the direction from the nearest node to the target state
    State direction(
        targetState.joint[0] - nearestNode->state.joint[0],
        targetState.joint[1] - nearestNode->state.joint[1],
        targetState.joint[2] - nearestNode->state.joint[2]
    );

    // Normalize the direction
    double norm = sqrt(
        direction.joint[0] * direction.joint[0] +
        direction.joint[1] * direction.joint[1] +
        direction.joint[2] * direction.joint[2]
    );

    State newState;
    if (norm > stepSize) {
        // Move from nearest node towards the target by the step size
        double scale = stepSize / norm;
        newState = State(
            nearestNode->state.joint[0] + scale * direction.joint[0],
            nearestNode->state.joint[1] + scale * direction.joint[1],
            nearestNode->state.joint[2] + scale * direction.joint[2]
        );
    } else {
        // Target state is within one step size, so set the new state to the target state
        newState = targetState;
    }

    // Check if the path to the new state is collision-free
    if (collisionChecker.isCollisionFree(newState)) {
        // Create a new node and add it to the tree
        TreeNode *newNode = new TreeNode(newState, nearestNode);
        nodes.push_back(newNode);
        return newNode;
    }

    // If the path is not collision-free, return nullptr
    return nullptr;
}

TreeNode *RRTTree::connect(const State &targetState) {
    TreeNode *newNode = nullptr;

    while (true) {
        TreeNode *nearestNode = nearest(targetState);
        
        State direction(
            targetState.joint[0] - nearestNode->state.joint[0],
            targetState.joint[1] - nearestNode->state.joint[1],
            targetState.joint[2] - nearestNode->state.joint[2]
        );

        // Normalize the direction
        double norm = sqrt(
            direction.joint[0] * direction.joint[0] +
            direction.joint[1] * direction.joint[1] +
            direction.joint[2] * direction.joint[2]
        );

        if (norm < std::numeric_limits<double>::epsilon()) {
            return newNode; // The target state is the same as the nearest node
        }

        State newState;
        bool stepCloser = false;
        if (norm > stepSize) {
            // Move from nearest node towards the target by the step size
            double scale = stepSize / norm;
            newState = State(
                nearestNode->state.joint[0] + scale * direction.joint[0],
                nearestNode->state.joint[1] + scale * direction.joint[1],
                nearestNode->state.joint[2] + scale * direction.joint[2]
            );
            stepCloser = true;
        } else {
            // Target state is within one step size, so set the new state to the target state
            newState = targetState;
        }

        // Check if the path to the new state is collision-free
        if (collisionChecker.isCollisionFree(newState)) {
            // Create a new node and add it to the tree
            newNode = new TreeNode(newState, nearestNode);
            nodes.push_back(newNode);

            // If a step was taken closer, continue, otherwise we've reached the target state
            if (!stepCloser) {
                return newNode;
            }
        } else {
            // If the path is not collision-free, return the last valid node
            return newNode;
        }
    }
}


std::vector<State> RRTConnectPlanner::plan() {
    bool reversed = false;
    while (true) {
        State randState = randomState();
        if (collisionChecker.isCollisionFree(randState)) {
            TreeNode *newNodeA = treeA.extend(randState);
            if (newNodeA && treeB.connect(newNodeA->state)) {
                return extractPath(newNodeA, treeB.nearest(newNodeA->state));
            }
            std::swap(treeA, treeB);
            reversed = !reversed;
        }
    }
}

State RRTConnectPlanner::randomState() {
    // Generate a random state within the joint limits
    State randomState;
    randomState.joint[0] = dist[0](gen);
    randomState.joint[1] = dist[1](gen);
    randomState.joint[2] = dist[2](gen);
    return randomState;
}

std::vector<State> RRTConnectPlanner::extractPath(TreeNode *endNodeA, TreeNode *endNodeB) {
    std::vector<State> path;
    // generate path in treeA
    TreeNode *currentNode = endNodeA;

    // Traverse from the end node back to the root node
    while (currentNode != nullptr) {
        path.push_back(currentNode->state); // Add the state to the path
        currentNode = currentNode->parent; // Move to the parent node
    }

    // The path is currently from end to start, so reverse it to start from the start node
    std::reverse(path.begin(), path.end());

    // generate path in treeB
    currentNode = endNodeB;

    // Traverse from the end node back to the root node
    while (currentNode != nullptr) {
        path.push_back(currentNode->state); // Add the state to the path
        currentNode = currentNode->parent; // Move to the parent node
    }

    return path;
}


