#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <algorithm>
#include <memory>  

class State {
public:
    double joint[3];

    State() {
        joint[0] = joint[1] = joint[2] = 0.0;
    }

    State(double j1, double j2, double j3) {
        joint[0] = j1;
        joint[1] = j2;
        joint[2] = j3;
    }

    // Add more functionality if needed
};

class TreeNode {
public:
    State state;
    TreeNode *parent;

    TreeNode(const State &state, TreeNode *parent = nullptr)
        : state(state), parent(parent) {}
};

class CollisionChecker {
public:
    bool isCollisionFree(const State &state) {
        return true;
    }
};

class RRTTree {
private:
    CollisionChecker collisionChecker;
public:
    std::vector<TreeNode *> nodes;
    // Define step size
    static constexpr double stepSize = 0.05; // This should be chosen according to the application

    RRTTree(const State &rootState) {
        nodes.push_back(new TreeNode(rootState));
    }

    ~RRTTree() {
        for (auto *node : nodes) {
            delete node;
        }
    }

    TreeNode *nearest(const State &targetState) {
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

    double distance(const State &a, const State &b) {
        // Euclidean distance in joint space
        double d0 = a.joint[0] - b.joint[0];
        double d1 = a.joint[1] - b.joint[1];
        double d2 = a.joint[2] - b.joint[2];
        return sqrt(d0 * d0 + d1 * d1 + d2 * d2);
    }

    TreeNode *extend(const State &targetState) {
        
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

    TreeNode *connect(const State &targetState) {
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

    // Additional methods as necessary
};

class RRTConnectPlanner {
private:
    RRTTree treeA;
    RRTTree treeB;
    CollisionChecker collisionChecker;

public:
    RRTConnectPlanner(const State &start, const State &goal)
        : treeA(start), treeB(goal) {}

    std::vector<State> plan() {
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

    // Assuming you have some predefined limits for your joints
    static constexpr double minJointLimits[3] = {-M_PI, -M_PI, -M_PI}; // Replace with your actual limits
    static constexpr double maxJointLimits[3] = {M_PI, M_PI, M_PI}; // Replace with your actual limits

    // Random device and generator for random state generation
    std::random_device rd;
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<> dist[3]{
        std::uniform_real_distribution<>(minJointLimits[0], maxJointLimits[0]),
        std::uniform_real_distribution<>(minJointLimits[1], maxJointLimits[1]),
        std::uniform_real_distribution<>(minJointLimits[2], maxJointLimits[2])
    };

    State randomState() {
        // Generate a random state within the joint limits
        State randomState;
        randomState.joint[0] = dist[0](gen);
        randomState.joint[1] = dist[1](gen);
        randomState.joint[2] = dist[2](gen);
        return randomState;
    }

    std::vector<State> extractPath(TreeNode *endNodeA, TreeNode *endNodeB) {
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
};

/*
// Example usage
int main() {
    State start(0, 0, 0);
    State goal(1, 1, 1);
    RRTConnectPlanner planner(start, goal);

    std::vector<State> path = planner.plan();
    // Process path
    return 0;
}
*/