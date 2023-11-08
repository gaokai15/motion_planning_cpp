#ifndef ENV_H
#define ENV_H

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include "robot.h"

using namespace std;

class State {
public:
    double joint[3];

    State();
    State(double j1, double j2, double j3);

};

// Define a box
struct Box {
    double x,y,alpha,w,h;

    Box(double x, double y, double alpha, double w, double h) : x(x), y(y), alpha(alpha), w(w), h(h) {}
    Box() : x(0), y(0), alpha(0), w(0), h(0) {}

    vector<Vec2> getVertices();
};


struct Projection {
    float min, max;
};


class Environment {
public:
    Robot robot = Robot(0.3,0.3,0.3,0.1);
    std::vector<Box> boxes;
    Box target;
    Box goal;

    // Plotting
    int ImageWidth = 500;
    int ImageHeight = 500;
    double EnvWidth = 3;
    double EnvHeight = 3;


    Environment();

    void setUpEnv();

    void getGraspPose(Box box, double& x, double& y, double& alpha);

    void updateTargetBoxPose(double x, double y, double alpha);

    void plotEnv();


    void update(cv::Mat &image);


    int animation(std::vector<State> path);


    double env2ImgScale(double x);

    double img2EnvScale(double x);

    void env2ImgLoc(double Ex, double Ey, double& x, double& y);

    void img2EnvLoc(double x, double y, double& Ex, double& Ey);

    bool isCollisionFree(const State &state);

    // Project a polygon on an axis and returns it as a range
    Projection project(const std::vector<Vec2>& polygon, const Vec2& axis);

    // Check if two projections overlap
    bool overlap(const Projection& proj1, const Projection& proj2);

    // The main function to check for polygon collision
    bool checkPolygonCollision(const std::vector<Vec2>& poly1, const std::vector<Vec2>& poly2);
    
};

#endif // ENV_H