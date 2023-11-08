#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <utility>
#include "vec2.h"


// Define a structure for a link
struct Link {
    double l,w; // size of the link
    double x,y,alpha; // position and orientation of the link
    double q; // joint angle

    Link( double l, double w, double q);
};

// Define the Robot class
class Robot {

public:
    double w0, l0; // width of the base
    std::vector<Link> links; // Vector of links
    double linkpose[4][3] = {0}; // position and orientation of the links
    // Constructor
    Robot(double l1, double l2, double l3, double w);

    // FK and IK
    void FK(double q1, double q2, double q3, double &x, double &y, double &theta);

    void IK(double x, double y, double theta, double &q1, double &q2, double &q3);

    // move to joint state
    void moveToJointState(double q1, double q2, double q3);

    // Function to print the current configuration of the robot
    void printConfiguration() const;

    // Function to output collision polygons
    vector<vector<Vec2>> getCollisionPolygons() const;
};

#endif // ROBOT_H