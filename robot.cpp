#include "robot.h"
#include <iostream>
#include <cmath>

// Link constructor implementation
Link::Link(double l, double w, double q) : l(l), w(w), q(q) {}

// Robot constructor implementation
Robot::Robot(double l1, double l2, double l3, double w) {
    w0 = w;
    l0 = 0.1;
    links.emplace_back(l0, w0, 0);
    links.emplace_back(l1, w, 0);
    links.emplace_back(l2, w, 0);
    links.emplace_back(l3, w, 0);

    moveToJointState(0, 0, 0);
}

void Robot::FK(double q1, double q2, double q3, double &x, double &y, double &theta){
    x=0, y=0, theta=M_PI/2;
    for(int linkID=1; linkID <4; linkID++){
        x += links[linkID-1].l*cos(theta);
        y += links[linkID-1].l*sin(theta);
        theta += links[linkID].q;
    }
    x += links[3].l*cos(theta);
    y += links[3].l*sin(theta);
}

void Robot::IK(double x, double y, double theta, double &q1, double &q2, double &q3){
    q1 = atan2(y,x) - atan2(links[1].l*sin(theta), l0 + links[1].l*cos(theta));
    q2 = atan2(y - links[1].l*sin(q1), x - links[1].l*cos(q1)) - q1;
    q3 = theta - q1 - q2;
}

void Robot::moveToJointState(double q1, double q2, double q3){
    links[0].q = 0;
    links[1].q = q1;
    links[2].q = q2;
    links[3].q = q3;

    double x=0, y=0, theta=M_PI/2;
    linkpose[0][0] = x;
    linkpose[0][1] = y;
    linkpose[0][2] = theta;
    for(int linkID=1; linkID <4; linkID++){
        x += links[linkID-1].l*cos(theta);
        y += links[linkID-1].l*sin(theta);
        theta += links[linkID].q;
        linkpose[linkID][0] = x;
        linkpose[linkID][1] = y;
        linkpose[linkID][2] = theta;
    }
}

void Robot::getEndEffectorPose() const {
    std::cout << "End-effector position: (" << linkpose[3][0] << ", "
              << linkpose[3][1] << ")" << std::endl;
}

// printConfiguration implementation
void Robot::printConfiguration() const {
    for (size_t i = 0; i < links.size(); ++i) {
        std::cout << "Link " << i+1 << ": Length = " << links[i].l
                  << ", Angle = " << links[i].q << " radians" << std::endl;
    }
}
