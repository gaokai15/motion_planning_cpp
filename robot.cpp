#include "robot.h"
#include <iostream>
#include <cmath>

using namespace std;
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
    // vector<double> q1_list;
    // vector<double> q2_list;
    // vector<double> q3_list;

    // to frame 1
    double theta0 = linkpose[0][2];
    double f1x = cos(-theta0)*x -sin(-theta0)*y - l0;
    double f2y = sin(-theta0)*x +cos(-theta0)*y;
    x = f1x;
    y = f2y;
    theta = theta - theta0;

    double u = x - links[3].l*cos(theta);
    double v = y - links[3].l*sin(theta);
    double cosq2 = (u*u + v*v - links[1].l*links[1].l - links[2].l*links[2].l)/(2*links[1].l*links[2].l);
    double sinq2 = sqrt(1-cosq2*cosq2);
    q2 = atan2(sinq2, cosq2);
    double sinq1 = (v*(links[1].l + links[2].l*cosq2) - u*links[2].l*sinq2)/(links[1].l*links[1].l + links[2].l*links[2].l + 2*links[1].l*links[2].l*cosq2);
    double cosq1 = (u*(links[1].l + links[2].l*cosq2) + v*links[2].l*sinq2)/(links[1].l*links[1].l + links[2].l*links[2].l + 2*links[1].l*links[2].l*cosq2);
    q1 = atan2(sinq1, cosq1);
    q3 = theta - q1 - q2;
    if(isnan(q1) || isnan(q2) || isnan(q3)){
        cout<<endl<<endl;
        cout << "Grasp/Place Pose Unreachable: IK failed" << endl;
        exit(1);
    }
    if(q1 > M_PI/2 || q1 < -M_PI/2 || q2 > M_PI/2 || q2 < -M_PI/2 || q3 > M_PI/2 || q3 < -M_PI/2){
        cout<<endl<<endl;
        cout<<q1<<" "<<q2<<" "<<q3<<endl;
        cout << "Grasp/Place Pose Out of Assumed Bound [-pi/2, pi/2]" << endl;
        exit(1);
    }
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

// printConfiguration implementation
void Robot::printConfiguration() const {
    for (size_t i = 0; i < links.size(); ++i) {
        std::cout << "Link " << i+1 << ": Length = " << links[i].l
                  << ", Angle = " << links[i].q << " radians" << std::endl;
    }
}


vector<vector<Vec2>> Robot::getCollisionPolygons() const {
    vector<vector<Vec2>> polygons;
    for(int i=0; i<4; i++){
        vector<Vec2> polygon;
        double x = linkpose[i][0];
        double y = linkpose[i][1];
        double theta = linkpose[i][2];
        double w = links[i].w;
        double l = links[i].l;
        polygon.push_back(Vec2(x + cos(theta-M_PI/2.0)*w/2.0, y + sin(theta-M_PI/2.0)*w/2.0));
        polygon.push_back(Vec2(x + cos(theta-M_PI/2.0)*w/2.0 + cos(theta)*l, y + sin(theta-M_PI/2.0)*w/2.0 + sin(theta)*l));
        polygon.push_back(Vec2(x + cos(theta+M_PI/2.0)*w/2.0 + cos(theta)*l, y + sin(theta+M_PI/2.0)*w/2.0 + sin(theta)*l));
        polygon.push_back(Vec2(x + cos(theta+M_PI/2.0)*w/2.0, y + sin(theta+M_PI/2.0)*w/2.0));
        polygons.push_back(polygon);
    }
    return polygons;
}