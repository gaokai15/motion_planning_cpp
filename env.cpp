#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include "robot.h"
#include "env.h"
#include "rrt_connect.h"

using namespace std;

State::State() {
    joint[0] = joint[1] = joint[2] = 0.0;
}

State::State(double j1, double j2, double j3) {
    joint[0] = j1;
    joint[1] = j2;
    joint[2] = j3;
}


vector<Vec2> Box::getVertices() {
        vector<Vec2> vertices;
        vertices.emplace_back(x + w/2*cos(alpha) + h/2*cos(alpha+M_PI/2), y + w/2*sin(alpha) + h/2*sin(alpha+M_PI/2));
        vertices.emplace_back(x - w/2*cos(alpha) + h/2*cos(alpha+M_PI/2), y - w/2*sin(alpha) + h/2*sin(alpha+M_PI/2));
        vertices.emplace_back(x - w/2*cos(alpha) - h/2*cos(alpha+M_PI/2), y - w/2*sin(alpha) - h/2*sin(alpha+M_PI/2));
        vertices.emplace_back(x + w/2*cos(alpha) - h/2*cos(alpha+M_PI/2), y + w/2*sin(alpha) - h/2*sin(alpha+M_PI/2));
        return vertices;
    }

Environment::Environment(){

    }

void Environment::setUpEnv(
        vector<double> cf_target,
        vector<vector<double>> cf_obs_list,
        vector<double> cf_goal,
        double cf_link_length,
        double cf_link_width,
        double cf_box_width,
        double cf_box_height,
        double cf_clearance
        ){
    clearance = cf_clearance;
    // Define the robot
    robot = Robot(cf_link_length,cf_link_length,cf_link_length,cf_link_width);

    // Define a obstacle boxes
    for(auto box_pose : cf_obs_list){
        Box box = Box(box_pose[0], box_pose[1], box_pose[2], cf_box_width, cf_box_height);
        boxes.push_back(box);
    }
    Box box;

    // Target 
    target = Box(cf_target[0],cf_target[1],cf_target[2], cf_box_width, cf_box_height);
    goal = Box(cf_goal[0],cf_goal[1],cf_goal[2], cf_box_width, cf_box_height);
}


void Environment::getGraspPose(Box box, double& x, double& y, double& alpha){
    // given box pose, update robot pose
    alpha = box.alpha;
    x = box.x - cos(box.alpha)*box.w/2;
    y = box.y - sin(box.alpha)*box.w/2;
}

void Environment::updateTargetBoxPose(double x, double y, double alpha){
    // given robot pose, update box pose
    target.x = x + cos(alpha)*target.w/2;
    target.y = y + sin(alpha)*target.w/2;
    target.alpha = alpha;
}


void Environment::plotEnv(){
    // Create a blank image (500x500 pixels) with a white background (color is BGR)
    cv::Mat image = cv::Mat::zeros(ImageWidth, ImageHeight, CV_8UC3);

    update(image);


    // // Create a window for display.
    // cv::namedWindow("Motion Planning", cv::WINDOW_AUTOSIZE);

    // Show our image inside it.
    cv::imshow("Robot", image);

    // Wait for a keystroke in the window
    cv::waitKey(0);

}


void Environment::update(cv::Mat &image) {
    // update state
    image.setTo(cv::Scalar(255, 255, 255));

    // Draw the robot
    cv::Scalar robot_color(0, 0, 255);
    for ( int linkID = 0; linkID < 4; linkID++) {
        double cx = robot.linkpose[linkID][0] + cos(robot.linkpose[linkID][2])*robot.links[linkID].l/2;
        double cy = robot.linkpose[linkID][1] + sin(robot.linkpose[linkID][2])*robot.links[linkID].l/2;
        
        double img_cx, img_cy;
        env2ImgLoc(cx, cy, img_cx, img_cy);

        double img_w = env2ImgScale(robot.links[linkID].l);
        double img_h = env2ImgScale(robot.links[linkID].w);

        cv::RotatedRect rotatedRect(cv::Point2f(img_cx, img_cy), cv::Size2f(img_w, img_h), -robot.linkpose[linkID][2]/M_PI*180.0); // 30 degrees rotation

        // Get the vertices of the rotated rectangle
        cv::Point2f vertices[4];
        rotatedRect.points(vertices);

        // Convert the vertices points to a vector of points for polylines
        std::vector<cv::Point> points;
        for (int i = 0; i < 4; ++i) {
            points.push_back(vertices[i]);
        }

        // Close the rectangle shape
        points.push_back(vertices[0]);

        // Draw the tilted rectangle
        cv::polylines(image, points, true, robot_color, 2); // Green color, 2 px thickness
    }





    // Draw boxes
    cv::Scalar obs_color(0, 0, 0);
    for ( auto box : boxes) {
        
        double img_cx, img_cy;
        env2ImgLoc(box.x, box.y, img_cx, img_cy);

        double img_w = env2ImgScale(box.w);
        double img_h = env2ImgScale(box.h);
        
        cv::RotatedRect rotatedRect(cv::Point2f(img_cx, img_cy), cv::Size2f(img_w, img_h), -box.alpha/M_PI*180.0); // 30 degrees rotation

        // Get the vertices of the rotated rectangle
        cv::Point2f vertices[4];
        rotatedRect.points(vertices);

        // Convert the vertices points to a vector of points for polylines
        std::vector<cv::Point> points;
        for (int i = 0; i < 4; ++i) {
            points.push_back(vertices[i]);
        }

        // Close the rectangle shape
        points.push_back(vertices[0]);

        // Draw the tilted rectangle
        cv::polylines(image, points, true, obs_color, 2); 
    }




    // Draw target
    cv::Scalar target_color(0, 255, 0);
    double img_cx, img_cy;
    env2ImgLoc(target.x, target.y, img_cx, img_cy);

    double img_w = env2ImgScale(target.w);
    double img_h = env2ImgScale(target.h);
    
    cv::RotatedRect rotatedRect(cv::Point2f(img_cx, img_cy), cv::Size2f(img_w, img_h), -target.alpha/M_PI*180.0); // 30 degrees rotation

    // Get the vertices of the rotated rectangle
    cv::Point2f vertices[4];
    rotatedRect.points(vertices);

    // Convert the vertices points to a vector of points for polylines
    std::vector<cv::Point> points;
    for (int i = 0; i < 4; ++i) {
        points.push_back(vertices[i]);
    }

    // Close the rectangle shape
    points.push_back(vertices[0]);

    // Draw the tilted rectangle
    cv::polylines(image, points, true, target_color, 2); // Green color, 2 px thickness





    // Draw goal
    cv::Scalar goal_color(128, 128, 128);
    env2ImgLoc(goal.x, goal.y, img_cx, img_cy);

    img_w = env2ImgScale(goal.w);
    img_h = env2ImgScale(goal.h);
    
    rotatedRect = cv::RotatedRect(cv::Point2f(img_cx, img_cy), cv::Size2f(img_w, img_h), -goal.alpha/M_PI*180.0); // 30 degrees rotation

    // Get the vertices of the rotated rectangle
    rotatedRect.points(vertices);

    // Convert the vertices points to a vector of points for polylines
    points = std::vector<cv::Point>();
    for (int i = 0; i < 4; ++i) {
        points.push_back(vertices[i]);
    }

    // Close the rectangle shape
    points.push_back(vertices[0]);

    // Draw the tilted rectangle
    cv::polylines(image, points, true, goal_color, 2); // Green color, 2 px thickness
    
}


int Environment::animation(std::vector<State> path){
    // Create a blank image (500x500 pixels) with a white background (color is BGR)
    cv::Mat image = cv::Mat::zeros(ImageWidth, ImageHeight, CV_8UC3);
    cout<<"Animation"<<endl;
    plotEnv();
    
    int num_steps = 1;
    double i_q1, i_q2, i_q3; // initial state
    double curr_q1, curr_q2, curr_q3; // current state
    double x,y,alpha; // robot pose
    i_q1 = path[0].joint[0];
    i_q2 = path[0].joint[1];
    i_q3 = path[0].joint[2];
    for(auto state : path){
        double q1 = state.joint[0];
        double q2 = state.joint[1];
        double q3 = state.joint[2];
        for(int i = 0; i<num_steps; i++){
            curr_q1 = i_q1 + (q1-i_q1)/num_steps*i;
            curr_q2 = i_q2 + (q2-i_q2)/num_steps*i;
            curr_q3 = i_q3 + (q3-i_q3)/num_steps*i;
            robot.moveToJointState(curr_q1, curr_q2, curr_q3);
            robot.FK(q1,q2,q3,x,y,alpha);
            updateTargetBoxPose(x,y,alpha);
            update(image);
            cv::imshow("Animation", image);
            cv::waitKey(50);
        }
        i_q1 = q1;
        i_q2 = q2;
        i_q3 = q3;
    }
    cv::imshow("Animation", image);
    cv::waitKey(0);
    return 0;
}


double Environment::env2ImgScale(double x){
    return x/EnvWidth*ImageWidth;
}

double Environment::img2EnvScale(double x){
    return x/ImageWidth*EnvWidth;
}

void Environment::env2ImgLoc(double Ex, double Ey, double& x, double& y){
    x = env2ImgScale(Ex) + ImageWidth/2;
    y = -env2ImgScale(Ey) + ImageHeight/2;
}

void Environment::img2EnvLoc(double x, double y, double& Ex, double& Ey){
    Ex = img2EnvScale(x-ImageWidth/2);
    Ey = -img2EnvScale(y-ImageHeight/2);
}

bool Environment::isCollisionFree(const State &state){
    robot.moveToJointState(state.joint[0], state.joint[1], state.joint[2]);
    double x,y,alpha;
    robot.FK(state.joint[0], state.joint[1], state.joint[2],x,y,alpha);
    updateTargetBoxPose(x,y,alpha);
    vector<vector<Vec2>> polygons = robot.getCollisionPolygons();
    polygons.push_back(target.getVertices());
    for(auto box : boxes){
        std::vector<Vec2> box_vertices = box.getVertices();
        for(auto poly : polygons){
            if(false){ // use polygon distance
            if(checkPolygonCollision(poly, box_vertices)){
                return false;
            }}
            else{
                if(checkPolygonCollision(poly, box_vertices)){
                    return false;
                }
                if(polygonDistance(poly, box_vertices) <= clearance){
                    return false;
                }
            }
        }
    }
    return true;
}


// Project a polygon on an axis and returns it as a range
Projection Environment::project(const std::vector<Vec2>& polygon, const Vec2& axis) {
    double min = axis.dot(polygon[0]);
    double max = min;
    for (const auto& point : polygon) {
        double projected = axis.dot(point);
        min = (projected < min) ? projected : min;
        max = (projected > max) ? projected : max;
    }
    return {min, max};
}

// Check if two projections overlap
bool Environment::overlap(const Projection& proj1, const Projection& proj2) {
    return !(proj1.max < proj2.min || proj2.max < proj1.min);
}

// The main function to check for polygon collision
bool Environment::checkPolygonCollision(const std::vector<Vec2>& poly1, const std::vector<Vec2>& poly2) {
    std::vector<Vec2> axes;

    // Get the axes for the first polygon
    for (size_t i = 0; i < poly1.size(); i++) {
        Vec2 edge = poly1[(i + 1) % poly1.size()] - poly1[i];
        axes.push_back(edge.perp());
    }

    // Get the axes for the second polygon
    for (size_t i = 0; i < poly2.size(); i++) {
        Vec2 edge = poly2[(i + 1) % poly2.size()] - poly2[i];
        axes.push_back(edge.perp());
    }

    // Check for overlap on all axes
    for (const auto& axis : axes) {
        Vec2 normalizedAxis = axis;
        normalizedAxis.normalize();
        Projection proj1 = project(poly1, normalizedAxis);
        Projection proj2 = project(poly2, normalizedAxis);
        if (!overlap(proj1, proj2)) {
            return false; // No collision
        }
    }

    return true; // Collision
}

// Helper function to compute the projection of a point onto a line defined by two points (a, b)
double Environment::pointLineDistance(const Vec2& point, const Vec2& a, const Vec2& b) {
    Vec2 ap = point - a;
    Vec2 ab = b - a;
    double magnitudeAB = ab.magnitude();
    double abDotAp = ab.dot(ap);
    double distance = abDotAp / magnitudeAB;
    Vec2 closestPoint;
    if (distance < 0) {
        closestPoint = a;
    } else if (distance > magnitudeAB) {
        closestPoint = b;
    } else {
        closestPoint = a + ab * (abDotAp / (magnitudeAB * magnitudeAB));
    }
    return point.distance(closestPoint);
}

// Check if the projections of two ranges overlap
bool Environment::overlap(double minA, double maxA, double minB, double maxB) {
    return !(minA > maxB || minB > maxA);
}

// Compute the minimum distance between two polygons
double Environment::polygonDistance(const std::vector<Vec2>& polyA, const std::vector<Vec2>& polyB) {
    double minDistance = std::numeric_limits<double>::infinity();
    
    // Check distances from vertices of polyA to edges of polyB and vice versa
    for (int i = 0; i < 2; ++i) {
        const std::vector<Vec2>& poly1 = (i == 0) ? polyA : polyB;
        const std::vector<Vec2>& poly2 = (i == 0) ? polyB : polyA;

        for (size_t j = 0; j < poly1.size(); ++j) {
            Vec2 a = poly1[j];
            Vec2 b = poly1[(j + 1) % poly1.size()];

            for (const auto& p : poly2) {
                double dist = pointLineDistance(p, a, b);
                minDistance = std::min(minDistance, dist);
            }
        }
    }

    // Check vertex to vertex distances
    for (const auto& vA : polyA) {
        for (const auto& vB : polyB) {
            double dist = vA.distance( vB);
            minDistance = std::min(minDistance, dist);
        }
    }

    return minDistance;
}


