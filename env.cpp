#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include "robot.h"

using namespace std;
// Define a box
struct Box {
    double x,y,alpha,w,h;

    Box(double x, double y, double alpha, double w, double h) : x(x), y(y), alpha(alpha), w(w), h(h) {}
    Box() : x(0), y(0), alpha(0), w(0), h(0) {}
};

class Environment {
public:
    Robot robot = Robot(0.3,0.3,0.3,0.1);;
    std::vector<Box> boxes;
    Box target;
    Box goal;

    // Plotting
    int ImageWidth = 500;
    int ImageHeight = 500;
    double EnvWidth = 3;
    double EnvHeight = 3;


    Environment(){

    }

    void setUpEnv(){
        // Define the robot
        robot = Robot(0.4,0.4,0.4,0.05);
        // print link pose
        std::cout << "Link pose: " << std::endl;
        for ( int linkID = 0; linkID < 4; linkID++) {
            std::cout<<linkID<<std::endl;
            std::cout << robot.linkpose[1][0] << " " << robot.linkpose[1][1] << " " << robot.linkpose[1][2] <<std::endl;
        }

        // Define a obstacle boxes
        Box box;
        box = Box(1.1, 0.4, 0.0, 0.3, 0.3);
        boxes.push_back(box);
        box = Box(1.1, -0.3, 0.0, 0.3, 0.3);
        boxes.push_back(box);

        // Target 
        target = Box(1.1, 0.05, 0.0, 0.3, 0.3);
        goal = Box(0.6, -0.15, -M_PI/2.0, 0.3, 0.3);
    };

    void plotEnv(){
        // Create a blank image (500x500 pixels) with a white background (color is BGR)
        cv::Mat image = cv::Mat::zeros(500, 500, CV_8UC3);
        image.setTo(cv::Scalar(255, 255, 255));

        // Define a few rectangles with (x, y, width, height)
        std::vector<cv::Rect> rectangles = {
            cv::Rect(50, 50, 100, 150),  // Rectangle 1
            cv::Rect(200, 100, 200, 100), // Rectangle 2
            cv::Rect(150, 250, 100, 200), // Rectangle 3
            // Add more rectangles if needed
        };

        // Draw the robot
        cv::Scalar robot_color(0, 0, 255);
        double theta = 0;
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
            
            cv::RotatedRect rotatedRect(cv::Point2f(img_cx, img_cy), cv::Size2f(img_w, img_h), box.alpha/M_PI*180.0); // 30 degrees rotation

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
        
        cv::RotatedRect rotatedRect(cv::Point2f(img_cx, img_cy), cv::Size2f(img_w, img_h), target.alpha/M_PI*180.0); // 30 degrees rotation

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
        
        rotatedRect = cv::RotatedRect(cv::Point2f(img_cx, img_cy), cv::Size2f(img_w, img_h), goal.alpha/M_PI*180.0); // 30 degrees rotation

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




        // // Create a window for display.
        // cv::namedWindow("Motion Planning", cv::WINDOW_AUTOSIZE);

        // Show our image inside it.
        cv::imshow("Robot", image);

        // Wait for a keystroke in the window
        cv::waitKey(0);

    };

    double env2ImgScale(double x){
        return x/EnvWidth*ImageWidth;
    };

    double img2EnvScale(double x){
        return x/ImageWidth*EnvWidth;
    };

    void env2ImgLoc(double Ex, double Ey, double& x, double& y){
        x = env2ImgScale(Ex) + ImageWidth/2;
        y = -env2ImgScale(Ey) + ImageHeight/2;
    };

    void img2EnvLoc(double x, double y, double& Ex, double& Ey){
        Ex = img2EnvScale(x-ImageWidth/2);
        Ey = -img2EnvScale(y-ImageHeight/2);
    };
};

int main(){
    Environment env = Environment();
    env.setUpEnv();
    env.robot.moveToJointState(-0.15*M_PI, -0.4*M_PI, M_PI/3.0);
    // std::cout << cv::getBuildInformation() << std::endl;
    env.plotEnv();
    return 0;
}