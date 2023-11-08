#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include "robot.h"
#include "rrt_connect.cpp"

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

        // Define a obstacle boxes
        Box box;
        box = Box(1.1, 0.7, 0.0, 0.3, 0.3);
        boxes.push_back(box);
        box = Box(1.1, 0.0, 0.0, 0.3, 0.3);
        boxes.push_back(box);

        // Target 
        target = Box(1.1, 0.35, 0.0, 0.3, 0.3);
        goal = Box(0, 1.3, M_PI/2.0, 0.3, 0.3);
    };


    void getGraspPose(Box box, double& x, double& y, double& alpha){
        // given box pose, update robot pose
        alpha = box.alpha;
        x = box.x - cos(box.alpha)*box.w/2;
        y = box.y - sin(box.alpha)*box.w/2;
    };

    void updateTargetBoxPose(double x, double y, double alpha){
        // given robot pose, update box pose
        target.x = x + cos(alpha)*target.w/2;
        target.y = y + sin(alpha)*target.w/2;
        target.alpha = alpha;
    };



    void plotEnv(){
        // Create a blank image (500x500 pixels) with a white background (color is BGR)
        cv::Mat image = cv::Mat::zeros(ImageWidth, ImageHeight, CV_8UC3);

        update(image);


        // // Create a window for display.
        // cv::namedWindow("Motion Planning", cv::WINDOW_AUTOSIZE);

        // Show our image inside it.
        cv::imshow("Robot", image);

        // Wait for a keystroke in the window
        cv::waitKey(0);

    };


    void update(cv::Mat &image) {
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


    int animation(std::vector<State> path){
        // Create a blank image (500x500 pixels) with a white background (color is BGR)
        cv::Mat image = cv::Mat::zeros(ImageWidth, ImageHeight, CV_8UC3);

        update(image);
        int num_steps = 5;
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
    double x,y,alpha;
    double a = 3.0;
    env.getGraspPose(env.target, x, y, alpha);
    cout<<"start_pose: "<<x<<", "<<y<<", "<<alpha<<endl;
    double q1,q2,q3;
    env.robot.IK(x,y,alpha,q1,q2,q3);
    env.robot.moveToJointState(q1,q2,q3);
    // env.robot.moveToJointState(-0.15*M_PI, -0.4*M_PI, M_PI/3.0);
    // std::cout << cv::getBuildInformation() << std::endl;
    // env.plotEnv();
    env.getGraspPose(env.goal, x, y, alpha);
    cout<<"goal_pose: "<<x<<", "<<y<<", "<<alpha<<endl;
    double g_q1,g_q2,g_q3;
    env.robot.IK(x,y,alpha,g_q1,g_q2,g_q3);
    
    State start(q1, q2, q3);
    State goal(g_q1, g_q2, g_q3);
    RRTConnectPlanner planner(start, goal);

    std::vector<State> path = planner.plan();
    env.animation(path);
    return 0;
}