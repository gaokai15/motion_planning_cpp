#include <iostream>
#include "env.h"
#include "rrt_connect.h"

int main(){
    Environment env = Environment();
    env.setUpEnv();
    double x,y,alpha;
    double a = 1.0;
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
    CollisionChecker collisionCheckerInstance(env);
    RRTConnectPlanner planner(start, goal, &collisionCheckerInstance);

    std::vector<State> path = planner.plan();
    env.animation(path);
    return 0;
}
