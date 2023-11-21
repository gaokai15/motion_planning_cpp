#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "env.h"
#include "rrt_connect.h"
#include "rrt_star.h"

using json = nlohmann::json;

int main(){
    // Open the configuration file
    ifstream config_stream("config.json");
    if (!config_stream.is_open()) {
        cerr << "Unable to open configuration file." << endl;
        return 1;
    }

    // Parse the JSON
    json config;
    try {
        config_stream >> config;
    } catch (json::parse_error& e) {
        cerr << "JSON parse error: " << e.what() << endl;
        return 1;
    }

    // Close the file as it's no longer needed
    config_stream.close();

    // Extract the vectors
    vector<double> cf_target = config["target"].get<vector<double>>();
    vector<vector<double>> cf_obs_list = config["obs"].get<vector<vector<double>>>();
    vector<double> cf_goal = config["goal"].get<vector<double>>();
    double cf_link_length = config["link_length"].get<double>();
    double cf_link_width = config["link_width"].get<double>();
    double cf_box_width = config["box_width"].get<double>();
    double cf_box_height = config["box_height"].get<double>();
    double cf_clearance = config["clearance"].get<double>();

    Environment env = Environment();
    env.setUpEnv(
        cf_target,
        cf_obs_list,
        cf_goal,
        cf_link_length,
        cf_link_width,
        cf_box_width,
        cf_box_height,
        cf_clearance
    );
    double x,y,alpha;
    env.getGraspPose(env.target, x, y, alpha);
    double q1,q2,q3;
    env.robot.IK(x,y,alpha,q1,q2,q3);
    env.robot.moveToJointState(q1,q2,q3);
    env.getGraspPose(env.goal, x, y, alpha);
    double g_q1,g_q2,g_q3;
    env.robot.IK(x,y,alpha,g_q1,g_q2,g_q3);
    CollisionChecker collisionCheckerInstance(env);
    State start(q1, q2, q3);
    State goal(g_q1, g_q2, g_q3);
    std::vector<State> path;
    bool ChooseRRTStar = false;
    if(ChooseRRTStar){
        unsigned int max_iter = 20000;
        double step_size = 0.05;
        double search_radius = 0.1;

        RRTStar rrt( start, goal, max_iter, step_size, search_radius, collisionCheckerInstance);
        rrt.Generate();
        path = rrt.GetPath();
    }
    else{
        RRTConnectPlanner planner(start, goal, collisionCheckerInstance);
        path = planner.plan();
    }
    path.push_back(State(g_q1, g_q2, g_q3));
    env.animation(path);
    return 0;
}
