#include "collision.h"



bool CollisionChecker::isCollisionFree(const State &state) {
    bool res = env.isCollisionFree(state);
    // cout<<"isCollisionFree: "<<res<<endl;
    // env.plotEnv();
    return res;
}


