#ifndef COLLISION_H
#define COLLISION_H

#include "env.h"
#include "vec2.h"


class CollisionChecker {
private:
    Environment env;
public:
    CollisionChecker(const Environment &env) : env(env) {}

    bool isCollisionFree(const State &state);
};

#endif // COLLISION_H