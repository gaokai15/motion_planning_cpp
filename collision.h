#ifndef COLLISION_H
#define COLLISION_H

#include "env.h"


class CollisionChecker {
public:
    bool isCollisionFree(const State &state);
};

#endif // COLLISION_H