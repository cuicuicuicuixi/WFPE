#pragma once

#include "physics/Collision/Collision.h"

namespace physE {

class Solver {
public:
    virtual void Solve(
        std::vector<Collision>& collisions,
        float dt) = 0;
};

}
