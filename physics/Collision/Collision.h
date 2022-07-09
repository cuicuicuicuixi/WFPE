#pragma once

#include "CollisionPoints.h"
#include "CollisionObject.h"

namespace physE {
    struct Collision {
        Object* ObjA;
        Object* ObjB;
        CollisionPoints Points;

        Collision(Object* _ObjA, Object* _ObjB, CollisionPoints _Points): ObjA(_ObjA), ObjB(_ObjB), Points(_Points)
        {

        }
    };
}
