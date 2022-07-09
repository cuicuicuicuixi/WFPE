#pragma once

#include "Solver.h"

namespace physE {


class SmoothPositionSolver
        : public Solver
{
public:
   void Solve(
       std::vector<Collision>& collisions,
       float dt)  override
    {
        std::vector<std::pair<QVector3D, QVector3D>> deltas;

        for (Collision& collision : collisions) {
            Object* aBody = collision.ObjA->IsDynamic ? collision.ObjA : nullptr;
            Object* bBody = collision.ObjB->IsDynamic ? collision.ObjB : nullptr;

            float aInvMass = aBody ? aBody->Mass : 0.0f;
            float bInvMass = bBody ? bBody->Mass : 0.0f;

            const float percent = 0.8f;
            const float slop = 0.01f;

            QVector3D correction = collision.Points.Normal * percent
                * fmax(collision.Points.Depth - slop, 0.0f)
                / (aInvMass + bInvMass);

            QVector3D deltaA;
            QVector3D deltaB;

            if (aBody ? aBody->Collider : false) {
                deltaA = aInvMass * correction;
            }

            if (bBody ? bBody->Collider : false) {
                deltaB = bInvMass * correction;
            }

            deltas.emplace_back(deltaA, deltaB);
        }

        for (unsigned i = 0; i < collisions.size(); i++) {
            Object* aBody = collisions[i].ObjA->IsDynamic ? collisions[i].ObjA : nullptr;
            Object* bBody = collisions[i].ObjB->IsDynamic ? collisions[i].ObjB : nullptr;

            if (aBody ? aBody->Collider : false) {
                aBody->Transform->Position -= deltas[i].first;
            }

            if (bBody ? bBody->Collider : false) {
                bBody->Transform->Position += deltas[i].second;
            }
        }

    }
};




}
