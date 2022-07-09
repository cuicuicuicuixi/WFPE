#include "physicalworld.h"
#include "Collision/DetectCollisoin.h"

namespace physE {

    void physicalworld::buildKDtree()
    {
        QVector<QVector3D> res;
        for (Object* a : m_objects)
        {
            res.push_back(a->Transform->Position);
        }
        tree.setInputCloud(res);
    }

    void physicalworld::ResolveCollisions(float dt)
    {
        std::vector<Collision> collisions;
        for (Object* a : m_objects) {
            QVector<int> kindices;
            QVector<float> k_distance_indices;
            buildKDtree();
            int num = tree.RadiusSearch(a->Transform->Position, 2, kindices, k_distance_indices);
            //if(num > 1)
            {
                //for (auto&i:kindices) {
                for (int i = 0; i < m_objects.size(); i++) {
                    Object* b = m_objects.at(i);
                    if (a == b) continue;

                    if (    !a->Collider
                        || !b->Collider)
                    {
                        continue;
                    }

                    CollisionPoints points = impl::DetectCollision(
                        a->Collider,
                        a->Transform,
                        b->Collider,
                        b->Transform);

                    if (points.HasCollision==true) {
                        //qDebug()<<a->Position<<" "<<b->Position;
                        collisions.emplace_back(a, b, points);
                    }
                }
            }
        }

        for (Object* a : m_objects) {
            CollisionPoints points = impl::DetectCollision(
                a->Collider,
                a->Transform,
                planeobject->Collider,
                planeobject->Transform);

            if (points.HasCollision==true) {
                //qDebug()<<a->Position<<" "<<planeobject->Position;
                collisions.emplace_back(a, planeobject, points);
            }
        }

        for (Solver* solver : m_solvers) {
            solver->Solve(collisions, dt);
        }
    }
}
