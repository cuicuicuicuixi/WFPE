#include "physicalworld.h"
#include "Collision/DetectCollisoin.h"

namespace physE {

    void physicalworld::buildKDtree()
    {
        QVector<QVector3D> res;
        for (Object* a : qAsConst(m_objects))
        {
            res.push_back(a->Transform->Position);
        }
        tree.setInputCloud(res);
    }

    void physicalworld::ResolveCollisions(float dt)
    {
        std::vector<Collision> collisions;
        for (Object* a : qAsConst(m_objects)) {
        //for (int i = 0; i < m_objects.size(); i++) {
            //Object* a = m_objects.at(i);
            //QVector<int> kindices;
            //QVector<float> k_distance_indices;
            //buildKDtree();
            //int num = tree.RadiusSearch(a->Transform->Position, 2, kindices, k_distance_indices);
            //if(num > 1)
            {
                //for (auto&i:kindices) {
                //for (int j = 0; j < m_objects.size(); j++) {
                    //Object* b = m_objects.at(j);
                for (Object* b : qAsConst(m_objects)) {
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

        for (Object* a : qAsConst(m_objects)) {
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

        std::sort(collisions.begin(), collisions.end(), [&](Collision lhs, Collision rhs){
            if(lhs.ObjA < rhs.ObjA) return true;

            if(lhs.ObjA == rhs.ObjA)
                return lhs.ObjB < rhs.ObjB;
            return false;
        });
        std::vector<Collision> uniqueCollisions;

            int i = 0;
            while(i < collisions.size())
            {
                Collision col = collisions[i];
                uniqueCollisions.push_back(col);
                i++;
                while(i < collisions.size())
                {
                    Collision poC = collisions[i];
                    if(col.ObjA != poC.ObjB || col.ObjB != poC.ObjA) break;
                    i++;
                }
            }


        for (Solver* solver : m_solvers) {
            solver->Solve(uniqueCollisions, dt);
        }
    }
}
