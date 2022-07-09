#ifndef PHYSICALWORLD_H
#define PHYSICALWORLD_H

#include "Collision/Collision.h"
#include "Collision/CollisionObject.h"
#include "Collision/SphereCollider.h"
#include "Collision/PlaneCollider.h"
#include "Collision/HullCollider.h"

#include "Dynamic/ImpluseSolveer.h"
#include "Dynamic/smoothPositionSolver.h"

#include "algo/kdtree.h"
#include "algo/kdtree.cpp"

namespace physE {

    class physicalworld
    {
    public:
        int sub_step = 1;
        QVector<Object*> m_objects;
        std::vector<Solver*> m_solvers;
        QVector3D m_gravity = 2*QVector3D(0, -9.81f, 0);
        KDTree<QVector3D> tree;
        void AddObject   (Object* object) {
            m_objects.push_back(object);
        }
        void RemoveObject(Object* object) { /* ... */ }

        void AddSolver   (Solver* solver) {
            m_solvers.push_back(solver);
        }
        void RemoveSolver(Solver* solver) { /* ... */ }

        physicalworld() : tree() {}

        Object * planeobject;

        void init()
        {
            std::vector<QVector3D> vertex;
            vertex.push_back(QVector3D( 1, 1, 0));
            vertex.push_back(QVector3D(-1, 1, 0));
            vertex.push_back(QVector3D( 1,-1, 0));
            vertex.push_back(QVector3D(-1,-1, 0));
            vertex.push_back(QVector3D( 1, 1, 2));
            vertex.push_back(QVector3D(-1, 1, 2));
            vertex.push_back(QVector3D( 1,-1, 2));
            vertex.push_back(QVector3D(-1,-1, 2));
            impl::HullCollider * hull = new impl::HullCollider();
            hull->setVetices(vertex);
            srand((unsigned)time(NULL));
            for(int i = 0; i < 2; i++)
            {
                double x = rand()%10-5;
                double y = rand()%10-5;
                double z = rand()%10-5;
                double vx = rand()%4-2;
                double vy = rand()%4-2;
                double vz = rand()%4-2;
                Object *HullObj1 = new Object(i,QVector3D(x,y,z), QVector3D(vx,vy,vz), hull, true);
                AddObject(HullObj1);
            }
            impl::PlaneCollider* pco = new impl::PlaneCollider(QVector3D(0,1,0), -50.0);
            planeobject = new Object(-1, QVector3D(0,0,0), QVector3D(0, 0, 0), pco);
            AddSolver(new ImpluseSolveer());
            //AddSolver((new SmoothPositionSolver()));
        }

        void Step(
            float dt)
        {
            float sub_dt = dt/(float)sub_step;
            //qDebug()<<sub_dt;
            for(int i(sub_step); i--;)
            {
                ResolveCollisions(sub_dt);

                for (Object* obj : m_objects) {
                    if(!obj->IsDynamic) continue;
                    obj->Force += obj->Mass * m_gravity; // apply a force

                    obj->Velocity += obj->Force / obj->Mass * sub_dt;
                    //obj->Position += obj->Velocity * sub_dt;

                    //qDebug()<<obj->Velocity;
                    obj->Transform->Position += obj->Velocity * sub_dt;

                    // constraint
//                    if(obj->Transform->Position.y() <= -49.5)
//                    {
//                        obj->Transform->Position.setY(-49.5);
//                        //obj->Transform->Position.setY(-49.5);
//                    }

                    obj->Force = QVector3D(0, 0, 0); // reset net force at the end
                }

            }
        }

        void buildKDtree();
        void ResolveCollisions(float dt);

        void Draw(QOpenGLFunctions_3_3_Core* glFunc, QOpenGLShaderProgram* shaderProgram)
        {
            shaderProgram->bind();
            for(auto& obj : m_objects)
            {
                obj->Draw(glFunc, shaderProgram);
            }
            shaderProgram->release();
        }

    };
}



#endif // PHYSICALWORLD_H
