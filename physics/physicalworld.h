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
            vertex.push_back(QVector3D( 1, 1, -1));
            vertex.push_back(QVector3D(-1, 1, -1));
            vertex.push_back(QVector3D( 1,-1, -1));
            vertex.push_back(QVector3D(-1,-1, -1));
            vertex.push_back(QVector3D( 1, 1, 1));
            vertex.push_back(QVector3D(-1, 1, 1));
            vertex.push_back(QVector3D( 1,-1, 1));
            vertex.push_back(QVector3D(-1,-1, 1));
            std::vector<int> face{
                0,1,2,
                1,2,3,
                4,5,6,
                5,6,7,
                0,1,5,
                0,4,5,
                2,3,7,
                2,6,7,
                0,2,6,
                0,6,4,
                1,3,7,
                1,5,7
            };
            impl::HullCollider * hull = new impl::HullCollider();
            hull->setData(vertex, face);
//            srand((unsigned)time(NULL));
//            for(int i = 0; i < 5; i++)
//            {
//                double x = rand()%10-5;
//                double y = rand()%10-5;
//                double z = 0;//rand()%10-5;
//                double vx = rand()%4-2;
//                double vy = rand()%4-2;
//                double vz = 0;//rand()%4-2;
//                Object *HullObj1 = new Object(i,QVector3D(x,y,z), QVector3D(vx,vy,vz), hull, true);
//                AddObject(HullObj1);
//            }
            Object *HullObj1 = new Object(1,QVector3D(-10,0,10), QVector3D(10,0,0), hull, true);
            AddObject(HullObj1);
            Object *HullObj2 = new Object(2,QVector3D(30,0,10), QVector3D(-10,0,0), hull, true);
            AddObject(HullObj2);
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

                for (Object* obj : qAsConst(m_objects)) {
                    if(!obj->IsDynamic) continue;
                    //obj->Force += obj->Mass * m_gravity; // apply a force

                    obj->Velocity += obj->Force / obj->Mass * sub_dt;

                    obj->angularVelocity += obj->torque * (1.0f / obj->I) * sub_dt;

                    obj->Transform->Position += obj->Velocity * sub_dt;

                    obj->Force = QVector3D(0, 0, 0); // reset net force at the end
                    obj->torque = QVector3D(0, 0, 0);
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
            planeobject->Draw(glFunc, shaderProgram);
            shaderProgram->release();
        }

    };
}



#endif // PHYSICALWORLD_H
