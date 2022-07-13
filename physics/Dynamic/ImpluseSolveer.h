#pragma once

#include "Solver.h"

namespace physE {

class ImpluseSolveer
        : public Solver
{
public:
    void Solve(
        std::vector<Collision>& collisions,
        float dt)  override
    {
        for (Collision& collision : collisions) {

            //qDebug()<<collision.Points.Depth;
            QVector3D ra = QVector3D();
            QVector3D rb = QVector3D();
            if(collision.Points.ContactPoint != QVector3D())
                ra = /*QVector3D();//*/collision.Points.ContactPoint - collision.ObjA->Transform->Position;
                rb = /*QVector3D();//*/collision.Points.ContactPoint - collision.ObjB->Transform->Position;

            Object* aBody = collision.ObjA->IsDynamic ? collision.ObjA : nullptr;
            Object* bBody = collision.ObjB->IsDynamic ? collision.ObjB : nullptr;

            QVector3D aVel = aBody? aBody->Velocity:QVector3D(0,0,0);
            QVector3D bVel = bBody? bBody->Velocity:QVector3D(0,0,0);

            QVector3D aAngVel = aBody? aBody->angularVelocity:QVector3D(0,0,0);
            QVector3D bAngVel = bBody? bBody->angularVelocity:QVector3D(0,0,0);

            QVector3D rVel = bVel - aVel + QVector3D::crossProduct(bAngVel, rb) - QVector3D::crossProduct(aAngVel, ra);

            float  nSpd = QVector3D::dotProduct(rVel, collision.Points.Normal);
            if (nSpd >= 0)
                continue;

            float aMass = aBody? aBody->Mass:0;
            float bMass = bBody? bBody->Mass:0;
            float inv_massA = aMass? 1 / aMass:0;
            float inv_massB = bMass? 1 / bMass:0;

            float aI = aBody? aBody->I:0;
            float bI = bBody? bBody->I:0;
            float inv_iA = aI? 1 / aI:0;
            float inv_iB = bI? 1 / bI:0;

            QVector3D raCrossN = QVector3D::crossProduct( ra, collision.Points.Normal );
            QVector3D rbCrossN = QVector3D::crossProduct( rb, collision.Points.Normal );
            float invMassSum =inv_massA + inv_massB + sqrt( raCrossN.length() ) * inv_iA + sqrt( rbCrossN.length() ) * inv_iB;


            float e = (aBody ? .8 : 1.0f)
                    * (bBody ? .8 : 1.0f);
            float j = -(1.0f + e) * nSpd / invMassSum;

            QVector3D impluse = j * collision.Points.Normal;

            if (aBody) {
                aVel -= impluse * inv_massA;
                aAngVel += inv_iA * QVector3D::crossProduct(ra, -impluse);
            }

            if (bBody) {
                bVel += impluse * inv_massB;
                bAngVel += inv_iB * QVector3D::crossProduct(rb, impluse);
            }

            // Friction

            rVel = bVel - aVel + QVector3D::crossProduct(bAngVel, rb) - QVector3D::crossProduct(aAngVel, ra);
            nSpd = QVector3D::dotProduct(rVel, collision.Points.Normal);

            QVector3D tangent = rVel - nSpd * collision.Points.Normal;

            if ((tangent).length() > 0.0001f) { // safe normalize
                tangent = tangent.normalized();
            }

            float fVel = QVector3D::dotProduct(rVel, tangent);

            float aSF = aBody ? .8  : 0.0f;
            float bSF = bBody ? .8  : 0.0f;
            float aDF = aBody ? .8 : 0.0f;
            float bDF = bBody ? .8 : 0.0f;
            float mu  = (float)QVector2D(aSF, bSF).length();

            float f  = -fVel / invMassSum;

            QVector3D friction;
            if (abs(f) < j * mu) {
                friction = f * tangent;
            }

            else {
                mu = QVector2D(aDF, bDF).length();
                friction = -j * tangent * mu;
            }



            const float percent = 0.2f; // usually 20% to 80%
            const float slop = 0.01f; // usually 0.01 to 0.1

            QVector3D correction = std::max( collision.Points.Depth - slop, 0.0f ) / (inv_massA + inv_massB) * percent * collision.Points.Normal;
            //qDebug()<<collision.Points.Depth<<", "<<collision.Points.Normal;
            if(aBody)
            {
                aBody->Velocity = aVel - friction * aMass;
                aBody->angularVelocity = aAngVel + inv_iA * QVector3D::crossProduct(ra, -friction);
                aBody->Transform->Position -= inv_massA * correction;
            }

            if(bBody)
            {
                bBody->Velocity = bVel + friction * bMass;
                bBody->angularVelocity = bAngVel + inv_iB * QVector3D::crossProduct(rb, friction);
                bBody->Transform->Position += inv_massB * correction;
            }
        }
    }

};

}
