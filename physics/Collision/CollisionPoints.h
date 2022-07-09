#pragma once

#include <QVector3D>
#include <QVector2D>

namespace physE
{
    struct CollisionPoints {
        QVector3D A; // Furthest point of A into B
        QVector3D B; // Furthest point of B into A
        QVector3D Normal; // B – A normalized
        float Depth;    // Length of B – A
        bool HasCollision;

        CollisionPoints()
            : A(), B(), Normal(), Depth()
        {}

        CollisionPoints(QVector3D a, QVector3D b, QVector3D normal, float distance, bool hasCollision)
            : A(a), B(b), Normal(normal), Depth(distance), HasCollision(hasCollision)
        {}

        void SwapPoints()
        {
            QVector3D tmp = A;
            A = B;
            B = tmp;
            Normal = -Normal;
        }
    };
}

