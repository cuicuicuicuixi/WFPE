#pragma once

#include "Collider.h"

namespace physE {
namespace impl {
    struct PlaneCollider
        : Collider
    {
        QVector3D Normal;
        float Distance;

        PlaneCollider()
            : Collider(ColliderType::PLANE)
            , Normal()
            , Distance(1.0f)
        {

        }

        PlaneCollider(QVector3D normal, float distance)
            : Collider(ColliderType::PLANE)
            , Normal(normal)
            , Distance(distance)
        {

        }

        QVector3D FindFurthestPoint(
            Transform* transform,
            const QVector3D&     direction) const override
        {
            assert(false);
            return QVector3D(0, 0, 0);
        }

        void Draw(QOpenGLFunctions_3_3_Core* glFunc, QOpenGLShaderProgram* shaderProgram, Transform* transform) override
        {

        }
    };
}

}
using namespace physE;
