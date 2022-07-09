#pragma once

#include "Collider.h"

namespace physE {
namespace impl {

    struct SphereCollider
        : Collider
    {
    public:
        QVector3D Center;
        float Radius;

        SphereCollider()
            : Collider(ColliderType::SPHERE)
            , Center()
            , Radius(1.0f)
        {

        }

        SphereCollider(QVector3D center, float radius)
            : Collider(ColliderType::SPHERE)
            , Center(center)
            , Radius(radius)
        {

        }

        QVector3D FindFurthestPoint(Transform* transform,
            const QVector3D& direction) const override
        {
            return Center + transform->Position
                + Radius * direction.normalized() * major(transform->Scale);
        }

        void Draw(QOpenGLFunctions_3_3_Core* glFunc, QOpenGLShaderProgram* shaderProgram, Transform* transform) override
        {

        }
    };

}
}
using namespace physE;
