# pragma once

#include <vector>
#include <QVector>
#include <QVector3D>
#include <QMatrix3x3>
#include <QMatrix4x4>
#include <stdlib.h>
#include <QDebug>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLExtraFunctions>
#include <QOpenGLTexture>
#include <QOpenGLWidget>
#include <QtOpenGLExtensions/QOpenGLExtensions>

namespace physE {

    inline float major(QVector3D& v) {
        float m = v.x();
        if (v.y() > m) m = v.y();
        if (v.z() > m) m = v.z();
        return m;
    }

    enum class ColliderType {
        PLANE,
        SPHERE,
        CAPSULE,
        HULL,
        MESH
    };



    struct Transform { // Describes an objects location
        QVector3D Position;
        QVector3D Scale;
        QMatrix4x4 Rotation;
    };

    struct VerNorm
    {
        QVector3D Vertex;
        QVector3D Norm = QVector3D();

        VerNorm(){}
        VerNorm(QVector3D ver) : Vertex(ver) {};
    };


    struct Collider {
        const ColliderType Type;

        Collider(ColliderType _type):Type(_type){}

        size_t get_type() const {
            return (size_t)Type;
        }



        virtual QVector3D FindFurthestPoint(
            Transform* transform,
            const QVector3D& direction) const = 0;

        virtual void Draw(QOpenGLFunctions_3_3_Core* glFunc, QOpenGLShaderProgram* shaderProgram, Transform* transform) = 0;
    };

}

using namespace physE;
