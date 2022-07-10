#pragma once

#include "Collider.h"

namespace physE {
namespace impl {
    struct PlaneCollider
        : Collider
    {
        QVector3D Normal;
        float Distance;

        std::vector<QVector3D> m_vertices{
            QVector3D(1,0,1),
            QVector3D(1,0,-1),
            QVector3D(-1,0,1),
            QVector3D(-1,0,-1)
        };
        std::vector<int> m_index
        {
            0,1,2,
            1,2,3
        };
        QOpenGLVertexArrayObject VAO;
        QOpenGLBuffer VBO;

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
            QMatrix4x4 model;
            model.translate(transform->Position+this->Distance*Normal);
            model.scale(10);
            shaderProgram->setUniformValue("model", model);
            if(VAO.objectId() == 0)
            {
                VAO.create();
                VAO.bind();
                VBO = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
                QOpenGLBuffer ebo(QOpenGLBuffer::IndexBuffer);
                VBO.create();
                ebo.create();
                VBO.bind();
                VBO.allocate(m_vertices.data(), m_vertices.size() * sizeof(QVector3D));
                ebo.bind();
                ebo.allocate(&m_index[0], m_index.size() * sizeof(unsigned int));

                shaderProgram->enableAttributeArray(0);
                shaderProgram->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(QVector3D));
                VAO.release();
            }

            QOpenGLVertexArrayObject::Binder bind(&VAO);
            //glFunc->glDrawArrays(GL_POINTS, 0, m_vertices.size());
            glFunc->glDrawElements(GL_TRIANGLES, m_index.size(), GL_UNSIGNED_INT, 0);
        }
    };
}

}
using namespace physE;
