#pragma once

#include "Collider.h"

namespace physE {
namespace impl {

    struct HullCollider : Collider
    {
        std::vector<QVector3D> m_vertices;

        QOpenGLVertexArrayObject VAO;
        QOpenGLBuffer VBO;

        HullCollider()
            :Collider(ColliderType::HULL)
        {

        }

        void addVertex(QVector3D point)
        {
            m_vertices.push_back(point);
        }

        void setVetices(std::vector<QVector3D> vertices)
        {
            m_vertices = vertices;
        }

        QVector3D FindFurthestPoint(
                Transform *transform,
                const QVector3D &direction) const override
        {
            QVector3D furthestPoint;

            float maxDistance = INT_MIN;

            for(auto& vertex : m_vertices)
            {
                float distance = QVector3D::dotProduct(vertex, direction);
                if(distance > maxDistance)
                {
                    maxDistance = distance;
                    furthestPoint = vertex;
                }
            }

            return furthestPoint + transform->Position;
        }

        void Draw(QOpenGLFunctions_3_3_Core* glFunc, QOpenGLShaderProgram* shaderProgram, Transform* transform)  override
        {
            std::vector<QVector3D> vec(m_vertices);
            //qDebug()<<transform->Position;
            for(auto&v:vec) v += transform->Position;
            if(VAO.objectId() == 0)
            {
                VAO.create();
                VAO.bind();
                VBO = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
                VBO.create();
                VBO.setUsagePattern(QOpenGLBuffer::DynamicDraw);
                VBO.bind();
                VBO.allocate(vec.data(), vec.size() * sizeof(QVector3D));
                shaderProgram->enableAttributeArray(0);
                shaderProgram->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(QVector3D));
                VAO.release();
            }
            else
            {
                VAO.bind();
                VBO.bind();
                VBO.write(0, vec.data(), vec.size() * sizeof(QVector3D));

                VBO.release();
                VAO.release();
            }

            QOpenGLVertexArrayObject::Binder bind(&VAO);
            glFunc->glDrawArrays(GL_POINTS, 0, m_vertices.size());
        }

    };
}

}


