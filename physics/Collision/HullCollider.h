#pragma once

#include "Collider.h"

namespace physE {
namespace impl {

    struct HullCollider : Collider
    {
        std::vector<QVector3D> m_vertices;
        std::vector<int> m_index;

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

        void setData(std::vector<QVector3D> vertices, std::vector<int> index)
        {
            m_vertices = vertices;
            m_index = index;
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
            QMatrix4x4 model;
            model.translate(transform->Position);
            shaderProgram->setUniformValue("model", model);
            if(VAO.objectId() == 0)
            {
                VAO.create();
                VAO.bind();
                VBO = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
                QOpenGLBuffer ebo(QOpenGLBuffer::IndexBuffer);
                VBO.create();
                VBO.setUsagePattern(QOpenGLBuffer::DynamicDraw);
                ebo.create();
                VBO.bind();
                VBO.allocate(m_vertices.data(), m_vertices.size() * sizeof(QVector3D));
                ebo.bind();
                ebo.allocate(&m_index[0], m_index.size() * sizeof(unsigned int));

                shaderProgram->enableAttributeArray(0);
                shaderProgram->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(QVector3D));
                VAO.release();
            }
            else
            {
                VAO.bind();
                VBO.bind();
                VBO.write(0, m_vertices.data(), m_vertices.size() * sizeof(QVector3D));

                VBO.release();
                VAO.release();
            }

            QOpenGLVertexArrayObject::Binder bind(&VAO);
            //glFunc->glDrawArrays(GL_POINTS, 0, m_vertices.size());
            glFunc->glDrawElements(GL_TRIANGLES, m_index.size(), GL_UNSIGNED_INT, 0);
        }

    };
}

}


