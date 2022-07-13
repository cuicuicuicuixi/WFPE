#pragma once

#include "Collider.h"
#include <new>

namespace physE {
namespace impl {

    struct HullCollider : Collider
    {
        std::vector<VerNorm> m_data;
        std::vector<int> m_index;

        QOpenGLVertexArrayObject VAO;
        QOpenGLBuffer VBO;

        HullCollider()
            :Collider(ColliderType::HULL)
        {

        }

        void setData(std::vector<QVector3D> vertices, std::vector<int> index)
        {
            m_index = index;
            m_data.resize(vertices.size());
            ComputeNormal(vertices);
        }

        bool ComputeNormal(std::vector<QVector3D> vertices)
        {
            unsigned verCt = vertices.size();
            std::vector<QVector3D> Norms;
            try {
                Norms.resize(verCt, QVector3D(0, 0, 0));
            } catch (const std::bad_alloc&) {
                return false;
            }
            for(int i = 0; i < m_index.size() / 3; i+=3)
            {
                const QVector3D A = vertices[m_index[i    ]];
                const QVector3D B = vertices[m_index[i + 1]];
                const QVector3D C = vertices[m_index[i + 2]];

                //compute face normal (right hand rule)
                QVector3D N = QVector3D::crossProduct(B - A, C - A);
                Norms[m_index[i    ]] += N;
                Norms[m_index[i + 1]] += N;
                Norms[m_index[i + 2]] += N;
            }

            for(int i = 0; i < verCt; i++)
            {
                Norms[i].normalize();
                if(QVector3D::dotProduct(Norms[i], vertices[i]) < 0)
                    Norms[i] = -Norms[i];
                m_data[i].Vertex = vertices[i];
                m_data[i].Norm = Norms[i];
            }
            return true;
        }

        QVector3D FindFurthestPoint(
                Transform *transform,
                const QVector3D &direction) const override
        {
            QVector3D furthestPoint;

            float maxDistance = INT_MIN;

            for(auto& point : m_data)
            {
                float distance = QVector3D::dotProduct(transform->Rotation*point.Vertex, direction);
                if(distance > maxDistance)
                {
                    maxDistance = distance;
                    furthestPoint = point.Vertex;
                }
            }

            return transform->Rotation*furthestPoint + transform->Position;
        }

        void Draw(QOpenGLFunctions_3_3_Core* glFunc, QOpenGLShaderProgram* shaderProgram, Transform* transform)  override
        {
            //QMatrix4x4 model = transform->Rotation;
            QMatrix4x4 model;
            model.translate(transform->Position);
            model *= transform->Rotation;
            shaderProgram->setUniformValue("model", model);
            if(VAO.objectId() == 0)
            {
                VAO.create();
                VAO.bind();
                VBO = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
                QOpenGLBuffer ebo(QOpenGLBuffer::IndexBuffer);
                VBO.create();
                //VBO.setUsagePattern(QOpenGLBuffer::DynamicDraw);
                ebo.create();
                VBO.bind();
                VBO.allocate(m_data.data(), m_data.size() * sizeof(VerNorm));
                ebo.bind();
                ebo.allocate(&m_index[0], m_index.size() * sizeof(unsigned int));

                shaderProgram->enableAttributeArray(0);
                shaderProgram->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(VerNorm));
                shaderProgram->enableAttributeArray(1);
                shaderProgram->setAttributeBuffer(1, GL_FLOAT, offsetof(VerNorm, Norm), 3, sizeof(VerNorm));
                VAO.release();
            }
//            else
//            {
//                VAO.bind();
//                VBO.bind();
//                VBO.write(0, m_data.data(), m_data.size() * sizeof(VerNorm));

//                VBO.release();
//                VAO.release();
//            }

            QOpenGLVertexArrayObject::Binder bind(&VAO);
            //glFunc->glDrawArrays(GL_POINTS, 0, m_vertices.size());
            glFunc->glDrawElements(GL_TRIANGLES, m_index.size(), GL_UNSIGNED_INT, 0);
        }

    };
}

}


