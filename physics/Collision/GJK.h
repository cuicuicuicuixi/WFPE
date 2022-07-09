#pragma once

#include "Collision.h"
#include <array>
#include <initializer_list>
#include <utility>

namespace physE
{
namespace impl
{
struct Simplex;


    struct Simplex
    {
        std::array<QVector3D, 4> Vertices;
        size_t m_size;

        Simplex()
            : Vertices()
            , m_size(0)
        {

        }

        Simplex& operator=(
                std::initializer_list<QVector3D> list)
        {
            for (auto v = list.begin(); v != list.end(); v++) {
                Vertices[std::distance(list.begin(), v)] = *v;
            }
            m_size = list.size();

            return *this;
        }

        void push_front(
                QVector3D vertex)
        {
            Vertices = {vertex, Vertices[0], Vertices[1], Vertices[2]};
            m_size = std::min<size_t>(m_size + 1, 4u);
        }

        QVector3D& operator[](unsigned i) { return Vertices[i]; }
        size_t size() const {return m_size;}

        auto begin() const { return Vertices.begin(); }
        auto end()   const { return Vertices.end() - (4u - m_size); }
    };



    QVector3D Support(
            Collider* colliderA, Transform* transformA,
            Collider* colliderB, Transform* transformB,
            QVector3D direction);
//    {
//        return colliderA->FindFurthestPoint(transformA, direction)
//                - colliderB->FindFurthestPoint(transformB, -direction);
//    }

    bool NextSimplex(Simplex &vertices, QVector3D &direction);


    CollisionPoints EPA(
            const Simplex & simplex,
            Collider* colliderA, Transform* transformA,
            Collider* colliderB, Transform* transformB);


    std::pair<bool, Simplex> GJK(
            Collider* colliderA, Transform* transformA,
            Collider* colliderB, Transform* transformB);
//    {
//        QVector3D support = Support(
//                    colliderA, transformA,
//                    colliderB, transformB, QVector3D(1,0,0));

//        Simplex vertices;
//        vertices.push_front(support);

//        QVector3D direction = -support;

//        size_t iterations = 0;
//        while(iterations++ < 32u)
//        {
//            support = Support(
//                colliderA, transformA,
//                colliderB, transformB, direction);

//            // 下一个 support points 是否 “穿过” 原点
//            if (QVector3D::dotProduct(support, direction) <= 0) {
//                break;
//            }

//            vertices.push_front(support);


//            if (NextSimplex(vertices, direction)) {
//                return std::make_pair(true, vertices);
//            }

//        }

//        return {false, vertices};
//    }


    bool SameDirection(
            const QVector3D& direction,
            const QVector3D& ao);
//    {
//        return QVector3D::dotProduct(direction, ao) > 0;
//    }



}
}
using namespace  physE;



