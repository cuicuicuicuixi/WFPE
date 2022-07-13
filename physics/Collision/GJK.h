#pragma once

#include "Collision.h"
#include <array>
#include <initializer_list>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

namespace physE
{
namespace impl
{
struct Simplex;



    class SupportPoint {
    public:
        QVector3D C;
        QVector3D A;
        QVector3D B;

        SupportPoint():C(),A(),B(){}

        SupportPoint(QVector3D c, QVector3D a, QVector3D b)
            : C(c), A(a), B(b)
        {

        }

        SupportPoint(const SupportPoint& b)
        {
            C = b.C;
            A = b.A;
            B = b.B;
        }

        SupportPoint& operator =(const SupportPoint& b)
        {
            C = b.C;
            A = b.A;
            B = b.B;
            return *this;
        }
    };

    struct Simplex
    {
        std::array<SupportPoint, 4> Vertices;
        size_t m_size;

        Simplex()
            : Vertices()
            , m_size(0)
        {

        }

        Simplex& operator=(
                std::initializer_list<SupportPoint> list)
        {
            for (auto v = list.begin(); v != list.end(); v++) {
                Vertices[std::distance(list.begin(), v)] = *v;
            }
            m_size = list.size();

            return *this;
        }

        void push_front(
                SupportPoint vertex)
        {
            Vertices = {vertex, Vertices[0], Vertices[1], Vertices[2]};
            m_size = std::min<size_t>(m_size + 1, 4u);
        }

        SupportPoint& operator[](unsigned i) { return Vertices[i]; }
        size_t size() const {return m_size;}

        auto begin() const { return Vertices.begin(); }
        auto end()   const { return Vertices.end() - (4u - m_size); }
    };

    SupportPoint Support(
            Collider* colliderA, Transform* transformA,
            Collider* colliderB, Transform* transformB,
            QVector3D direction);

    bool NextSimplex(Simplex &vertices, QVector3D &direction);


    CollisionPoints EPA(
            const Simplex & simplex,
            Collider* colliderA, Transform* transformA,
            Collider* colliderB, Transform* transformB);


    std::pair<bool, Simplex> GJK(
            Collider* colliderA, Transform* transformA,
            Collider* colliderB, Transform* transformB);

    bool SameDirection(
            const QVector3D& direction,
            const QVector3D& ao);



}
}
using namespace  physE;



