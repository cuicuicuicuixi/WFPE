#pragma once

#include "CollisionPoints.h"
#include "SphereCollider.h"
#include "PlaneCollider.h"
#include "HullCollider.h"
#include "GJK.h"

namespace physE {
namespace impl {
    using Detect_Collision_func = CollisionPoints(*)(
        Collider*, Transform*,
        Collider*, Transform*);

    CollisionPoints Test_Plane_Sphere(
        Collider* a, Transform* at,
        Collider* b, Transform* bt)
    {
        using Plane = PlaneCollider;
        using Sphere = SphereCollider;

        Plane* A = (Plane*)a;
        Sphere* B = (Sphere*)b;

        QVector3D  aCenter = B->Center + bt->Position;
        float aRadius = B->Radius;

        QVector3D normal  = A->Normal.normalized();
        QVector3D onPlane = normal * A->Distance + at->Position;
        float distance = QVector3D::dotProduct(aCenter - onPlane, normal); // distance from center of sphere to plane surface
        if (distance > aRadius) {
            return CollisionPoints();
        }

        CollisionPoints res;
        QVector3D aDeep = aCenter - normal * aRadius;
        QVector3D bDeep = aCenter - normal * distance;

        res.A = aDeep;
        res.B = bDeep;
        res.Depth = distance - aRadius;
        res.Normal = normal;
        res.HasCollision = true;
        return res;
    }

    CollisionPoints Test_Sphere_Sphere(
        Collider* a, Transform* at,
        Collider* b, Transform* bt)
    {

        using Sphere = SphereCollider;

        Sphere* A = (Sphere*)a;
        Sphere* B = (Sphere*)b;

        CollisionPoints res;
        QVector3D acenter = A->Center + at->Position;
        QVector3D bcenter = B->Center + bt->Position;
        QVector3D ab = bcenter - acenter;
        float distance = ab.length();
        if(distance <= A->Radius + B->Radius)
        {
            //qDebug()<<distance<<"<="<<Radius<<" + "<<sphere->Radius;
            res.Normal = ab.normalized();
            res.A = acenter + A->Radius*res.Normal;
            res.B = bcenter - B->Radius*res.Normal;
            res.Depth = (res.A - res.B).length();
            res.HasCollision = true;
        }
        return res;
    }

    CollisionPoints Test_Plane_Hull(
        Collider* a, Transform* at,
        Collider* b, Transform* bt)
    {

        assert( a->Type == ColliderType::PLANE
                && ( b->Type == ColliderType::HULL));

        using Plane = PlaneCollider;
        using Hull  = HullCollider;

        Plane*  A = (Plane*)a;
        Hull*   B = (Hull*) b;

        // 平板 A 的法线， 需要 at 中的 rotate， 此处未加
        QVector3D normal = A->Normal.normalized();

        QVector3D plane = normal * A->Distance + at->Position;
        QVector3D bDeep = B->FindFurthestPoint(bt, -normal);

        QVector3D ba = plane - bDeep;

        float distance = QVector3D::dotProduct(ba, normal);

        if (distance < 0) {
            return CollisionPoints();
        }

        //qDebug() << distance;
        // Might nudge 'plane' twoards bDeep (furthest point of plane in B)
        return CollisionPoints(plane, bDeep, normal, distance, true);

    }

    CollisionPoints Test_GJK(
        Collider* a, Transform* at,
        Collider* b, Transform* bt)
    {
        Collider* A = (Collider*)a;
        Collider* B = (Collider*)b;

        //auto [collision, simplex] = GJK(A, at, B, bt);
        auto Pair = GJK(A, at, B, bt);
        bool collision = std::get<0>(Pair);
        Simplex simplex = std::get<1>(Pair);


        if(collision)
        {
            qDebug()<<"111111111111";
            for(int i = 0; i < 4; i++)
            {
                qDebug()<<simplex[i];
            }
            qDebug()<<"111111111111";
            return EPA(simplex, A, at, B, bt);
        }
        return CollisionPoints();
    }


    struct Detec_Collision_funcs
    {
        Detect_Collision_func test[5][5] = {
            {nullptr, Test_Plane_Sphere,  nullptr, Test_Plane_Hull, nullptr},
            {nullptr, Test_Sphere_Sphere, nullptr, Test_GJK,        nullptr},
            {nullptr, nullptr,            nullptr, Test_GJK,        nullptr},
            {nullptr, nullptr,            nullptr, Test_GJK,        nullptr},
            {nullptr, nullptr,            nullptr, nullptr,         nullptr},
        };
    };




    CollisionPoints DetectCollision(
        Collider*a, Transform *at,
        Collider*b, Transform *bt)
    {
        size_t atype = a->get_type();
        size_t btype = b->get_type();

        bool swap = atype > btype;

        if (swap) {
            size_t tid = atype;
            atype = btype;
            btype = tid;

            Collider* tc = a;
            a = b;
            b = tc;

            Transform* tt = at;
            at = bt;
            bt = tt;
        }

        CollisionPoints res;

        Detec_Collision_funcs DCF;
        auto& func = DCF.test[atype][btype];

        if (func) {
            res = func(a, at, b, bt);
        }

        if (swap && res.HasCollision)
        {
            res.SwapPoints();
        }

        return res;

    }
}

}
