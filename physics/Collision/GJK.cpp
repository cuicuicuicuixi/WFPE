#include "GJK.h"

namespace physE {

namespace impl {

    std::pair<bool, Simplex> GJK(
            Collider* colliderA, Transform* transformA,
            Collider* colliderB, Transform* transformB)
    {
        SupportPoint support = Support(
                    colliderA, transformA,
                    colliderB, transformB, QVector3D(1,0.1,0).normalized());

        Simplex vertices;
        vertices.push_front(support);

        QVector3D direction = -support.C;

        size_t iterations = 0;
        while(iterations++ < 32u)
        {
            support = Support(
                colliderA, transformA,
                colliderB, transformB, direction);

            // 下一个 support points 是否 “穿过” 原点
            if (QVector3D::dotProduct(support.C, direction) <= 0) {
                break;
            }

            vertices.push_front(support);


            if (NextSimplex(vertices, direction)) {
                return std::make_pair(true, vertices);
            }

        }

        return {false, vertices};
    }

    SupportPoint Support(
            Collider* colliderA, Transform* transformA,
            Collider* colliderB, Transform* transformB,
            QVector3D direction)
    {
        SupportPoint P;
        P.A = colliderA->FindFurthestPoint(transformA, direction);
        P.B = colliderB->FindFurthestPoint(transformB, -direction);
        P.C = P.A - P.B;
        return P;
    }

    bool SameDirection(
            const QVector3D& direction,
            const QVector3D& ao)
    {
        return QVector3D::dotProduct(direction, ao) > 0;
    }

    bool Line(
            Simplex& vertices,
            QVector3D& direction)
    {
        SupportPoint a = vertices[0];
        SupportPoint b = vertices[1];

        QVector3D ab = b.C - a.C;
        QVector3D ao =   - a.C;

        if(SameDirection(ab, ao))
        {
            // 计算 ab 上指向原点的垂线向量
            direction = QVector3D::crossProduct(QVector3D::crossProduct(ab, ao), ab);
        }
        else
        {
            vertices = {a};
            direction = ao;
        }

        return false;
    }

    bool Triangle(
            Simplex& vertices,
            QVector3D& direction)
    {
        SupportPoint a = vertices[0];
        SupportPoint b = vertices[1];
        SupportPoint c = vertices[2];

        QVector3D ab = b.C - a.C;
        QVector3D ac = c.C - a.C;
        QVector3D ao =   - a.C;

        QVector3D abc = QVector3D::crossProduct(ab, ac);

        if (SameDirection(QVector3D::crossProduct(abc, ac), ao)) {
            if (SameDirection(ac, ao)) {
                vertices = { a, c };
                direction = QVector3D::crossProduct(QVector3D::crossProduct(ac, ao), ac);
            }

            else {
                return Line(vertices = { a, b }, direction);
            }
        }

        else {
            if (SameDirection(QVector3D::crossProduct(ab, abc), ao)) {
                return Line(vertices = { a, b }, direction);
            }

            else {
                if (SameDirection(abc, ao)) {
                    direction = abc;
                }

                else {
                    vertices = { a, c, b };
                    direction = -abc;
                }
            }
        }

        return false;
    }

    bool Tetrahedron(
            Simplex& vertices,
            QVector3D& direction)
    {
        SupportPoint a = vertices[0];
        SupportPoint b = vertices[1];
        SupportPoint c = vertices[2];
        SupportPoint d = vertices[3];

        QVector3D ab = b.C - a.C;
        QVector3D ac = c.C - a.C;
        QVector3D ad = d.C - a.C;
        QVector3D ao =   - a.C;

        QVector3D abc = QVector3D::crossProduct(ab, ac);
        QVector3D acd = QVector3D::crossProduct(ac, ad);
        QVector3D adb = QVector3D::crossProduct(ad, ab);

        if (SameDirection(abc, ao)) return Triangle(vertices = { a, b, c }, direction);
        if (SameDirection(acd, ao)) return Triangle(vertices = { a, c, d }, direction);
        if (SameDirection(adb, ao)) return Triangle(vertices = { a, d, b }, direction);

        return true;

    }

    bool NextSimplex(
            Simplex& vertices,
            QVector3D& direction)
    {
        switch (vertices.size()) {
        case 2: return        Line(vertices, direction);
        case 3: return    Triangle(vertices, direction);
        case 4: return Tetrahedron(vertices, direction);
        }

        return false;
    }


    // 计算每个平面的法线以及到原点距离， 以及最小距离平面的索引
    std::pair<std::vector<QVector4D>, size_t> GetFaceNormals(
            const std::vector<SupportPoint>&   polytope,
            const std::vector<size_t>&      faces)
    {
        std::vector<QVector4D> normals;
        size_t minTriangle = 0;
        float minDistance = FLT_MAX;

        for(size_t i = 0; i < faces.size(); i+= 3u)
        {
            QVector3D a = polytope[faces[i  ]].C;
            QVector3D b = polytope[faces[i+1]].C;
            QVector3D c = polytope[faces[i+2]].C;

            QVector3D normal = QVector3D::crossProduct(b - a, c - a).normalized();
            float distance = QVector3D::dotProduct(normal, a);

            if(distance < 0)
            {
                normal   *= -1;
                distance *= -1;
            }

            normals.emplace_back(normal, distance);

            if(distance < minDistance)
            {
                minTriangle = i / 3;
                minDistance = distance;
            }
        }
        //qDebug()<<normals[minTriangle];
        //qDebug()<<polytope[faces[3*minTriangle]]<<", "<<polytope[faces[3*minTriangle+1]]<<", "<<polytope[faces[3*minTriangle+2]];

        return { normals, minTriangle };
    }

    void AddIfUniqueEdge(
            std::vector<std::pair<size_t, size_t>>& edges,
            const std::vector<size_t>& faces,
            size_t a,
            size_t b)
    {
        auto reverse = std::find(              //      0--<--3
            edges.begin(),                     //     / \ B /   A: 2-0
            edges.end(),                       //    / A \ /    B: 0-2
            std::make_pair(faces[b], faces[a]) //   1-->--2
        );

        if (reverse != edges.end()) {
            edges.erase(reverse);
        }

        else {
            edges.emplace_back(faces[a], faces[b]);
        }
    }


    CollisionPoints EPA(
            const Simplex & simplex,
            Collider* colliderA, Transform* transformA,
            Collider* colliderB, Transform* transformB)
    {
        std::vector<SupportPoint> polytope(simplex.begin(), simplex.end());
        std::vector<size_t> faces = {
            0,  1,  2,
            0,  3,  1,
            0,  2,  3,
            1,  3,  2
        };

        auto P = GetFaceNormals(polytope, faces);
        std::vector<QVector4D>  normals = std::get<0>(P);
        size_t                  minFace = std::get<1>(P);

        QVector3D minNormal;
        float minDistance = FLT_MAX;

        size_t iterations = 0;
        while (minDistance == FLT_MAX)
        {
            minNormal   = normals[minFace].toVector3D();
            minDistance = normals[minFace].w();

            if(iterations++ > 32) break;

            SupportPoint support = Support(colliderA, transformA, colliderB, transformB, minNormal);
            float sDistance = QVector3D::dotProduct(minNormal, support.C);

            if(std::abs(sDistance - minDistance) > 0.001f)
            {
                minDistance = FLT_MAX;

                std::vector<std::pair<size_t, size_t>> uniqueEdges;

                for(size_t i = 0; i < normals.size(); i++)
                {
                    // 判断下一个支撑点 “穿过” 原点
                    if(SameDirection(normals[i].toVector3D(), support.C))
                    {
                        size_t f = i * 3;

                        AddIfUniqueEdge(uniqueEdges, faces, f,     f + 1);
                        AddIfUniqueEdge(uniqueEdges, faces, f + 1, f + 2);
                        AddIfUniqueEdge(uniqueEdges, faces, f + 2, f    );

                        //将 face 以及 normals 尾部数据交换到已经处理的位置 然后删除队尾
                        faces[f + 2] = faces.back(); faces.pop_back();
                        faces[f + 1] = faces.back(); faces.pop_back();
                        faces[f    ] = faces.back(); faces.pop_back();

                        normals[i] = normals.back(); normals.pop_back();

                        i--;
                    }
                }

                if (uniqueEdges.size() == 0) {
                    break;
                }

                // 现在我们有了一个 unique edge 的列表，我们可
                // 以将 newface 添加到一个列表中，并将支撑点添加
                // 到多面体中。将 newface 存储在他们自己的列表中
                // 允许我们仅计算这些 newface 的法线。
                std::vector<size_t> newFaces;
                for (int i = 0; i < uniqueEdges.size(); i++) {
                    size_t edge1 = std::get<0>(uniqueEdges[i]);
                    size_t edge2 = std::get<1>(uniqueEdges[i]);

                    newFaces.push_back(edge1);
                    newFaces.push_back(edge2);
                    // 即 support 的索引
                    newFaces.push_back(polytope.size());
                }

                polytope.push_back(support);

                auto newNormalsFaces = GetFaceNormals(polytope, newFaces);
                std::vector<QVector4D> newNormals = std::get<0>(newNormalsFaces);
                size_t newMinFace = std::get<1>(newNormalsFaces);

                float oldMinDistance = FLT_MAX;

                for (size_t i = 0; i < normals.size(); i++) {
                    if (normals[i].w() < oldMinDistance) {
                        oldMinDistance = normals[i].w();
                        minFace = i;
                    }
                }

                if (newNormals[newMinFace].w() < oldMinDistance) {
                    minFace = newMinFace + normals.size();
                }

                faces  .insert(faces  .end(), newFaces  .begin(), newFaces  .end());
                normals.insert(normals.end(), newNormals.begin(), newNormals.end());

            }
        }
        if (minDistance == FLT_MAX) {
            return {};
        }

        CollisionPoints points;

        /* 求解 Contact Point
         * 1、求解 origin 在 EPA 最终得到的三角形上的投影点在该三角形上的重心坐标
         *  即解关于 x,y,z 的方程 Cp = x Ca + y Cb + z Cc
         *
         */
        qDebug()<<"#########################";
        qDebug()<<normals[minFace];
        qDebug()<<polytope[faces[3*minFace]].C<<","<<polytope[faces[3*minFace + 1]].C<<","<<polytope[faces[3*minFace + 2]].C;
        qDebug()<<"#########################";

        QVector3D Cp = minNormal * minDistance;
        SupportPoint Ca = polytope[faces[3*minFace]];
        SupportPoint Cb = polytope[faces[3*minFace + 1]];
        SupportPoint Cc = polytope[faces[3*minFace + 2]];

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3,3);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3,1);  //3D curve

        A(0, 0) = Ca.C.x(); A(1, 0) = Ca.C.y(); A(2, 0) = Ca.C.z();
        A(0, 1) = Cb.C.x(); A(1, 1) = Cb.C.y(); A(2, 1) = Cb.C.z();
        A(0, 2) = Cc.C.x(); A(1, 2) = Cc.C.y(); A(2, 2) = Cc.C.z();

        B(0, 0) = Cp.x(); B(1, 0) = Cc.C.y(); B(2, 0) = Cp.z();

        Eigen::MatrixXd X = A.colPivHouseholderQr().solve(B);

        float alpha = X(0, 0);
        float belta = X(1, 0);
        float gamma = X(2, 0);
        qDebug()<< alpha << ", " << belta <<", " << gamma;
        qDebug()<<Cp;


        QVector3D Ap = alpha * Ca.A + belta * Cb.A + gamma * Cc.A;
        QVector3D Bp = alpha * Ca.B + belta * Cb.B + gamma * Cc.B;
        qDebug()<< Ap << ", " << Bp;

        points.Normal = minNormal;
        points.Depth = minDistance + 0.001f;
        points.HasCollision = true;
        points.ContactPoint = Ap;

        return points;
    }
}

}
