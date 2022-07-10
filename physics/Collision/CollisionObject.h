#pragma once

#include "Collider.h"

namespace physE {
    struct Collision;
    struct Object {
        int idx;        
        QVector3D Velocity;
        QVector3D Force;
        float Mass;
        float I; //转动惯量

        // Angular components
        float orientation = 0; // radians
        float angularVelocity = 0;
        float torque = 0;

        Collider* Collider;
        Transform* Transform;

        bool IsTrigger;
        bool IsStatic;
        const bool IsDynamic;

        Object(bool _IsDynamic = false)
            : IsTrigger(false)
            , IsStatic(!_IsDynamic)
            , IsDynamic(_IsDynamic)
        {

        }

        Object(int _idx, QVector3D pos, bool _IsDynamic = false): idx(_idx), IsDynamic(_IsDynamic)
        {
            Velocity = QVector3D(0, 0, 0);
            Force = QVector3D(0, 0, 0);
            Mass = 1;
            Transform = new struct Transform();
            Transform->Position = pos;
            Transform->Rotation = QMatrix3x3();
            Transform->Scale = QVector3D(1, 1, 1);
        }

        Object(int _idx, QVector3D pos, QVector3D vel, bool _IsDynamic = false): idx(_idx), Velocity(vel), IsDynamic(_IsDynamic)
        {
            Force = QVector3D(0, 0, 0);
            Mass = 1;
            Transform = new struct Transform();
            Transform->Position = pos;
            Transform->Rotation = QMatrix3x3();
            Transform->Scale = QVector3D(1, 1, 1);
        }

        Object(int _idx, QVector3D pos, QVector3D vel, struct Collider* _Collider, bool _IsDynamic = false)
            : idx(_idx)
            , Velocity(vel)
            , Collider(_Collider)
            , IsDynamic(_IsDynamic)
        {
            Force = QVector3D(0, 0, 0);
            Mass = 1;
            Transform = new struct Transform();
            Transform->Position = pos;
            Transform->Rotation = QMatrix3x3();
            Transform->Scale = QVector3D(1, 1, 1);
        }

        Object(int _idx, QVector3D pos, QVector3D vel, struct Collider* _Collider, struct Transform* _tran, bool _IsDynamic = false)
            : idx(_idx)
            , Velocity(vel)
            , Collider(_Collider)
            , IsDynamic(_IsDynamic)
        {
            Force = QVector3D(0, 0, 0);
            Mass = 1;
            Transform = _tran;
            Transform = new struct Transform();
            Transform->Position = pos;
            Transform->Rotation = QMatrix3x3();
            Transform->Scale = QVector3D(1, 1, 1);
        }

        Object(int _idx, struct Collider* _Collider, bool _IsDynamic = false)
            : idx(_idx)
            , Collider(_Collider)
            , IsDynamic(_IsDynamic)
        {
            Transform = new struct Transform();
            Transform->Position = QVector3D(0, 0, 0);
            Transform->Rotation = QMatrix3x3();
            Transform->Scale = QVector3D(1, 1, 1);
        }

        QVector3D operator + (Object &b)
        {
            return Transform->Position + b.Transform->Position;
        }

        QVector3D operator - (Object &b)
        {
            return Transform->Position - b.Transform->Position;
        }

        double operator [] (int &b)
        {
            return Transform->Position[b];
        }

        void Draw(QOpenGLFunctions_3_3_Core* glFunc, QOpenGLShaderProgram* shaderProgram)
        {
            Collider->Draw(glFunc, shaderProgram, Transform);
        }
    };
}
