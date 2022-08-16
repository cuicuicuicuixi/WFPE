#ifndef CLOTH_H
#define CLOTH_H
#include <algorithm>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLExtraFunctions>
#include <QOpenGLTexture>
#include <QOpenGLWidget>
#include <QtOpenGLExtensions/QOpenGLExtensions>
#include "../Constraints/linkconstraints.h"

class Cloth
{
public:
    // Initialize the cloth
    int cloth_width  = 60;
    int cloth_height = 30;
    float    links_length = 20.0f;
    float    start_x      = ((cloth_width - 1) * links_length) * 0.5f;
    std::vector<Particle*>       objects;
    std::vector<LinkConstraint> constraints;

    Cloth();

    void update(float dt)
    {
        removeBrokenLinks();
        applyGravity();
        applyAirFriction();
        updatePositions(dt);
        solveConstraints();
        updateDerivatives(dt);
    }

    void applyGravity()
    {
        const QVector3D gravity(0.0f, 1500.0f, 0.0f);
        for (Particle* p : objects) {
            p->forces += gravity * p->mass;
        }
    }

    void applyAirFriction()
    {
        const float friction_coef = 0.5f;
        for (Particle* p : objects) {
            p->forces -= p->velocity * friction_coef;
        }
    }

    void updatePositions(float dt)
    {
        for (Particle* p : objects) {
            p->update(dt);
        }
    }

    void updateDerivatives(float dt)
    {
        for (Particle* p : objects) {
            p->updateDerivatives(dt);
        }
    }

    void solveConstraints()
    {
        for (uint32_t i(32); i--;) {
            for (LinkConstraint &l: constraints) {
                l.solve();
            }
        }
    }

    void removeBrokenLinks()
    {
        //constraints.remove_if([](const LinkConstraint& c) {return !c.isValid();});
        constraints.erase(std::remove_if(constraints.begin(), constraints.end(), [](const LinkConstraint& c) {
            return !c.isValid();
        }), constraints.end());
    }

    int addParticle(QVector3D position)
    {
        const int particle_id = objects.size();
        objects.emplace_back(new Particle(position));
        objects[particle_id]->id = particle_id;
        return particle_id;
    }

    void addLink(int particle_1, int particle_2, float max_elongation_ratio = 1.5f)
    {
        const int link_id = constraints.size();
        constraints.emplace_back(LinkConstraint(objects[particle_1], objects[particle_2]));
        constraints[link_id].id = link_id;
        constraints[link_id].max_elongation_ratio = max_elongation_ratio;
    }

    void Draw(QOpenGLFunctions_3_3_Core* glFunc, QOpenGLShaderProgram* shaderProgram) {

    }
};

#endif // CLOTH_H
