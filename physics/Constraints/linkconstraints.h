#ifndef LINKCONSTRAINTS_H
#define LINKCONSTRAINTS_H

#include <vector>
#include <QVector3D>

struct Particle
{
    uint          id            = 0;
    float        mass          = 1.0f;
    bool         moving        = true;
    QVector3D position;
    QVector3D position_old;
    QVector3D velocity;
    QVector3D forces;

    Particle() = default;

    explicit
    Particle(QVector3D pos)
        : position(pos)
        , position_old(pos)
    {}

    void update(float dt)
    {
        if (!moving) return;
        position_old = position;
        velocity += (forces / mass) * dt;
        position += velocity * dt;
    }

    void updateDerivatives(float dt)
    {
        velocity = (position - position_old) / dt;
        forces = {};
    }

    void move(QVector3D v)
    {
        if (!moving) return;
        position += v;
    }
};


class LinkConstraint
{
public:
    uint id                          = 0;
    Particle* particle_1;
    Particle* particle_2;
    float       distance             = 1.0f;
    float       strength             = 1.0f;
    float       max_elongation_ratio = 1.5f;
    bool        broken               = false;

    LinkConstraint() = default;

    LinkConstraint(Particle* p_1, Particle* p_2)
        : particle_1(p_1)
        , particle_2(p_2)
    {
        distance = (p_1->position - p_2->position).length();
    }

    [[nodiscard]]
    bool isValid() const
    {
        return particle_2 && particle_1 && !broken;
    }

    void solve()
    {
        if (!isValid()) { return; }
        Particle& p_1 = *particle_1;
        Particle& p_2 = *particle_2;
        const QVector3D v = p_1.position - p_2.position;
        const float dist = v.length();
        if (dist > distance) {
            // 距离超出阈值时连接断开
            broken = dist > distance * max_elongation_ratio;
            const QVector3D n = v / dist;
            const float c = distance - dist;
            const QVector3D p = -(c * strength) / (p_1.mass + p_2.mass) * n;
            // Apply position correction
            p_1.move(-p / p_1.mass);
            p_2.move( p / p_2.mass);
        }
    }
};

#endif // LINKCONSTRAINTS_H
