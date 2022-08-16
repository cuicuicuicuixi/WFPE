#include "cloth.h"

Cloth::Cloth()
{
    for (uint32_t y(0); y < cloth_height; ++y) {
            // This is just an arbitrary formula to make the top links stronger since
            // they are under bigger stress
            const float max_elongation = 1.2f * (2.0f - y / float(cloth_height));
            for (uint32_t x(0); x < cloth_width; ++x) {
                const int id = addParticle(
                    QVector3D(start_x + x * links_length, y * links_length, 0)
                );
                // Add left link if there is a particle on the left
                if (x > 0) {
                    addLink(id-1, id, max_elongation * 0.9f);
                }
                // Add top link if there is a particle on the top
                if (y > 0) {
                    addLink(id-cloth_width, id, max_elongation);
                } else {
                    // If not, pin the particle
                    objects[id]->moving = false;
                }
            }
    }

}
