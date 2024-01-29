#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
		for (int i = 0; i < num_nodes; i++)
		{
			Vector2D position = start + (end-start)*((double)i / (double)(num_nodes-1));
			masses.push_back(new Mass(position, node_mass, false));
		}
		for(int i = 0; i < num_nodes-1; i++)
		{
			springs.push_back(new Spring(masses[i], masses[i+1], k));
		}

//        Comment-in this part when you implement the constructor
          for (auto &i : pinned_nodes) {
              masses[i]->pinned = true;
          }
		
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
			Mass* a = s->m1;
			Mass* b = s->m2;
			Vector2D r_pos = b->position - a->position;
			double mod = r_pos.norm();
			Vector2D Fba = - s->k * r_pos / mod * (mod - s->rest_length);
			a->forces = -Fba;
			b->forces = Fba;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
				Vector2D accel = m->forces / m->mass + gravity;
				m->position += delta_t * m->velocity;
				m->velocity += delta_t * accel;
                // TODO (Part 2): Add global damping
				
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
			Mass* a = s->m1;
			Mass* b = s->m2;
			Vector2D r_pos = b->position - a->position;
			double mod = r_pos.norm();
			Vector2D Fba = - s->k * r_pos / mod * (mod - s->rest_length);
			a->forces = -Fba;
			b->forces = Fba;
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
				Vector2D a = m->forces / m->mass + gravity;
                m->position += (m->position - m->last_position) * a * delta_t * delta_t;
				m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
