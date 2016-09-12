#include "ExplicitEuler.h"

ExplicitEuler::ExplicitEuler()
: SceneStepper()
{}

ExplicitEuler::~ExplicitEuler()
{}

/**
 * Compute the updated position and velocity using explicit Euler.
 */
bool ExplicitEuler::stepScene( TwoDScene& scene, scalar dt )
{

    // q ̇^(n+1)=q ̇^n+hM^(−1) F(q^n,q ̇^n )
    int num_particles = scene.getNumParticles();
    VectorXs& x = scene.getX(); // the system's position DoFs
    VectorXs& v = scene.getV(); // the system's velocity DoFs
    // const VectorXs& m = scene.getM(); // the masses associated to each DoF
    for (int i = 0; i < num_particles; ++i) {
        // Determine if the ith particle is fixed
        if (scene.isFixed(i)) continue;
        // x_new = x + v * dt
        x.segment<2>(2*i) += v.segment<2>(2*i) * dt; // const Vector2s& pos
        // v_new = v + a * dt
        // todo: suppose a == 0
    }
    return true;
}

std::string ExplicitEuler::getName() const
{
    return "Explicit Euler";
}
