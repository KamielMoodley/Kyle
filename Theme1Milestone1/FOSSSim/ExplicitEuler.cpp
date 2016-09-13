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
    // Your code goes here!

    VectorXs& x = scene.getX(); // the system's position DoFs
    VectorXs& v = scene.getV(); // the system's velocity DoFs
    const VectorXs& m = scene.getM(); // the masses associated to each DoF

    // F = - \grad(U)
    VectorXs gradU; gradU.resize(x.size()); gradU.fill(0);
    scene.accumulateGradU(gradU); // dx = 0, dv = 0
    VectorXs a = - gradU.cwiseQuotient(m);

    // x_new = x + v * dt
    x += v * dt;
    // v_new = v + a * dt
    int num_particles = scene.getNumParticles();
    for (int i = 0; i < num_particles; ++i) {
        // Determine if the ith particle is fixed
        if (scene.isFixed(i)) continue;
        v.segment<2>(2*i) += a.segment<2>(2*i) * dt;
    }

    return true;
}

std::string ExplicitEuler::getName() const
{
    return "Explicit Euler";
}
