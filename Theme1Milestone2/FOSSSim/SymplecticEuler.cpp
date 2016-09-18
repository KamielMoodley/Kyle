#include "SymplecticEuler.h"

SymplecticEuler::SymplecticEuler()
: SceneStepper()
{}

SymplecticEuler::~SymplecticEuler()
{}

bool SymplecticEuler::stepScene( TwoDScene& scene, scalar dt )
{
  // Add milestone 2 code here.

  VectorXs& x = scene.getX(); // the system's position DoFs
  VectorXs& v = scene.getV(); // the system's velocity DoFs
  const VectorXs& m = scene.getM(); // the masses associated to each DoF

  // get a
  VectorXs gradU; gradU.resize(x.size()); gradU.setZero();
  scene.accumulateGradU(gradU);
  VectorXs a = - gradU.cwiseQuotient(m); // F = - \grad(U)
  int numParticles = scene.getNumParticles();
  for( int i = 0; i < numParticles; ++i )
    if( scene.isFixed(i) ) a.segment<2>(2*i).setZero();

  v += dt * a;
  x += dt * v; 
  return true;
}

std::string SymplecticEuler::getName() const
{
  return "Symplectic Euler";
}
