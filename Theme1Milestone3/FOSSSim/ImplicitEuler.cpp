#include "ImplicitEuler.h"

ImplicitEuler::ImplicitEuler()
: SceneStepper()
{}

ImplicitEuler::~ImplicitEuler()
{}

#define epsilon 1e-9

bool ImplicitEuler::stepScene( TwoDScene& scene, scalar dt )
{
  VectorXs& x = scene.getX();
  VectorXs& v = scene.getV();
  const VectorXs& m = scene.getM();
  assert(x.size() == v.size());
  assert(x.size() == m.size());

  // Add milestone 3 code here.

  // Implement implicit euler here for extra credit!
  int ndof = x.size();
  assert( ndof%2 == 0 );
  int num = scene.getNumParticles(), i;

  MatrixXs M = m.asDiagonal();
  VectorXs B(ndof), gradU(ndof), delta(ndof);
  VectorXs v1 = v, v0(ndof), dx(ndof), dv(ndof);
  MatrixXs fdx(ndof, ndof), fdv(ndof, ndof), A(ndof, ndof); // f = - dF
  do {
    v0 = v1;
    dx = dt * v0;
    dv = v0 - v;
    fdx.setZero(); fdv.setZero(); gradU.setZero();
    scene.accumulateddUdxdx(fdx, dx, dv);
    scene.accumulateddUdxdv(fdv, dx, dv);
    scene.accumulateGradU(gradU, dx, dv);
    A = M + dt*dt * fdx + dt * fdv;
    B = -M * dv - dt * gradU;
    // Fixed Vertices 
    for( i = 0; i < num; ++i )
      if( scene.isFixed(i) ) {
        B.segment<2>(2*i).setZero(); // Zero B for fixed DoFs
        // Set the row and column of A to 0 for fixed DoFs
        A.row(2*i).setZero(); A.row(2*i+1).setZero();
        A.col(2*i).setZero(); A.col(2*i+1).setZero();
        // Set the diagonal entry of A to 1 for fixed DoFs
        A(2*i, 2*i) = 1; A(2*i+1, 2*i+1) = 1;
      }
    delta = A.fullPivLu().solve(B); // Compute the solution to A*x = B
    v1 = v0 + delta;
} while (delta.norm() >= epsilon);

  v = v1;
  x += dt * v;

  return true;
}

std::string ImplicitEuler::getName() const
{
  return "Implicit Euler";
}
