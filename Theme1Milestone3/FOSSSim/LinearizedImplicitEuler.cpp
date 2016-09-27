#include "LinearizedImplicitEuler.h"

LinearizedImplicitEuler::LinearizedImplicitEuler()
: SceneStepper()
{}

LinearizedImplicitEuler::~LinearizedImplicitEuler()
{}

bool LinearizedImplicitEuler::stepScene( TwoDScene& scene, scalar dt )
{
  VectorXs& x = scene.getX();
  VectorXs& v = scene.getV();
  const VectorXs& m = scene.getM();
  assert(x.size() == v.size());
  assert(x.size() == m.size());

  // Add milestone 3 code here.

  // Implement implicit euler here!

  // Get the force Jacobian from two d scene
  int ndof = x.size();
  assert( ndof%2 == 0 );
  // Note that the system's state is passed to two d scene as a change from the last timestep's solution
  VectorXs dx = dt*v;
  VectorXs dv = VectorXs::Zero(ndof);
  MatrixXs fdx = MatrixXs::Zero(ndof, ndof); // f = - dF
  MatrixXs fdv = MatrixXs::Zero(ndof, ndof);
  scene.accumulateddUdxdx(fdx, dx, dv);
  scene.accumulateddUdxdv(fdv, dx, dv);

  VectorXs gradU = VectorXs::Zero(ndof);
  scene.accumulateGradU(gradU, dx, dv);

  MatrixXs M = m.asDiagonal();
  MatrixXs A = M + dt*dt * fdx + dt * fdv;

  // Fixed Vertices
  int num = scene.getNumParticles(), i;
  for( i = 0; i < num; ++i )
    if( scene.isFixed(i) ) {
      // Zero the force for fixed DoFs
      gradU.segment<2>(2*i).setZero();
      // Set the row and column of A to 0 for fixed DoFs
      A.row(2*i).setZero(); A.row(2*i+1).setZero();
      A.col(2*i).setZero(); A.col(2*i+1).setZero();
      // Set the diagonal entry of A to 1 for fixed DoFs
      A(2*i, 2*i) = 1; A(2*i+1, 2*i+1) = 1;
    }

  VectorXs delta = A.fullPivLu().solve(-dt * gradU); // Compute the solution to A*x = b
  v += delta;
  x += dt * v;

  return true;
}

std::string LinearizedImplicitEuler::getName() const
{
  return "Linearized Implicit Euler";
}
