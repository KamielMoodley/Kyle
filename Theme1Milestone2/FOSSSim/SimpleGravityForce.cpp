#include "SimpleGravityForce.h"

SimpleGravityForce::SimpleGravityForce( const Vector2s& gravity )
: Force()
, m_gravity(gravity)
{
  assert( (m_gravity.array()==m_gravity.array()).all() );
  assert( (m_gravity.array()!=std::numeric_limits<scalar>::infinity()).all() );
}

SimpleGravityForce::~SimpleGravityForce()
{}

void SimpleGravityForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size()%2 == 0 );

  // Add milestone 1 code here.
  // Assume 0 potential is at origin
  for( int i = 0; i < x.size()/2; ++i ) E -= m(2*i)*m_gravity.dot(x.segment<2>(2*i));
}

void SimpleGravityForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == gradE.size() );
  assert( x.size()%2 == 0 );

  // Add milestone 1 code here.
  for( int i = 0; i < x.size()/2; ++i ) gradE.segment<2>(2*i) -= m(2*i)*m_gravity;
}

void SimpleGravityForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( x.size() == hessE.cols() );
  assert( x.size()%2 == 0 );
  // Nothing to do.
}

void SimpleGravityForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( x.size() == hessE.cols() );
  assert( x.size()%2 == 0 );
  // Nothing to do.
}

Force* SimpleGravityForce::createNewCopy()
{
  return new SimpleGravityForce(*this);
}
