#include "SpringForce.h"

SpringForce::SpringForce( const std::pair<int,int>& endpoints, const scalar& k, const scalar& l0, const scalar& b )
: Force()
, m_endpoints(endpoints)
, m_k(k)
, m_l0(l0)
, m_b(b)
{
  assert( m_endpoints.first >= 0 );
  assert( m_endpoints.second >= 0 );
  assert( m_endpoints.first != m_endpoints.second );
  assert( m_k >= 0.0 );
  assert( m_l0 >= 0.0 );
  assert( m_b >= 0.0 );
}

SpringForce::~SpringForce()
{}

void SpringForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
  assert( x.size() == v.size() );
  assert( x.size()%2 == 0 );
  assert( m_endpoints.first >= 0 );  assert( m_endpoints.first < x.size()/2 );
  assert( m_endpoints.second >= 0 ); assert( m_endpoints.second < x.size()/2 );

  scalar l = (x.segment<2>(2*m_endpoints.second)-x.segment<2>(2*m_endpoints.first)).norm();
  E += 0.5*m_k*(l-m_l0)*(l-m_l0);
}

void SpringForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
  assert( x.size() == v.size() );
  assert( x.size() == gradE.size() );
  assert( x.size()%2 == 0 );
  assert( m_endpoints.first >= 0 );  assert( m_endpoints.first < x.size()/2 );
  assert( m_endpoints.second >= 0 ); assert( m_endpoints.second < x.size()/2 );

  // Compute the elastic component
  Vector2s nhat = x.segment<2>(2*m_endpoints.second)-x.segment<2>(2*m_endpoints.first);
  scalar l = nhat.norm();
  assert( l != 0.0 );
  nhat /= l;
  Vector2s fdamp = nhat;
  nhat *= m_k*(l-m_l0);
  gradE.segment<2>(2*m_endpoints.first)  -= nhat;
  gradE.segment<2>(2*m_endpoints.second) += nhat;

  // Compute the internal damping
  // Remember we are computing minus the force here
  fdamp *= m_b*fdamp.dot(v.segment<2>(2*m_endpoints.second)-v.segment<2>(2*m_endpoints.first));
  gradE.segment<2>(2*m_endpoints.first)  -= fdamp;
  gradE.segment<2>(2*m_endpoints.second) += fdamp;
}

void SpringForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( x.size() == hessE.cols() );
  assert( x.size()%2 == 0 );
  assert( m_endpoints.first >= 0 );  assert( m_endpoints.first < x.size()/2 );
  assert( m_endpoints.second >= 0 ); assert( m_endpoints.second < x.size()/2 );

  // Add milestone 3 code here.

  // Implement force Jacobian here!
  Vector2s n;
  int xiInd = 2*m_endpoints.first, xjInd = 2*m_endpoints.second;
  scalar l = getLSetN(x.segment<2>(xiInd), x.segment<2>(xjInd), n);
  assert( l != 0.0 );

  Matrix2s Id; Id.setIdentity();
  Matrix2s nn = n * n.transpose();
  Vector2s vdiff = v.segment<2>(xiInd) - v.segment<2>(xjInd);
  Matrix2s K = -m_k * (nn + (l - m_l0)/l * (Id - nn)) // Contribution from elastic component
      -m_b/l * (n.dot(vdiff) * Id + n * vdiff.transpose()) * (Id - nn); // Contribution from damping
  hessE.block<2,2>(xiInd, xiInd) -= K;
  hessE.block<2,2>(xiInd, xjInd) += K;
  hessE.block<2,2>(xjInd, xiInd) += K;
  hessE.block<2,2>(xjInd, xjInd) -= K;
}

void SpringForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( x.size() == hessE.cols() );
  assert( x.size()%2 == 0 );
  assert( m_endpoints.first >= 0 );  assert( m_endpoints.first < x.size()/2 );
  assert( m_endpoints.second >= 0 ); assert( m_endpoints.second < x.size()/2 );

  // Add milestone 3 code here.

  // Implement force Jacobian here!
  // Contribution from damping
  Vector2s n;
  int xiInd = 2*m_endpoints.first, xjInd = 2*m_endpoints.second;
  getLSetN(x.segment<2>(xiInd), x.segment<2>(xjInd), n);
  Matrix2s B = m_b * n * n.transpose();
  hessE.block<2,2>(xiInd, xiInd) += B;
  hessE.block<2,2>(xiInd, xjInd) -= B;
  hessE.block<2,2>(xjInd, xiInd) -= B;
  hessE.block<2,2>(xjInd, xjInd) += B;
}

Force* SpringForce::createNewCopy()
{
  return new SpringForce(*this);
}
