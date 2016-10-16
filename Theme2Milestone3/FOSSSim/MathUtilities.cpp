#include "MathUtilities.h"

namespace mathutils
{

bool approxSymmetric( const MatrixXs& A, const scalar& eps )
{
  for( int i = 0; i < A.rows(); ++i ) for( int j = i+1; j < A.cols(); ++j ) if( fabs(A(i,j)-A(j,i)) >= eps ) return false;
  return true;
}

scalar crossTwoD( const Vector2s& a, const Vector2s& b )
{
  return a.x()*b.y()-a.y()*b.x();
}

Vector2s rotateCounterClockwise90degrees( const Vector2s& x )
{
  return Vector2s(-x.y(),x.x());
}

}
