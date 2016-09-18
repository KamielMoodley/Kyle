#ifndef __FORCE_H__
#define __FORCE_H__

#include <Eigen/Core>

#include "MathDefs.h"

class Force
{
public:

  virtual ~Force();

  virtual void addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E ) = 0;

  virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE ) = 0;

  virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE ) = 0;

  virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE ) = 0;

  virtual Force* createNewCopy() = 0;
};

inline scalar getLSetN(const Vector2s & x_1, const Vector2s & x_2, Vector2s & n){
    scalar l = (x_1 - x_2).norm();
    if (l == 0.0) n.setZero();
    else n = (x_1 - x_2) / l;
    return l;
}
inline scalar getL(const Vector2s & x_1, const Vector2s & x_2){
    return (x_1 - x_2).norm(); // 2-norm, the Euclidean distance.
}

#endif
