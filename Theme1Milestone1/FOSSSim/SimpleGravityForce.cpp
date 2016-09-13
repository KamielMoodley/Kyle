#include "SimpleGravityForce.h"

// Constant Gravity

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

    // Your code goes here!

    // U = sum( - (mi .* g).dot(xi) )
    // potential energy U is irrelevant to v
    int num_particles = x.size() / 2;
    scalar U = 0;
    for (int i = 0; i < num_particles; ++i) {
        U += - (m.segment<2>(2*i)).cwiseProduct(m_gravity).dot(x.segment<2>(2*i));
    }
    E += U;
}

/**
 * GradE = gradient of energy = ∂E / ∂x
 * HessE = hessian of energy = (∂^2 E) / (∂x∂x)
 * HessE = hessian of energy over position then over velocity = (∂^2 E) / (∂x˙∂x)
 */
void SimpleGravityForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
    assert( x.size() == v.size() );
    assert( x.size() == m.size() );
    assert( x.size() == gradE.size() );
    assert( x.size()%2 == 0 );

    // Your code goes here!
    
    // \grad(U) = - F = - mi .* g
    int num_particles = x.size() / 2;
    VectorXs gradU; gradU.resize(x.size()); gradU.fill(0);
    for (int i = 0; i < num_particles; ++i) {
        gradU.segment<2>(2*i) = - (m.segment<2>(2*i)).cwiseProduct(m_gravity);
    }
    gradE += gradU;
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
