#include "PenaltyForce.h"
#include "TwoDScene.h"

PenaltyForce::PenaltyForce( const TwoDScene &scene, const scalar stiffness, const scalar thickness )
: Force()
, m_scene(scene)
, m_k(stiffness)
, m_thickness(thickness)
{
}

PenaltyForce::~PenaltyForce()
{}

void PenaltyForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
    // Feel free to implement if you feel like doing so.
}

void PenaltyForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
    for (int i = 0; i < m_scene.getNumParticles(); i++)
    {
        for (int j = i + 1; j < m_scene.getNumParticles(); j++)
        {
            addParticleParticleGradEToTotal(x, i, j, gradE);
        }

        for (int e = 0; e < m_scene.getNumEdges(); e++)
        {
            if (m_scene.getEdge(e).first != i && m_scene.getEdge(e).second != i)
                addParticleEdgeGradEToTotal(x, i, e, gradE);
        }

        for (int f = 0; f < m_scene.getNumHalfplanes(); f++)
        {
            addParticleHalfplaneGradEToTotal(x, i, f, gradE);
        }
    }
}

void PenaltyForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
    assert(!"Implicit integration of penalty forces is not supported in this milestone.");
    // Nothing to do.
}

void PenaltyForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
    assert(!"Implicit integration of penalty forces is not supported in this milestone.");
    // Nothing to do.
}

Force* PenaltyForce::createNewCopy()
{
    return new PenaltyForce(*this);
}




// BEGIN STUDENT CODE

scalar PenaltyForce::getGap(const scalar &r1, const scalar &r2, const VectorXs &n) {
    return n.norm() - r1 - r2 - m_thickness;
}
inline VectorXs getNhat(const VectorXs& n) {
    scalar nnorm = n.norm();
    assert(nnorm > 0);
    return n / nnorm;
}

// Adds the gradient of the penalty potential (-1 * force) for a pair of
// particles to the total.
// Read the positions of the particles from the input variable x. Radii can
// be obtained from the member variable m_scene, the penalty force stiffness
// from member variable m_k, and penalty force thickness from member variable
// m_thickness.
// Inputs:
//   x:    The positions of the particles in the scene.
//   idx1: The index of the first particle, i.e. the position of this particle
//         is ( x[2*idx1], x[2*idx1+1] ).
//   idx2: The index of the second particle.
// Outputs:
//   gradE: The total gradient of penalty force. *ADD* the particle-particle
//          gradient to this total gradient.
void PenaltyForce::addParticleParticleGradEToTotal(const VectorXs &x, int idx1, int idx2, VectorXs &gradE)
{
    Vector2s x1 = x.segment<2>(2*idx1);
    Vector2s x2 = x.segment<2>(2*idx2);

    double r1 = m_scene.getRadius(idx1);
    double r2 = m_scene.getRadius(idx2);

    // Add Theme 2 Milestone 1 code here.
    Vector2s n = x2 - x1;
    scalar gap = getGap(r1, r2, n);
    if (gap >= 0) return;
    Vector2s nhat = getNhat(n);

    Matrix2s Id; Id.setIdentity();
    MatrixXs gradn(2,4); gradn << -Id, Id;
    VectorXs gradV = m_k * gap * gradn.transpose() * nhat;
    gradE.segment<2>(2*idx1) += gradV.segment<2>(0);
    gradE.segment<2>(2*idx2) += gradV.segment<2>(2);
}

// Adds the gradient of the penalty potential (-1 * force) for a particle-edge
// pair to the total.
// Read the positions of the particle and edge endpoints from the input
// variable x.
// Inputs:
//   x:    The positions of the particles in the scene.
//   vidx: The index of the particle.
//   eidx: The index of the edge, i.e. the indices of the particle making up the
//         endpoints of the edge are given by m_scene.getEdge(eidx).first and
//         m_scene.getEdges(eidx).second.
// Outputs:
//   gradE: The total gradient of penalty force. *ADD* the particle-edge
//          gradient to this total gradient.
inline scalar getAlpha(const VectorXs &x1, const VectorXs &x2, const VectorXs &x3) {
    scalar edgeSqr = (x3 - x2).squaredNorm();
    if (edgeSqr == 0) return -1; // assert(edgeSqr > 0);
    scalar alpha = (x1 - x2).dot(x3 - x2) / edgeSqr;
    alpha = (alpha < 0)? 0: ((alpha > 1)? 1: alpha);
    return alpha;
}
void PenaltyForce::addParticleEdgeGradEToTotal(const VectorXs &x, int vidx, int eidx, VectorXs &gradE)
{
    int eidx1 = m_scene.getEdge(eidx).first;
    int eidx2 = m_scene.getEdge(eidx).second;
    Vector2s x1 = x.segment<2>(2*vidx);
    Vector2s x2 = x.segment<2>(2*eidx1);
    Vector2s x3 = x.segment<2>(2*eidx2);

    double r1 = m_scene.getRadius(vidx);
    double r2 = m_scene.getEdgeRadii()[eidx];

    // Add Theme 2 Milestone 1 code here.
    scalar alpha = getAlpha(x1, x2, x3);
    if (alpha < 0) return;
    Vector2s x4 = x2 + alpha * (x3 - x2);
    Vector2s n = x4 - x1;
    scalar gap = getGap(r1, r2, n);
    if (gap >= 0) return;
    Vector2s nhat = getNhat(n);

    Matrix2s Id; Id.setIdentity();
    MatrixXs gradn(2,6); gradn << -Id, (1-alpha)*Id, alpha*Id;
    VectorXs gradV = m_k * gap * gradn.transpose() * nhat;
    gradE.segment<2>(2*vidx) += gradV.segment<2>(0);
    gradE.segment<2>(2*eidx1) += gradV.segment<2>(2);
    gradE.segment<2>(2*eidx2) += gradV.segment<2>(4);
}

// Adds the gradient of the penalty potential (-1 * force) for a particle-
// half-plane pair to the total.
// Read the positions of the particle from the input variable x.
// Inputs:
//   x:    The positions of the particles in the scene.
//   vidx: The index of the particle.
//   pidx: The index of the half-plane, i.e. the position and normal vectors
//         for the half-plane can be retrieved by calling
//         m_scene.getHalfplane(pidx).
// Outputs:
//   gradE: The total gradient of the penalty force. *ADD* the particle-
//          half-plane gradient to this total gradient.
void PenaltyForce::addParticleHalfplaneGradEToTotal(const VectorXs &x, int vidx, int pidx, VectorXs &gradE)
{
    Vector2s x1 = x.segment<2>(2*vidx);
    Vector2s xh = m_scene.getHalfplane(pidx).first;
    Vector2s nh = m_scene.getHalfplane(pidx).second;

    // Add Theme 2 Milestone 1 code here.
    scalar nhsqr = nh.squaredNorm();
    assert(nhsqr > 0);
    Vector2s n = (xh - x1).dot(nh) * nh / nhsqr;
    scalar gap = getGap(m_scene.getRadius(vidx), 0, n);
    if (gap >= 0) return;
    Vector2s nhat = getNhat(n);

    Matrix2s gradn = - nh * nh.transpose() / nhsqr;
    Vector2s gradV = m_k * gap * gradn.transpose() * nhat;
    gradE.segment<2>(2*vidx) += gradV;
}
