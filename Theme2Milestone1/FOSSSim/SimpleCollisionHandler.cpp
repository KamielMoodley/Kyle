#include "SimpleCollisionHandler.h"
#include <iostream>
#include <set>

// Loops over all edges, particles, and half-planes in the simulation, and
// applies collision detection and handling to each pair.
//
// Note: No collision detection is done between edges that are connected
// by a particle (since these are always colliding.)
//
// Does not need to be changed by students.

void SimpleCollisionHandler::handleCollisions(TwoDScene &scene, const VectorXs &oldpos, VectorXs &oldvel, scalar dt)
{
    Vector2s n;

    for (int i = 0; i < scene.getNumParticles(); i++)
    {
        for (int j = i + 1; j < scene.getNumParticles(); j++)
        {
            if (detectParticleParticle(scene, i, j, n))
            {
                addParticleParticleImpulse(i, j, n, 0);
                respondParticleParticle(scene, i, j, n);
            }
        }

        for (int e = 0; e < scene.getNumEdges(); e++)
        {
            if (scene.getEdges()[e].first != i && scene.getEdges()[e].second != i)
                if (detectParticleEdge(scene, i, e, n))
                {
                    addParticleEdgeImpulse(i, e, n, 0);
                    respondParticleEdge(scene, i, e, n);
                }
        }

        for (int f = 0; f < scene.getNumHalfplanes(); f++)
        {
            if (detectParticleHalfplane(scene, i, f, n))
            {
                addParticleHalfplaneImpulse(i, f, n, 0);
                respondParticleHalfplane(scene, i, f, n);
            }
        }
    }
}

std::string SimpleCollisionHandler::getName() const
{
    return "Simple Collision Handling";
}




// BEGIN STUDENT CODE //


// Detects whether two particles are overlapping (including the radii of each)
// and approaching.
// If the two particles overlap and are approaching, returns true and sets
// the vector n to be the vector between the first and second particle.
// Inputs:
//   scene: The scene data structure. The positions and radii of the particles
//          can be obtained from here.
//   idx1:  The index of the first particle. (Ie, the degrees of freedom
//          corresponding to this particle are entries 2*idx1 and 2*idx1+1 in
//          scene.getX().
//   idx2:  The index of the second particle.
// Outputs:
//   n: The vector between the two particles.
//   Returns true if the two particles overlap and are approaching.

inline bool beApproaching(const VectorXs &v1, const VectorXs &v2, const VectorXs &n) {
    return n.dot(v1 - v2) > 0;
}
inline bool beOverlapping(const scalar &r1, const scalar &r2, const VectorXs &n) {
    return n.norm() < r1 + r2;
}

bool SimpleCollisionHandler::detectParticleParticle(TwoDScene &scene, int idx1, int idx2, Vector2s &n)
{
    Vector2s x1 = scene.getX().segment<2>(2*idx1);
    Vector2s x2 = scene.getX().segment<2>(2*idx2);

    // Add Theme 2 Milestone 1 code here.
    Vector2s v1 = scene.getV().segment<2>(2*idx1);
    Vector2s v2 = scene.getV().segment<2>(2*idx2);
    scalar r1 = scene.getRadius(idx1);
    scalar r2 = scene.getRadius(idx2);
    n = x2 - x1;
    return beApproaching(v1, v2, n) && beOverlapping(r1, r2, n);
}

// Detects whether a particle and an edge are overlapping (including the radii
// of both) and are approaching.
// If the two objects overlap and are approaching, returns true and sets the
// vector n to be the shortest vector between the particle and the edge.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   eidx:  The index of the edge. (Ie, the indices of particle with index e are
//          scene.getEdges()[e].first and scene.getEdges()[e].second.)
// Outputs:
//   n: The shortest vector between the particle and the edge.
//   Returns true if the two objects overlap and are approaching.
inline scalar getAlpha(const VectorXs &x1, const VectorXs &x2, const VectorXs &x3) {
    scalar edgeSqr = (x3 - x2).squaredNorm();
    if (edgeSqr == 0) return -1; // assert(edgeSqr > 0);
    scalar alpha = (x1 - x2).dot(x3 - x2) / edgeSqr;
    alpha = (alpha < 0)? 0: ((alpha > 1)? 1: alpha);
    return alpha;
}
bool SimpleCollisionHandler::detectParticleEdge(TwoDScene &scene, int vidx, int eidx, Vector2s &n)
{
    Vector2s x1 = scene.getX().segment<2>(2*vidx);
    Vector2s x2 = scene.getX().segment<2>(2*scene.getEdges()[eidx].first);
    Vector2s x3 = scene.getX().segment<2>(2*scene.getEdges()[eidx].second);

    // Add Theme 2 Milestone 1 code here.
    scalar alpha = getAlpha(x1, x2, x3);
    if (alpha < 0) return false;
    Vector2s v1 = scene.getV().segment<2>(2*vidx);
    Vector2s v2 = scene.getV().segment<2>(2*scene.getEdges()[eidx].first);
    Vector2s v3 = scene.getV().segment<2>(2*scene.getEdges()[eidx].second);
    scalar r1 = scene.getRadius(vidx);
    scalar r4 = scene.getEdgeRadii()[eidx];

    Vector2s x4 = x2 + alpha * (x3 - x2);
    Vector2s v4 = v2 + alpha * (v3 - v2);
    n = x4 - x1;

    return beApproaching(v1, v4, n) && beOverlapping(r1, r4, n);
}

// Detects whether a particle and a half-plane are overlapping (including the
// radius of the particle) and are approaching.
// If the two objects overlap and are approaching, returns true and sets the
// vector n to be the shortest vector between the particle and the half-plane.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   pidx:  The index of the halfplane. The vectors (px, py) and (nx, ny) can
//          be retrieved by calling scene.getHalfplane(pidx).
// Outputs:
//   n: The shortest vector between the particle and the half-plane.
//   Returns true if the two objects overlap and are approaching.
bool SimpleCollisionHandler::detectParticleHalfplane(TwoDScene &scene, int vidx, int pidx, Vector2s &n)
{
    Vector2s x1 = scene.getX().segment<2>(2*vidx);
    Vector2s px = scene.getHalfplane(pidx).first;
    Vector2s pn = scene.getHalfplane(pidx).second;

    // Add Theme 2 Milestone 1 code here.
    scalar pnsqr = pn.squaredNorm();
    assert(pnsqr > 0);
    n = (px - x1).dot(pn) * pn / pnsqr;
    return (n.dot(scene.getV().segment<2>(2*vidx)) > 0)
        && beOverlapping(scene.getRadius(vidx), 0, n);
}

inline VectorXs getNhat(const VectorXs& n) {
    scalar nnorm = n.norm();
    assert(nnorm > 0);
    return n / nnorm;
}

// Responds to a collision detected between two particles by applying an impulse
// to the velocities of each one.
// You can get the COR of the simulation by calling getCOR().
// Inputs:
//   scene: The scene data structure.
//   idx1:  The index of the first particle.
//   idx2:  The index of the second particle.
//   n:     The vector between the first and second particle.
// Outputs:
//   None.
void SimpleCollisionHandler::respondParticleParticle(TwoDScene &scene, int idx1, int idx2, const Vector2s &n)
{
    const VectorXs &M = scene.getM();
    VectorXs &v = scene.getV();

    // Add Theme 2 Milestone 1 code here.

    // case: m1, m2 both inf?? or 0??
    if (scene.isFixed(idx1) && scene.isFixed(idx2)) return;
    scalar m1 = (scene.isFixed(idx1))? std::numeric_limits<double>::infinity() : M(2*idx1);
    scalar m2 = (scene.isFixed(idx2))? std::numeric_limits<double>::infinity() : M(2*idx2);
    VectorXs nhat = getNhat(n);
    double newtonianCOR = (1.0 + getCOR()) / 2.0;
    scalar I = 2 * (v.segment<2>(2*idx2) - v.segment<2>(2*idx1)).dot(nhat) / (1/m1 + 1/m2) * newtonianCOR;
    v.segment<2>(2*idx1) += I/m1 * nhat;
    v.segment<2>(2*idx2) -= I/m2 * nhat;
}

// Responds to a collision detected between a particle and an edge by applying
// an impulse to the velocities of each one.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   eidx:  The index of the edge.
//   n:     The shortest vector between the particle and the edge.
// Outputs:
//   None.
void SimpleCollisionHandler::respondParticleEdge(TwoDScene &scene, int vidx, int eidx, const Vector2s &n)
{
    const VectorXs &M = scene.getM();

    int eidx1 = scene.getEdges()[eidx].first;
    int eidx2 = scene.getEdges()[eidx].second;

    VectorXs x1 = scene.getX().segment<2>(2*vidx); // copy
    VectorXs x2 = scene.getX().segment<2>(2*eidx1);
    VectorXs x3 = scene.getX().segment<2>(2*eidx2);

    VectorXs &v = scene.getV();
    VectorXs v1 = scene.getV().segment<2>(2*vidx);
    VectorXs v2 = scene.getV().segment<2>(2*eidx1);
    VectorXs v3 = scene.getV().segment<2>(2*eidx2);

    // Add Theme 2 Milestone 1 code here.
    scalar m1 = (scene.isFixed(vidx))? std::numeric_limits<double>::infinity() : M(2*vidx);
    scalar m2 = (scene.isFixed(eidx1))? std::numeric_limits<double>::infinity() : M(2*eidx1);
    scalar m3 = (scene.isFixed(eidx2))? std::numeric_limits<double>::infinity() : M(2*eidx2);
    scalar alpha = getAlpha(x1, x2, x3);
    if (alpha < 0) return; 
    scalar Idenominator = (1/m1 + pow(1-alpha,2)/m2 + pow(alpha,2)/m3);
    if (Idenominator == 0) return;
    VectorXs nhat = getNhat(n);
    VectorXs v4 = v2 + alpha * (v3 - v2);
    double newtonianCOR = (1.0 + getCOR()) / 2.0;
    scalar I = 2 * (v4 - v1).dot(nhat) / Idenominator * newtonianCOR;
    v.segment<2>(2*vidx) += I/m1 * nhat;
    v.segment<2>(2*eidx1) -= I*(1-alpha)/m2 * nhat;
    v.segment<2>(2*eidx2) -= I*alpha/m3 * nhat;

}


// Responds to a collision detected between a particle and a half-plane by
// applying an impulse to the velocity of the particle.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   pidx:  The index of the half-plane.
//   n:     The shortest vector between the particle and the half-plane.
// Outputs:
//   None.
void SimpleCollisionHandler::respondParticleHalfplane(TwoDScene &scene, int vidx, int pidx, const Vector2s &n)
{
    VectorXs nhat = getNhat(n);

    // Add Theme 2 Milestone 1 code here.
    // fixed??
    double newtonianCOR = (1.0 + getCOR()) / 2.0;
    scene.getV().segment<2>(2*vidx) -= 2 * scene.getV().segment<2>(2*vidx).dot(nhat) * newtonianCOR * nhat;

}
