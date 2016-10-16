#include "ContestDetector.h"
#include <iostream>
#include "TwoDScene.h"
#include <set>

// For all pairs of potentially-overlapping objects, applies the penalty force.
//
// Does not need to be modified by students.
void ContestDetector::performCollisionDetection(const TwoDScene &scene, const VectorXs &qs, const VectorXs &qe, DetectionCallback &dc)
{
  if( (qs-qe).norm() > 1e-8)
    {
      std::cerr << "Contest collision detector is only designed for use with the penalty method!" << std::endl;
      exit(1);
    }

  PPList pppairs;
  PEList pepairs;
  PHList phpairs;

  findCollidingPairs(scene, qe, pppairs, pepairs, phpairs);
  for(PPList::iterator it = pppairs.begin(); it != pppairs.end(); ++it)
    dc.ParticleParticleCallback(it->first, it->second);
  for(PEList::iterator it = pepairs.begin(); it != pepairs.end(); ++it)
    {
      //particle never collides with an edge it's an endpoint of
      if(scene.getEdge(it->second).first != it->first && scene.getEdge(it->second).second != it->first)
	dc.ParticleEdgeCallback(it->first, it->second);
    }
  for(PHList::iterator it = phpairs.begin(); it != phpairs.end(); ++it)
    dc.ParticleHalfplaneCallback(it->first, it->second);
}

// Given particle positions, computes lists of *potentially* overlapping object
// pairs. How exactly to do this is up to you.
// Inputs: 
//   scene:  The scene object. Get edge information, radii, etc. from here. If 
//           for some reason you'd also like to use particle velocities in your
//           algorithm, you can get them from here too.
//   x:      The positions of the particle.
// Outputs:
//   pppairs: A list of (particle index, particle index) pairs of potentially
//            overlapping particles. IMPORTANT: Each pair should only appear
//            in the list at most once. (1, 2) and (2, 1) count as the same 
//            pair.
//   pepairs: A list of (particle index, edge index) pairs of potential
//            particle-edge overlaps.
//   phpairs: A list of (particle index, halfplane index) pairs of potential
//            particle-halfplane overlaps.
void ContestDetector::findCollidingPairs(const TwoDScene &scene, const VectorXs &x, PPList &pppairs, PEList &pepairs, PHList &phpairs)
{
}
