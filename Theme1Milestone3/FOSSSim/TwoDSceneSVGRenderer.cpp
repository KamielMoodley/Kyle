#include "TwoDSceneSVGRenderer.h"

TwoDSceneSVGRenderer::TwoDSceneSVGRenderer( const TwoDScene& scene, const std::vector<renderingutils::Color>& particle_colors, const std::vector<renderingutils::Color>& edge_colors, const std::vector<renderingutils::ParticlePath>& particle_paths, int imagewidth, int imageheight, const renderingutils::Color& bgcolor )
: m_scene(scene)
, m_w(imagewidth)
, m_h(imageheight)
, m_bgcolor(bgcolor)
, m_particle_colors(particle_colors)
, m_edge_colors(edge_colors)
, m_particle_paths(particle_paths)
, m_circle_points()
, m_semi_circle_points()
{
  assert( (int) m_particle_colors.size() == m_scene.getNumParticles() );
  assert( (int) m_edge_colors.size() == m_scene.getNumEdges() );
}

void TwoDSceneSVGRenderer::updateState()
{
  const VectorXs& x = m_scene.getX();
  for( std::vector<renderingutils::ParticlePath>::size_type i = 0; i < m_particle_paths.size(); ++i )
    m_particle_paths[i].addToPath(x.segment<2>(2*m_particle_paths[i].getParticleIdx()));
}

std::string TwoDSceneSVGRenderer::intToHexString( int integer ) const
{
  std::stringstream newstring;
  newstring << std::hex << std::setfill('0') << std::setw(2) << integer;
  return newstring.str();
}

void TwoDSceneSVGRenderer::computeBoundingBox( const VectorXs& x, const std::vector<scalar>& radii, scalar& xmin, scalar& xmax, scalar& ymin, scalar& ymax ) const
{
  assert( x.size()%2 == 0 );
  int numparticles = x.size()/2;
  assert( numparticles == (int) radii.size() );
  
  xmin =  std::numeric_limits<double>::infinity();
  xmax = -std::numeric_limits<double>::infinity();
  ymin =  std::numeric_limits<double>::infinity();
  ymax = -std::numeric_limits<double>::infinity();

  for( int i = 0; i < numparticles; ++i )
  {
    if( x(2*i)   - radii[i] < xmin ) xmin = x(2*i)   - radii[i];
    if( x(2*i)   + radii[i] > xmax ) xmax = x(2*i)   + radii[i];
    if( x(2*i+1) - radii[i] < ymin ) ymin = x(2*i+1) - radii[i];
    if( x(2*i+1) + radii[i] > ymax ) ymax = x(2*i+1) + radii[i];
  }

  // Also include particle paths in bounding box computation
  for( std::vector<renderingutils::ParticlePath>::size_type i = 0; i < m_particle_paths.size(); ++i )
  {
    const std::list<Vector2s>& ppath = m_particle_paths[i].getPath();    
    for( std::list<Vector2s>::const_iterator itr = ppath.begin(); itr != ppath.end(); ++itr )
    {
      if( itr->x() < xmin ) xmin = itr->x();
      if( itr->x() > xmax ) xmax = itr->x();
      if( itr->y() < ymin ) ymin = itr->y();
      if( itr->y() > ymax ) ymax = itr->y();
    }
  }    
}

void TwoDSceneSVGRenderer::computeSimToImageMap( const VectorXs& x, const std::vector<scalar>& radii, scalar& scale, scalar& xmin, scalar& ymin, scalar& xshift, scalar& yshift ) const
{
  scalar xmax, ymax;
  computeBoundingBox( x, radii, xmin, xmax, ymin, ymax );
  scalar xwidth = xmax-xmin;
  scalar ywidth = ymax-ymin;

  scalar scalex = ((scalar)m_w)/xwidth;
  scalar scaley = ((scalar)m_h)/ywidth;
  scale  = std::min(scalex,scaley); // Multiplication is to give the border some extra width

  scalar remainderx = ((scalar)m_w) - scale*xwidth;
  scalar remaindery = ((scalar)m_h) - scale*ywidth;

  xshift = 0.5*remainderx;
  yshift = 0.5*remaindery;
}

void TwoDSceneSVGRenderer::renderSolidCircle( std::fstream& file, const Vector2s& center, const scalar& r, const renderingutils::Color& color ) const
{  
  file << "<circle fill=\"#";
  file << intToHexString(floor(255.0*color.r+0.5)) << intToHexString(floor(255.0*color.g+0.5)) << intToHexString(floor(255.0*color.b+0.5)) << "\"" << " stroke=\"#";
  file << intToHexString(floor(255.0*color.r+0.5)) << intToHexString(floor(255.0*color.g+0.5)) << intToHexString(floor(255.0*color.b+0.5)) << "\"" << " stroke-width=\"0\" cx=\"";
  file << center.x();
  file << "\" cy=\"";
  file << center.y();
  file << "\" r=\"";
  file << r;
  file << "\"/>" << std::endl;    
}

void TwoDSceneSVGRenderer::renderCircle( std::fstream& file, const Vector2s& center, const scalar& r, const renderingutils::Color& color ) const
{
  file << "<circle cx=\"" << center.x() << "\" cy=\"" << center.y() << "\" r=\"" << r << "\" stroke=\"#";
  file << intToHexString(floor(255.0*color.r+0.5)) << intToHexString(floor(255.0*color.g+0.5)) << intToHexString(floor(255.0*color.b+0.5)) << "\" stroke-width=\"1\" fill=\"none\"/>" << std::endl;
}

void TwoDSceneSVGRenderer::renderSweptEdge( std::fstream& file, const Vector2s& x0, const Vector2s& x1, const scalar& r, const renderingutils::Color& color ) const
{
  scalar theta = atan2(x1.y()-x0.y(),x1.x()-x0.x());

  scalar rx = -r*sin(theta);
  scalar ry =  r*cos(theta);

  scalar p0x = x0.x() + rx;
  scalar p0y = x0.y() + ry;
  scalar p1x = x0.x() - rx;
  scalar p1y = x0.y() - ry;
  scalar p2x = x1.x() - rx;
  scalar p2y = x1.y() - ry;
  scalar p3x = x1.x() + rx;
  scalar p3y = x1.y() + ry;

  file << "<polygon points=\"" << p0x << "," << p0y << " " << p1x << "," << p1y << " " << p2x << "," << p2y << " " << p3x << "," << p3y;
  file << "\" style=\"fill:#" << intToHexString(floor(255.0*color.r+0.5)) << intToHexString(floor(255.0*color.g+0.5)) << intToHexString(floor(255.0*color.b+0.5));
  file << "; stroke:#000000;stroke-width:0\"/>" << std::endl;

  renderSolidCircle( file, x0, r, color );
  renderSolidCircle( file, x1, r, color );
}

void TwoDSceneSVGRenderer::renderShared( std::fstream& file, const VectorXs& x, const std::vector<std::pair<int,int> >& edges, const std::vector<scalar>& radii, const std::vector<scalar>& edgeradii, const scalar& scale, const scalar& xmin, const scalar& ymin, const scalar& xshift, const scalar& yshift  ) const
{
  int numparticles = x.size()/2;
  
  for( std::vector<renderingutils::ParticlePath>::size_type i = 0; i < m_particle_paths.size(); ++i )
  {
    const std::list<Vector2s>& ppath = m_particle_paths[i].getPath();
    const renderingutils::Color& pathcolor = m_particle_paths[i].getColor();
    
    file << "<polyline points=\"";
    
    for( std::list<Vector2s>::const_iterator itr = ppath.begin(); itr != ppath.end(); ++itr )
    {
      Vector2s point;
      point << scale*(itr->x()-xmin) + xshift, ((scalar)m_h) - scale*(itr->y()-ymin) - yshift;
      file << point.x() << "," << point.y() << " ";
    }
    
    file << "\" style=\"fill:none;stroke:#";
    file << intToHexString(floor(255.0*pathcolor.r+0.5)) << intToHexString(floor(255.0*pathcolor.g+0.5)) << intToHexString(floor(255.0*pathcolor.b+0.5));
    file << ";stroke-width:1\"/>" << std::endl;
  }  
  
  // Render edges
  assert( edgeradii.size() == edges.size() );
  for( std::vector<std::pair<int,int> >::size_type i = 0; i < edges.size(); ++i )
  {
    assert( edges[i].first >= 0 );  assert( edges[i].first < m_scene.getNumParticles() );
    assert( edges[i].second >= 0 ); assert( edges[i].second < m_scene.getNumParticles() );
    
    int i0 = edges[i].first;
    int i1 = edges[i].second;
    
    Vector2s p0;
    p0 << scale*(x(2*i0)-xmin) + xshift, ((scalar)m_h) - scale*(x(2*i0+1)-ymin) - yshift;
    
    Vector2s p1;
    p1 << scale*(x(2*i1)-xmin) + xshift, ((scalar)m_h) - scale*(x(2*i1+1)-ymin) - yshift;
    
    renderSweptEdge( file, p0, p1, scale*edgeradii[i], m_edge_colors[i] );
  }
  
  // Render particles
  for( int i = 0; i < numparticles; ++i )
  {
    Vector2s center;
    center << scale*(x(2*i)-xmin) + xshift, ((scalar)m_h) - scale*(x(2*i+1)-ymin) - yshift;
    renderSolidCircle( file, center, scale*radii[i], m_particle_colors[i] );
  }  
}


void TwoDSceneSVGRenderer::renderScene( const std::string& filename ) const
{
  const VectorXs& x = m_scene.getX();
  assert( x.size()%2 == 0 );
  assert( 2*m_scene.getNumParticles() == x.size() );
  int numparticles = x.size()/2;
  const std::vector<scalar>& radii = m_scene.getRadii();
  assert( numparticles == (int) radii.size() );
  
  std::fstream file(filename.c_str(), std::fstream::out);
  if(!file)
  {
    std::cerr << "Failure writing SVG file!" << std::endl;
    exit(1);
  }
  
  scalar scale, xmin, ymin, xshift, yshift;
  computeSimToImageMap( x, radii, scale, xmin, ymin, xshift, yshift );
  
  file << "<?xml version=\"1.0\" encoding=\"utf-8\"?> <!-- Generator: Adobe Illustrator 13.0.0, SVG Export Plug-In . SVG Version: 6.00 Build 14948)  --> <svg version=\"1.2\" baseProfile=\"tiny\" id=\"Layer_1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" x=\"0px\" y=\"0px\" width=\"";
  file << m_w;
  file << "px\" height=\"";
  file << m_h;
  file << "px\" viewBox=\"0 0 ";
  file << m_w << " " << m_h;
  file << "\" xml:space=\"preserve\">" << std::endl;

  // Simulate a background color by rendering a large colored quad
  file << "<polygon points=\"" << 0 << "," << 0 << " " << m_w << "," << 0 << " " << m_w << "," << m_h << " " << 0 << "," << m_h;
  file << "\" style=\"fill:#" << intToHexString(floor(255.0*m_bgcolor.r+0.5)) << intToHexString(floor(255.0*m_bgcolor.g+0.5)) << intToHexString(floor(255.0*m_bgcolor.b+0.5));
  file << "; stroke:#000000;stroke-width:0\"/>" << std::endl;  
  
  
  const std::vector<std::pair<int,int> >& edges = m_scene.getEdges();
  const std::vector<scalar>& edgeradii = m_scene.getEdgeRadii();
  renderShared( file, x, edges, radii, edgeradii, scale, xmin, ymin, xshift, yshift );

  file << "</svg>" << std::endl;
  
  file.close();
}

void TwoDSceneSVGRenderer::renderComparisonScene( const std::string& filename, const TwoDScene& otherscene, const scalar& eps ) const
{
  //std::cout << "comparison" << std::endl;

  const VectorXs& x = m_scene.getX();
  const VectorXs& v = m_scene.getV();
  assert( x.size()%2 == 0 );
  assert( 2*m_scene.getNumParticles() == x.size() );
  int numparticles = x.size()/2;
  const std::vector<scalar>& radii = m_scene.getRadii();
  assert( numparticles == (int) radii.size() );
  
  std::fstream file(filename.c_str(), std::fstream::out);
  if(!file)
  {
    std::cerr << "Failure writing SVG file!" << std::endl;
    exit(1);
  }
  
  scalar scale, xmin, ymin, xshift, yshift;
  computeSimToImageMap( x, radii, scale, xmin, ymin, xshift, yshift );
  
  file << "<?xml version=\"1.0\" encoding=\"utf-8\"?> <!-- Generator: Adobe Illustrator 13.0.0, SVG Export Plug-In . SVG Version: 6.00 Build 14948)  --> <svg version=\"1.2\" baseProfile=\"tiny\" id=\"Layer_1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" x=\"0px\" y=\"0px\" width=\"";
  file << m_w;
  file << "px\" height=\"";
  file << m_h;
  file << "px\" viewBox=\"0 0 ";
  file << m_w << " " << m_h;
  file << "\" xml:space=\"preserve\">" << std::endl;
  
  // Simulate a background color by rendering a large colored quad
  file << "<polygon points=\"" << 0 << "," << 0 << " " << m_w << "," << 0 << " " << m_w << "," << m_h << " " << 0 << "," << m_h;
  file << "\" style=\"fill:#" << intToHexString(floor(255.0*m_bgcolor.r+0.5)) << intToHexString(floor(255.0*m_bgcolor.g+0.5)) << intToHexString(floor(255.0*m_bgcolor.b+0.5));
  file << "; stroke:#000000;stroke-width:0\"/>" << std::endl;
  
  const std::vector<std::pair<int,int> >& edges = m_scene.getEdges();
  const std::vector<scalar>& edgeradii = m_scene.getEdgeRadii();
  renderShared( file, x, edges, radii, edgeradii, scale, xmin, ymin, xshift, yshift );
  
  
  
  const VectorXs& otherx = otherscene.getX();  
  const VectorXs& otherv = otherscene.getV();

  for( int i = 0; i < numparticles; ++i )
  {
    scalar x_resid = (otherx.segment<2>(2*i)-x.segment<2>(2*i)).norm();
    scalar v_resid = (otherv.segment<2>(2*i)-v.segment<2>(2*i)).norm();
    if( x_resid > eps || v_resid > eps )
    {      
      Vector2s center;
      center << scale*(x(2*i)-xmin) + xshift, ((scalar)m_h) - scale*(x(2*i+1)-ymin) - yshift;
      renderCircle( file, center, 1.5*scale*radii[i], renderingutils::Color(1.0,0.0,0.0) );      
    }
  }
  
  
  file << "</svg>" << std::endl;
  
  file.close();  
  
}




//void TwoDSceneRenderer::circleMajorResiduals( const TwoDScene& oracle_scene, const TwoDScene& testing_scene, scalar eps ) const
//{
//  assert(   oracle_scene.getNumParticles() == testing_scene.getNumParticles() );
//  assert( 2*oracle_scene.getNumParticles() == oracle_scene.getX().size() );
//  assert( 2*oracle_scene.getNumParticles() == testing_scene.getX().size() );
//  assert( 2*oracle_scene.getNumParticles() == oracle_scene.getV().size() );
//  assert( 2*oracle_scene.getNumParticles() == testing_scene.getV().size() );
//
//  const VectorXs& oracle_x = oracle_scene.getX();
//  const VectorXs& testing_x = testing_scene.getX();
//
//  const VectorXs& oracle_v = oracle_scene.getV();
//  const VectorXs& testing_v = testing_scene.getV();
//
//  glColor3d(1.0,0.0,0.0);
//  for( int i = 0; i < oracle_scene.getNumParticles(); ++i )
//  {
//    scalar x_resid = (oracle_x.segment<2>(2*i)-testing_x.segment<2>(2*i)).norm();
//    scalar v_resid = (oracle_v.segment<2>(2*i)-testing_v.segment<2>(2*i)).norm();
//    if( x_resid > eps || v_resid > eps )
//      renderCircle( oracle_x.segment<2>(2*i), 2.0*oracle_scene.getRadius(i) );
//  }
//}

