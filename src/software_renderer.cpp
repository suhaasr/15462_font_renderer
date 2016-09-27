#include "software_renderer.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cassert>

#include "triangulation.h"

#define MAX(x,y) (((x)>(y))?(x):(y))
#define MIN(x,y) (((x)<(y))?(x):(y))

#define DEBUG

#ifdef DEBUG
#define DEBUG_ASSERT(x) (assert(x))
#endif

#ifndef DEBUG
#define DEBUG_ASSERT(x)
#endif

#define FAIL (assert(1 == 0))


using namespace std;

namespace CMU462 {


// Implements SoftwareRenderer //

void SoftwareRendererImp::draw_svg( SVG& svg ) {

  // set top level transformation
  transformation = canvas_to_screen;

  // draw all elements
  for ( size_t i = 0; i < svg.elements.size(); ++i ) {
    draw_element(svg.elements[i]);
  }

  // draw canvas outline
  Vector2D a = transform(Vector2D(    0    ,     0    )); a.x--; a.y++;
  Vector2D b = transform(Vector2D(svg.width,     0    )); b.x++; b.y++;
  Vector2D c = transform(Vector2D(    0    ,svg.height)); c.x--; c.y--;
  Vector2D d = transform(Vector2D(svg.width,svg.height)); d.x++; d.y--;

  rasterize_line(a.x, a.y, b.x, b.y, Color::Black);
  rasterize_line(a.x, a.y, c.x, c.y, Color::Black);
  rasterize_line(d.x, d.y, b.x, b.y, Color::Black);
  rasterize_line(d.x, d.y, c.x, c.y, Color::Black);

  // resolve and send to render target
  resolve();

}

void SoftwareRendererImp::set_sample_rate( size_t sample_rate ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->sample_rate = sample_rate;
  if (this->supersample_target != NULL)
  {
    delete [] this->supersample_target;
  }
  this->supersample_target = 
    new unsigned char[(4 /* *sample_rate */ *target_w)*( /* sample_rate * */target_h)];
  memset(supersample_target,255,4*target_w*target_h/* *sample_rate*sample_rate */);
}

void SoftwareRendererImp::set_render_target( unsigned char* render_target,
                                             size_t width, size_t height ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->render_target = render_target;
  this->target_w = width;
  this->target_h = height;
  if (this->supersample_target != NULL)
  {
    delete [] this->supersample_target;
  }
  this->supersample_target = new unsigned char[4*target_w*target_h];
  memset(supersample_target,255,4*target_w*target_h);
}

void SoftwareRendererImp::draw_element( SVGElement* element ) {

  // Task 5 (part 1):
  // Modify this to implement the transformation stack

  switch(element->type) {
    case POINT:
      draw_point(static_cast<Point&>(*element));
      break;
    case LINE:
      draw_line(static_cast<Line&>(*element));
      break;
    case POLYLINE:
      draw_polyline(static_cast<Polyline&>(*element));
      break;
    case RECT:
      draw_rect(static_cast<Rect&>(*element));
      break;
    case POLYGON:
      draw_polygon(static_cast<Polygon&>(*element));
      break;
    case ELLIPSE:
      draw_ellipse(static_cast<Ellipse&>(*element));
      break;
    case IMAGE:
      draw_image(static_cast<Image&>(*element));
      break;
    case GROUP:
      draw_group(static_cast<Group&>(*element));
      break;
    default:
      break;
  }

}


// Primitive Drawing //

void SoftwareRendererImp::draw_point( Point& point ) {
  Vector2D p = transform(point.position);
  rasterize_point( p.x, p.y, point.style.fillColor );
}

void SoftwareRendererImp::draw_line( Line& line ) { 

  Vector2D p0 = transform(line.from);
  Vector2D p1 = transform(line.to);
  rasterize_line( p0.x, p0.y, p1.x, p1.y, line.style.strokeColor );

}

void SoftwareRendererImp::draw_polyline( Polyline& polyline ) {

  Color c = polyline.style.strokeColor;

  if( c.a != 0 ) {
    int nPoints = polyline.points.size();
    for( int i = 0; i < nPoints - 1; i++ ) {
      Vector2D p0 = transform(polyline.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polyline.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_rect( Rect& rect ) {

  Color c;
  
  // draw as two triangles
  float x = rect.position.x;
  float y = rect.position.y;
  float w = rect.dimension.x;
  float h = rect.dimension.y;

  Vector2D p0 = transform(Vector2D(   x   ,   y   ));
  Vector2D p1 = transform(Vector2D( x + w ,   y   ));
  Vector2D p2 = transform(Vector2D(   x   , y + h ));
  Vector2D p3 = transform(Vector2D( x + w , y + h ));
  
  // draw fill
  c = rect.style.fillColor;
  if (c.a != 0 ) {
    rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    rasterize_triangle( p2.x, p2.y, p1.x, p1.y, p3.x, p3.y, c );
  }

  // draw outline
  c = rect.style.strokeColor;
  if( c.a != 0 ) {
    rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    rasterize_line( p1.x, p1.y, p3.x, p3.y, c );
    rasterize_line( p3.x, p3.y, p2.x, p2.y, c );
    rasterize_line( p2.x, p2.y, p0.x, p0.y, c );
  }

}

void SoftwareRendererImp::draw_polygon( Polygon& polygon ) {

  Color c;

  // draw fill
  c = polygon.style.fillColor;
  if( c.a != 0 ) {

    // triangulate
    vector<Vector2D> triangles;
    triangulate( polygon, triangles );

    // draw as triangles
    for (size_t i = 0; i < triangles.size(); i += 3) {
      Vector2D p0 = transform(triangles[i + 0]);
      Vector2D p1 = transform(triangles[i + 1]);
      Vector2D p2 = transform(triangles[i + 2]);
      rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    }
  }

  // draw outline
  c = polygon.style.strokeColor;
  if( c.a != 0 ) {
    int nPoints = polygon.points.size();
    for( int i = 0; i < nPoints; i++ ) {
      Vector2D p0 = transform(polygon.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polygon.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_ellipse( Ellipse& ellipse ) {

  // Extra credit 

}

void SoftwareRendererImp::draw_image( Image& image ) {

  Vector2D p0 = transform(image.position);
  Vector2D p1 = transform(image.position + image.dimension);

  rasterize_image( p0.x, p0.y, p1.x, p1.y, image.tex );
}

void SoftwareRendererImp::draw_group( Group& group ) {

  for ( size_t i = 0; i < group.elements.size(); ++i ) {
    draw_element(group.elements[i]);
  }

}

// Rasterization //

// The input arguments in the rasterization functions 
// below are all defined in screen space coordinates

void SoftwareRendererImp::rasterize_point( float x, float y, Color color ) {

  // fill in the nearest pixel
  DEBUG_ASSERT(this->supersample_target != NULL);
  int sx = (int) floor(x);
  int sy = (int) floor(y);
  int offset = 4 * (sx + sy * target_w);

  // check bounds
  if ( sx < 0 || sx >= target_w ) return;
  if ( sy < 0 || sy >= target_h ) return;

  // fill sample - NOT doing alpha blending!
  supersample_target[offset    ] = (uint8_t) (color.r * 255);
  supersample_target[offset + 1] = (uint8_t) (color.g * 255);
  supersample_target[offset + 2] = (uint8_t) (color.b * 255);
  supersample_target[offset + 3] = (uint8_t) (color.a * 255);

}

void SoftwareRendererImp::rasterize_line( float x0, float y0,
                                          float x1, float y1,
                                          Color color) {
  // Task 2: 
  // Implement line rasterization
  int roundX0 = (int) round(x0);
  int roundY0 = (int) round(y0);
  int roundX1 = (int) round(x1);
  int roundY1 = (int) round(y1);
  int epsilon = 0;
  int swap, deltaX, deltaY, currX, currY;

  //Make sure the points are left to right
  if (roundX1 < roundX0 || (roundX1 == roundX0 && roundY0 > roundY1))
  {
    swap = roundX0; roundX0 = roundX1; roundX1 = swap;
    swap = roundY0; roundY0 = roundY1; roundY1 = swap;
  }

  deltaY = roundY1 - roundY0;
  deltaX = roundX1 - roundX0;
  
  if (deltaY >= 0) //Line has positive, undefined, or horizontal slope
  {
    if (deltaX >= deltaY) //Line moves more right than "up"
    {
      currY = roundY0;
      for (currX = roundX0; currX <= roundX1; currX++)
      {
        rasterize_point(currX,currY,color);
        epsilon += deltaY;
        if((epsilon << 1) >= deltaX)
        {
          currY ++; epsilon -= deltaX;
        }
      }
    }
    else //Line moves more up than right, so use y as the primary loop var
    {
      currX = roundX0;
      for (currY = roundY0; currY <= roundY1; currY++)
      {
        rasterize_point(currX,currY,color);
        epsilon += deltaX;
        if((epsilon << 1) >= deltaY)
        {
          currX ++; epsilon -= deltaY;
        }
      }
    }
  }
  else //Line has negative slope
  {
    if (deltaX >= -deltaY) //Line moves more right than "down"
    {
      currY = roundY0;
      for (currX = roundX0; currX <= roundX1; currX++) 
      {
        rasterize_point(currX,currY,color);
        epsilon -= deltaY;
        if ((epsilon << 1) >= deltaX)
        {
          currY--; epsilon-= deltaX;
        }
      }
    }
    else //Line moves more "down" than right
    {
      currX = roundX0;
      for (currY = roundY0; currY >= roundY1; currY--)
      {
        rasterize_point(currX, currY, color);
        epsilon += deltaX;
        if ((epsilon << 1) >= -deltaY)
        {
          currX++; epsilon += deltaY;
        }
      }
    }
  }


}

bool inEdge(float x0, float y0,
            float x1, float y1,
            float baseX, float baseY,
            float testX, float testY)
{
  return (((x1-x0)*(testY-y0) - (y1-y0)*(testX-x0)) *
          ((x1-x0)*(baseY-y0) - (y1-y0)*(baseX-x0))) >= 0;
}

bool inTriangle(float x0, float y0,
                float x1, float y1, 
                float x2, float y2, 
                float testX, float testY)
{
  return (inEdge(x2,y2,x1,y1,x0,y0,testX,testY) &&
          inEdge(x1,y1,x0,y0,x2,y2,testX,testY) &&
          inEdge(x2,y2,x0,y0,x1,y1,testX,testY));
}

void SoftwareRendererImp::rasterize_triangle( float x0, float y0,
                                              float x1, float y1,
                                              float x2, float y2,
                                              Color color ) {
  // Task 3: 
  // Implement triangle rasterization
  float minX = MIN(MIN(x0,x1),x2);
  float maxX = MAX(MAX(x0,x1),x2);
  float minY = MIN(MIN(y0,y1),y2);
  float maxY = MAX(MAX(y0,y1),y2);
  for (float currX = floor(minX) - .5; currX <= ceil(maxX) + .5; currX += 1)
  {
    for (float currY = floor(minY) - .5; currY <= ceil(maxY) + .5; currY += 1)
    {
      //Could improve this by calculating side tests for triangle vertices
      //outside this loop
      if (inTriangle(x0,y0,x1,y1,x2,y2,currX,currY))
      {
        rasterize_point(currX,currY,color);
      }
    }
  }
}

void SoftwareRendererImp::rasterize_image( float x0, float y0,
                                           float x1, float y1,
                                           Texture& tex ) {
  // Task 6: 
  // Implement image rasterization

}

void SoftwareRendererImp::resolve_point(int x, int y)
{
  DEBUG_ASSERT(this->supersample_target != NULL);
  int offset = 4 * (x + y * this->target_w);
  this->render_target[offset    ] = (uint8_t) this->supersample_target[offset    ];
  this->render_target[offset + 1] = (uint8_t) this->supersample_target[offset + 1];
  this->render_target[offset + 2] = (uint8_t) this->supersample_target[offset + 2];
  this->render_target[offset + 3] = (uint8_t) this->supersample_target[offset + 3];
}

// resolve samples to render target
void SoftwareRendererImp::resolve( void ) {

  // Task 4: 
  // Implement supersampling
  // You may also need to modify other functions marked with "Task 4".
  DEBUG_ASSERT(supersample_target != NULL);
  for (int currX = 0; currX < target_w; currX++)
  {
    for (int currY = 0; currY < target_h; currY++)
    {
      resolve_point(currX,currY);
    }
  }
  memset(supersample_target, 255, 4*target_w*target_h/**sample_rate*sample_rate*/);
  return;

}


} // namespace CMU462
