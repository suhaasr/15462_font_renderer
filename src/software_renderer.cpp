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
  this->supersample_target_size = 4 *target_w * target_h * sample_rate * sample_rate;
  this->sample_rate = sample_rate;
  if (this->supersample_target != NULL)
  {
    delete [] this->supersample_target;
  }
  this->supersample_target = 
    new unsigned char[supersample_target_size];
  memset(supersample_target,255,supersample_target_size);
}

void SoftwareRendererImp::set_render_target( unsigned char* render_target,
                                             size_t width, size_t height ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->render_target = render_target;
  this->target_w = width;
  this->target_h = height;
  this->supersample_target_size = 4 *target_w * target_h * sample_rate * sample_rate;
  if (this->supersample_target != NULL)
  {
    delete [] this->supersample_target;
  }
  this->supersample_target = new unsigned char[this->supersample_target_size];
  memset(supersample_target,255,4*target_w*target_h);
}

void SoftwareRendererImp::draw_element( SVGElement* element ) {

  // Task 5 (part 1):
  // Modify this to implement the transformation stack

  Matrix3x3 prevTransform = this->transformation;
  this->transformation = transformation * element->transform;

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
this->transformation = prevTransform;
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

void SoftwareRendererImp::fill_pixel(int x, int y, Color color) {
  unsigned char* pixel_start = &(this->supersample_target[4 * sample_rate * sample_rate * (x + y * target_w)]);
  int inner_offset;
  DEBUG_ASSERT(4 * sample_rate * sample_rate * (x + y * target_w) + 4 * ((sample_rate - 1) + (sample_rate - 1) * sample_rate) + 3 < this->supersample_target_size);
  for (int py = 0; py < sample_rate; py++)
  {
    for (int px = 0; px < sample_rate; px++)
    {
      rasterize_sample(x*sample_rate+px, y*sample_rate+py, color);
    }
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

  fill_pixel(x,y,color);
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

void SoftwareRendererImp::rasterize_sample(int sampleX, int sampleY, Color color) {
  DEBUG_ASSERT(this->supersample_target != NULL);
  int x = sampleX / sample_rate;
  int y = sampleY / sample_rate;
  int px = sampleX % sample_rate;
  int py = sampleY % sample_rate;
  int offset = 4 * (sample_rate * sample_rate * (x + y * this->target_w) + (px + py * sample_rate));
  if (offset < 0 || offset >= supersample_target_size) return;
  float oldR = this->supersample_target[offset    ];
  float oldG = this->supersample_target[offset + 1];
  float oldB = this->supersample_target[offset + 2];
  float oldA = this->supersample_target[offset + 3];
  this->supersample_target[offset    ] = (uint8_t) (((1 - color.a) * oldR/255.0 + color.r * color.a)*255);
  this->supersample_target[offset + 1] = (uint8_t) (((1 - color.a) * oldG/255.0 + color.g * color.a)*255);
  this->supersample_target[offset + 2] = (uint8_t) (((1 - color.a) * oldB/255.0 + color.b * color.a)*255);
  this->supersample_target[offset + 3] = (uint8_t) ((1-(1-color.a)*(1-oldA/255.0))*255);
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
  float minX = MAX(MIN(MIN(x0,x1),x2),0.0);
  float maxX = MIN(MAX(MAX(x0,x1),x2),target_w);
  float minY = MAX(MIN(MIN(y0,y1),y2),0.0);
  float maxY = MIN(MAX(MAX(y0,y1),y2),target_h);
  //sampleX and sampleY mark sample point pixels in the supersample_target
  for (int sampleX = int(floor(minX) * sample_rate); 
       sampleX < (int) (ceil(maxX) * sample_rate); 
       sampleX += 1)
  {
    for (int sampleY = int(floor(minY) * sample_rate);
         sampleY < (int) (ceil(maxY)* sample_rate);
         sampleY += 1)
    {
      //Could improve this by calculating side tests for triangle vertices
      //outside this loop

      //divide by sample_rate to get the actual x and y values that we are trying to test.
      if (inTriangle(x0,y0,x1,y1,x2,y2,(sampleX+.5)/sample_rate,(sampleY+.5)/sample_rate))
      {
        rasterize_sample(sampleX,sampleY,color);
      }
    }
  }
}

void SoftwareRendererImp::rasterize_image( float x0, float y0,
                                           float x1, float y1,
                                           Texture& tex ) {
  // Task 6: 
  // Implement image rasterization
  for(float x = MAX(0,x0)*sample_rate; x < MIN(target_w, x1)*sample_rate; x++) 
  {
    for(float y = MAX(0,y0)*sample_rate; y < MIN(target_w, y1)*sample_rate; y++) 
    {
      rasterize_sample(x,y,sampler->sample_trilinear(tex,(x/sample_rate-x0)/(x1-x0),(y/sample_rate-y0)/(y1-y0),1.0/(x1-x0),1.0/(y1-y0)));
    }
  }
}

float SoftwareRendererImp::resolve_channel(int x, int y, int channel_offset)
{
  unsigned char* pixel_start = &(this->supersample_target[4 * sample_rate * sample_rate * (x + y * target_w) + channel_offset]);
  float channel_sum = 0.0;
  int inner_offset;
  for (int channel_py = 0; channel_py < sample_rate; channel_py++)
  {
    for (int channel_px = 0; channel_px < sample_rate; channel_px++)
    {
      inner_offset = 4 * (channel_px + channel_py * sample_rate);
      channel_sum += pixel_start[inner_offset];
    }
  }
  return channel_sum/(sample_rate*sample_rate);
}

void SoftwareRendererImp::resolve_pixel(int x, int y)
{
  DEBUG_ASSERT(this->supersample_target != NULL);
  int render_offset = 4 * (x + y * this->target_w);
  int offset = 4 * sample_rate * sample_rate * (x + y * this->target_w);
  this->render_target[render_offset    ] = (uint8_t) resolve_channel(x, y, 0);
  this->render_target[render_offset + 1] = (uint8_t) resolve_channel(x, y, 1);
  this->render_target[render_offset + 2] = (uint8_t) resolve_channel(x, y, 2);
  this->render_target[render_offset + 3] = (uint8_t) resolve_channel(x, y, 3);
}

// resolve samples to render target
void SoftwareRendererImp::resolve( void ) {

  // Task 4: 
  // Implement supersampling
  // You may also need to modify other functions marked with "Task 4".
  memset(render_target, 255, 4 * target_w * target_h);
  DEBUG_ASSERT(supersample_target != NULL);
  for (int currX = 0; currX < target_w; currX++)
  {
    for (int currY = 0; currY < target_h; currY++)
    {
      resolve_pixel(currX,currY);
    }
  }
  memset(supersample_target, 255, 4*target_w*target_h*sample_rate*sample_rate);
  return;

}


} // namespace CMU462
