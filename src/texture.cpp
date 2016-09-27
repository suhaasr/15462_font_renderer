#include "texture.h"

#include <assert.h>
#include <iostream>
#include <algorithm>

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

inline void uint8_to_float( float dst[4], unsigned char* src ) {
  uint8_t* src_uint8 = (uint8_t *)src;
  dst[0] = src_uint8[0] / 255.f;
  dst[1] = src_uint8[1] / 255.f;
  dst[2] = src_uint8[2] / 255.f;
  dst[3] = src_uint8[3] / 255.f;
}

inline void float_to_uint8( unsigned char* dst, float src[4] ) {
  uint8_t* dst_uint8 = (uint8_t *)dst;
  dst_uint8[0] = (uint8_t) ( 255.f * max( 0.0f, min( 1.0f, src[0])));
  dst_uint8[1] = (uint8_t) ( 255.f * max( 0.0f, min( 1.0f, src[1])));
  dst_uint8[2] = (uint8_t) ( 255.f * max( 0.0f, min( 1.0f, src[2])));
  dst_uint8[3] = (uint8_t) ( 255.f * max( 0.0f, min( 1.0f, src[3])));
}

void Sampler2DImp::generate_mips(Texture& tex, int startLevel) {

  // NOTE(sky): 
  // The starter code allocates the mip levels and generates a level 
  // map simply fills each level with a color that differs from its
  // neighbours'. The reference solution uses trilinear filtering
  // and it will only work when you have mipmaps.

  // Task 7: Implement this

  // check start level
  if ( startLevel >= tex.mipmap.size() ) {
    std::cerr << "Invalid start level"; 
  }

  // allocate sublevels
  int baseWidth  = tex.mipmap[startLevel].width;
  int baseHeight = tex.mipmap[startLevel].height;
  int numSubLevels = (int)(log2f( (float)max(baseWidth, baseHeight)));

  numSubLevels = min(numSubLevels, kMaxMipLevels - startLevel - 1);
  tex.mipmap.resize(startLevel + numSubLevels + 1);

  int width  = baseWidth;
  int height = baseHeight;
  for (int i = 1; i <= numSubLevels; i++) {

    MipLevel& level = tex.mipmap[startLevel + i];

    // handle odd size texture by rounding down
    width  = max( 1, width  / 2); assert(width  > 0);
    height = max( 1, height / 2); assert(height > 0);

    level.width = width;
    level.height = height;
    level.texels = vector<unsigned char>(4 * width * height);

  }
  /*
  // fill all 0 sub levels with interchanging colors
  Color colors[3] = { Color(1,0,0,1), Color(0,1,0,1), Color(0,0,1,1) };
  for(size_t i = 1; i < tex.mipmap.size(); ++i) {

    Color c = colors[i % 3];
    MipLevel& mip = tex.mipmap[i];

    for(size_t i = 0; i < 4 * mip.width * mip.height; i += 4) {
      float_to_uint8( &mip.texels[i], &c.r );
    }
  }
  */
  for(size_t i = 1; i < numSubLevels; ++i)
  {
    MipLevel& mip = tex.mipmap[i];
    MipLevel& lowerMip = tex.mipmap[i-1];
    float rSum, gSum, bSum, aSum;
    for (size_t x = 0; x < mip.width; x += 1) 
    {
      for (size_t y = 0; y < mip.height; y += 1) 
      {
        rSum = gSum = bSum = aSum = 0.0;
        for (size_t lowerLevelx = 2*x; lowerLevelx < 2*(x + 1); lowerLevelx += 1) 
        {
          for (size_t lowerLevely = 2*y; lowerLevely < 2*(y+1); lowerLevely += 1) 
          {
            int fourTimesXPlusYTimesTexMipmapWidth = 4*(lowerLevelx+lowerLevely*tex.mipmap[i-1].width);
            rSum += lowerMip.texels[fourTimesXPlusYTimesTexMipmapWidth + 0];
            gSum += lowerMip.texels[fourTimesXPlusYTimesTexMipmapWidth + 1];
            bSum += lowerMip.texels[fourTimesXPlusYTimesTexMipmapWidth + 2];
            aSum += lowerMip.texels[fourTimesXPlusYTimesTexMipmapWidth + 3];
          }
        }
        mip.texels[4*(x+y*mip.width) + 0] = (uint8_t)(rSum/4.0);
        mip.texels[4*(x+y*mip.width) + 1] = (uint8_t)(gSum/4.0);
        mip.texels[4*(x+y*mip.width) + 2] = (uint8_t)(bSum/4.0);
        mip.texels[4*(x+y*mip.width) + 3] = (uint8_t)(aSum/4.0);
      }
    }    
  }

}

Color Sampler2DImp::sample_nearest(Texture& tex, 
                                   float u, float v, 
                                   int level) {

  // Task 6: Implement nearest neighbour interpolation
  
  // return magenta for invalid level
   if (level >= 0 && level <= 14) {
    int x = (int) round(float(u*tex.mipmap[level].width));
    int y = (int) round(float(v*tex.mipmap[level].height));
    int offset = 4*(x+y*tex.mipmap[level].width);
    float r = tex.mipmap[level].texels[offset    ];
    float g = tex.mipmap[level].texels[offset + 1];
    float b = tex.mipmap[level].texels[offset + 2];
    float a = tex.mipmap[level].texels[offset + 3];
    return Color(r/255.0,g/255.0,b/255.0,a/255.0);
  }
  // return magenta for invalid level
  return Color(1,0,1,1);

}

float interpolateX(int xLo, int xHi, int y, float loWeight, float hiWeight, MipLevel& texLevel, int colorOffset) {
  int offset = 4*(xLo + y * texLevel.width) + colorOffset;
  return texLevel.texels[offset] * loWeight + texLevel.texels[offset + 4*(xHi-xLo)] * hiWeight;
}

float interpolateVals(float val1, float val2, float loWeight, float hiWeight) {
  return val1 * loWeight + val2 * hiWeight;
}

Color Sampler2DImp::sample_bilinear(Texture& tex, 
                                    float u, float v, 
                                    int level) {
  
  // Task 6: Implement bilinear filtering
  if(u < 0 || u > 1 || v < 0 || v > 1 || level < 0 || level > 14) {
    return Color(1,0,1,1); //return magenta for invalid level
  }
  //subtract .5 because pixel indices are offset .5 from their true coordinates
  float texX = u * tex.mipmap[level].width - .5;
  float texY = v * tex.mipmap[level].height - .5;
  int xLo = (int) floor(texX);
  int xHi = xLo + 1;
  //int xHi = (int) ceil(texX);
  int yLo = (int) floor(texY);
  int yHi = yLo + 1;
  //int yHi = (int) ceil(texY);
  MipLevel& texLevel = tex.mipmap[level];
  float hiXWeight = texX - xLo;
  float loXWeight = 1 - hiXWeight;
  float hiYWeight = texY - yLo;
  float loYWeight = 1 - hiYWeight;
  float red = interpolateVals(interpolateX(xLo, xHi, yLo, loXWeight, hiXWeight, texLevel, 0),
                              interpolateX(xLo, xHi, yHi, loXWeight, hiXWeight, texLevel, 0),
                              loYWeight,
                              hiYWeight);
  float green = interpolateVals(interpolateX(xLo, xHi, yLo, loXWeight, hiXWeight, texLevel, 1),
                                interpolateX(xLo, xHi, yHi, loXWeight, hiXWeight, texLevel, 1),
                                loYWeight,
                                hiYWeight);
  float blue = interpolateVals(interpolateX(xLo, xHi, yLo, loXWeight, hiXWeight, texLevel, 2),
                               interpolateX(xLo, xHi, yHi, loXWeight, hiXWeight, texLevel, 2),
                               loYWeight,
                               hiYWeight);
  float alpha = interpolateVals(interpolateX(xLo, xHi, yLo, loXWeight, hiXWeight, texLevel, 3),
                                interpolateX(xLo, xHi, yHi, loXWeight, hiXWeight, texLevel, 3),
                                loYWeight,
                                hiYWeight);
  return Color(red/255.0, green/255.0, blue/255.0, alpha/255.0);
}

Color Sampler2DImp::sample_trilinear(Texture& tex, 
                                     float u, float v, 
                                     float u_scale, float v_scale) {

  // Task 7: Implement trilinear filtering
  float dvdy = v_scale * tex.height;
  float dudx = u_scale * tex.width;
  float d = log2f(fmax(dvdy, dudx));
  float loWeight, hiWeight;
  int loD, hiD;
  Color hiColor, loColor;
  if (d >= 0)
  {
    loD = (int) floor(d);
    hiD = (int) ceil(d);
    hiWeight = d-loD;
    loWeight = 1-hiWeight;
    loColor = sample_bilinear(tex,u,v,loD);
    hiColor = sample_bilinear(tex,u,v,hiD);
    return Color(loColor.r*loWeight + hiColor.r*hiWeight,
                 loColor.g*loWeight + hiColor.g*hiWeight,
                 loColor.b*loWeight + hiColor.b*hiWeight,
                 loColor.a*loWeight + hiColor.a*hiWeight);
  }
  return sample_bilinear(tex,u,v,0);

}

} // namespace CMU462
