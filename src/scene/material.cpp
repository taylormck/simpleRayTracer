#include "ray.h"
#include "material.h"
#include "light.h"

#include "../fileio/bitmap.h"
#include "../fileio/pngimage.h"

using namespace std;
extern bool debugMode;

// Apply the phong model to this point on the surface of the object, returning
// the color of that point.
Vec3d Material::shade( Scene *scene, const ray& r, const isect& i ) const
{
  Vec3d final_color = scene->ambient() + ka(i);
  Vec3d diff_color = kd(i);
  Vec3d spec_color = ks(i);
  double shininess = shininess;

  if (debugMode) cout << "kd: " << diff_color << endl;

  //  Set up a few vectors (and a double) to be reused for each light
  Vec3d light_color;
  Vec3d light_dir;
  Vec3d isect_point = r.at(i.t);
  Vec3d shadow_att;
  double diff_co;

  // Loop through the lights and add their contribution
  for (vector<Light*>::const_iterator litr = scene->beginLights();
      litr != scene->endLights(); ++litr) {
    // Get values for the light
    Light* pLight = *litr;
    light_dir = pLight->getDirection(isect_point);

    // Check for shadows
    shadow_att = pLight->shadowAttenuation(isect_point);
    if (debugMode) cout << "shadow: " << shadow_att << endl;

    if (!shadow_att.iszero()) {
      // If not in shadow, do lighting
      light_color = pLight->getColor(diff_color);
      light_color = prod(light_color, shadow_att);

      // Calculate the diffuse lighting term
      diff_co = (light_dir * i.N) * pLight->distanceAttenuation(isect_point);
      light_color *= diff_co;

      // Add the diffuse lighting contribution from this light to the
      // final color result
      final_color += prod(diff_color, light_color);
    }
  }

  return final_color;
}

TextureMap::TextureMap( string filename ) {

  int start = filename.find_last_of('.');
  int end = filename.size() - 1;
  if (start >= 0 && start < end) {
    string ext = filename.substr(start, end);
    if (!ext.compare(".png")) {
      png_cleanup(1);
      if (!png_init(filename.c_str(), width, height)) {
        double gamma = 2.2;
        int channels, rowBytes;
        unsigned char* indata = png_get_image(gamma, channels, rowBytes);
        int bufsize = rowBytes * height;
        data = new unsigned char[bufsize];
        for (int j = 0; j < height; j++)
          for (int i = 0; i < rowBytes; i += channels)
            for (int k = 0; k < channels; k++)
              *(data + k + i + j * rowBytes) = *(indata + k + i + (height - j - 1) * rowBytes);
        png_cleanup(1);
      }
    }
    else
      if (!ext.compare(".bmp")) data = readBMP(filename.c_str(), width, height);
      else data = NULL;
  } else data = NULL;
  if (data == NULL) {
    width = 0;
    height = 0;
    string error("Unable to load texture map '");
    error.append(filename);
    error.append("'.");
    throw TextureMapException(error);
  }
}

Vec3d TextureMap::getMappedValue( const Vec2d& coord ) const
{
  // YOUR CODE HERE

  // In order to add texture mapping support to the
  // raytracer, you need to implement this function.
  // What this function should do is convert from
  // parametric space which is the unit square
  // [0, 1] x [0, 1] in 2-space to bitmap coordinates,
  // and use these to perform bilinear interpolation
  // of the values.



  return Vec3d(1.0, 1.0, 1.0);

}


Vec3d TextureMap::getPixelAt( int x, int y ) const
{
  // This keeps it from crashing if it can't load
  // the texture, but the person tries to render anyway.
  if (0 == data)
    return Vec3d(1.0, 1.0, 1.0);

  if( x >= width )
    x = width - 1;
  if( y >= height )
    y = height - 1;

  // Find the position in the big data array...
  int pos = (y * width + x) * 3;
  return Vec3d( double(data[pos]) / 255.0,
      double(data[pos+1]) / 255.0,
      double(data[pos+2]) / 255.0 );
}

Vec3d MaterialParameter::value( const isect& is ) const
{
  if( 0 != _textureMap )
    return _textureMap->getMappedValue( is.uvCoordinates );
  else
    return _value;
}

double MaterialParameter::intensityValue( const isect& is ) const
{
  if( 0 != _textureMap )
  {
    Vec3d value( _textureMap->getMappedValue( is.uvCoordinates ) );
    return (0.299 * value[0]) + (0.587 * value[1]) + (0.114 * value[2]);
  }
  else
    return (0.299 * _value[0]) + (0.587 * _value[1]) + (0.114 * _value[2]);
}

