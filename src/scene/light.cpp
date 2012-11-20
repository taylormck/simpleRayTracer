#include <cmath>

#include "light.h"



using namespace std;

double DirectionalLight::distanceAttenuation( const Vec3d& P ) const
{
  // distance to light is infinite, so f(di) goes to 0.  Return 1.
  return 1.0;
}


Vec3d DirectionalLight::shadowAttenuation( const Vec3d& P ) const
{
  Vec3d shadow_att;
  Vec3d light_dir = -orientation;
  ray point_to_light (P, light_dir, ray::SHADOW);
  isect i;
  if (scene->intersect(point_to_light, i)) {
    shadow_att = Vec3d(0, 0, 0);
  } else {
    shadow_att = Vec3d(1.0, 1.0, 1.0);
  }

  return shadow_att;
}

Vec3d DirectionalLight::getColor( const Vec3d& P ) const
{
  // Color doesn't depend on P
  return color;
}

Vec3d DirectionalLight::getDirection( const Vec3d& P ) const
{
  return -orientation;
}

double PointLight::distanceAttenuation( const Vec3d& P ) const {
  double distance = (position - P).length();
  return min(1.0, max(0.0,  // Clamps the value to [0, 1]
      1.0 / (constantTerm
          + linearTerm * distance
          + quadraticTerm * distance * distance)));
}

Vec3d PointLight::getColor( const Vec3d& P ) const
{
  // Color doesn't depend on P
  return color;
}

Vec3d PointLight::getDirection( const Vec3d& P ) const
{
  Vec3d ret = position - P;
  ret.normalize();
  return ret;
}


Vec3d PointLight::shadowAttenuation(const Vec3d& P) const
{
  Vec3d shadow_att = Vec3d(1.0, 1.0, 1.0);
  Vec3d light_dir = position - P;
  ray point_to_light (P, light_dir, ray::SHADOW);
  isect i;
  if (scene->intersect(point_to_light, i)) {
    if (i.t < light_dir.length())
      shadow_att = Vec3d(0);
  }

  return shadow_att;
}
