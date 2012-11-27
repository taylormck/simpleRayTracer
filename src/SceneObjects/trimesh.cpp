#include <cmath>
#include <float.h>
#include "trimesh.h"

extern bool debugMode;

using namespace std;

Trimesh::~Trimesh()
{
  for( Materials::iterator i = materials.begin(); i != materials.end(); ++i )
    delete *i;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex( const Vec3d &v )
{
  vertices.push_back( v );
}

void Trimesh::addMaterial( Material *m )
{
  materials.push_back( m );
}

void Trimesh::addNormal( const Vec3d &n )
{
  normals.push_back( n );
}

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace( int a, int b, int c )
{
  int vcnt = vertices.size();

  if( a >= vcnt || b >= vcnt || c >= vcnt ) return false;

  TrimeshFace *newFace = new TrimeshFace( scene, new Material(*this->material), this, a, b, c );
  newFace->setTransform(this->transform);
  faces.push_back( newFace );
  return true;
}

char *
Trimesh::doubleCheck()
// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
{
  if( !materials.empty() && materials.size() != vertices.size() )
    return "Bad Trimesh: Wrong number of materials.";
  if( !normals.empty() && normals.size() != vertices.size() )
    return "Bad Trimesh: Wrong number of normals.";

  return 0;
}

bool Trimesh::intersectLocal(const ray&r, isect&i) const
{
  double tmin = 0.0;
  double tmax = 0.0;
  typedef Faces::const_iterator iter;
  bool have_one = false;
  for( iter j = faces.begin(); j != faces.end(); ++j ) {
    isect cur;
    if( (*j)->intersectLocal( r, cur ) )
    {
      if( !have_one || (cur.t < i.t) )
      {
        i = cur;
        have_one = true;
      }
    }
  }
  if( !have_one ) i.setT(1000.0);
  return have_one;
}

// Intersect ray r with the triangle abc.  If it hits returns true,
// and puts the t parameter, barycentric coordinates, normal, object id,
// and object material in the isect object
bool TrimeshFace::intersectLocal( const ray& r, isect& i ) const
{
  const Vec3d& a = parent->vertices[ids[0]];
  const Vec3d& b = parent->vertices[ids[1]];
  const Vec3d& c = parent->vertices[ids[2]];

  Vec3d p0 = r.at(0);
  Vec3d d = r.getDirection();

  // Try to rule out most of these things quickly
  // While not really a spacial data structure, these rule outs
  // cause a significant speed up
  Vec3d min = localbounds.getMin();
  Vec3d max = localbounds.getMax();
  for (int j = 2; j >= 0; --j) {
    if ((p0[j] < min[j] && d[j] <= 0) || (p0[j] > max[j] && d[j] >= 0)) {
      return false;
    }
  }

  // The normal was derived during the constructor

  // Find t
  double D = -1.0 * normal * a;
  double t = -1.0 * (normal * p0 + D) / (normal * d);

  if ((t < RAY_EPSILON) || (i.t != 0 && t > i.t))
    return false;

  // Find p
  Vec3d p = r.at(t);

  // Find barycentric coordinates
  // Formula found online at
  // http://gamedev.stackexchange.com/questions/23743/
  // whats-the-most-efficient-way-to-find-barycentric-coordinates
  Vec3d v0 = b - a;
  Vec3d v1 = c - a;
  Vec3d v2 = p - a;
  double lambda_0, lambda_1, lambda_2;
  double d00 = v0 * v0;
  double d01 = v0 * v1;
  double d11 = v1 * v1;
  double d20 = v2 * v0;
  double d21 = v2 * v1;
  double denom = d00 * d11 - d01 * d01;
  lambda_0 = (d11 * d20 - d01 * d21) / denom;
  if (lambda_0 < 0)
    return false;
  lambda_1 = (d00 * d21 - d01 * d20) / denom;
  if (lambda_1 < 0)
    return false;
  lambda_2 = 1.0 - lambda_0 - lambda_1;
  if (lambda_2 < 0)
    return false;

  // If we reach this point, we have an intersection
  i.setBary(lambda_0, lambda_1, lambda_2);
  i.setT(t);
  i.setObject(this);

  if (debugMode) cout << "vertNorms: " << parent->vertNorms << endl;

  // Set the normal
  Vec3d n = normal;
  if (parent->normals.size() > 0) {
    Vec3d n0 = parent->normals[ids[0]] * lambda_0;
    Vec3d n1 = parent->normals[ids[1]] * lambda_1;
    Vec3d n2 = parent->normals[ids[2]] * lambda_2;
    n = n0 + n1 + n2;
  }
  i.setN(n);

  if (parent->materials.size() > 0) {  // Per vertex material
    if (debugMode) cout << "Using interpolated material." << endl;
    Material m;
    Material m0 = *parent->materials[ids[0]];
    Material m1 = *parent->materials[ids[1]];
    Material m2 = *parent->materials[ids[2]];
    m = lambda_0 * m0;
    m += lambda_1 * m1;
    m += lambda_2 * m2;
    i.setMaterial(m);
  } else if (material != NULL) {  // Per face material
    if (debugMode) cout << "Using face's material" << endl;
    i.setMaterial(*material);
  } else {  // Scene material ?
    if (debugMode) cout << "Using scene material" << endl;
    // If we get here, the material had better be set in the scene
//    i.setMaterial();
  }

  return true;
}

void Trimesh::generateNormals()
// Once you've loaded all the verts and faces, we can generate per
// vertex normals by averaging the normals of the neighboring faces.
{
  int cnt = vertices.size();
  normals.resize( cnt );
  int *numFaces = new int[ cnt ]; // the number of faces assoc. with each vertex
  memset( numFaces, 0, sizeof(int)*cnt );

  for( Faces::iterator fi = faces.begin(); fi != faces.end(); ++fi )
  {
    Vec3d faceNormal = (**fi).getNormal();

    for( int i = 0; i < 3; ++i )
    {
      normals[(**fi)[i]] += faceNormal;
      ++numFaces[(**fi)[i]];
    }
  }

  for( int i = 0; i < cnt; ++i )
  {
    if( numFaces[i] )
      normals[i]  /= numFaces[i];
  }

  delete [] numFaces;
  vertNorms = true;
}

