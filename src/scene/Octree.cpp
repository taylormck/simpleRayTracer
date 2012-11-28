/*
 * Octree.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: taylormckinney
 */

#include "Octree.h"

OctreeNode::OctreeNode(OctreeNode* _parent, BoundingBox _bb, int _depth, std::vector<Geometry*>* _objs)
: parent(_parent), bb(_bb), depth(_depth) {

  for(int i = 0; i < 8; ++i) {
    children[i] = 0;
  }

  objects = new std::vector<Geometry*>();

  int i_limit = _objs->size();
  for (int i = 0; i < i_limit; ++i) {
    Geometry* g = _objs->at(i);
    if (bb.intersects(g->getBoundingBox())) {
      objects.push_back(g);
    }
  }

  if (objects.size() > MAX_OBJECTS && depth <= MAX_DEPTH) {
    subdivide(&objects);
  } else {
    leaf = true;
  }
}

OctreeNode::~OctreeNode() {
  for (int i = 0; i < 8; ++i) {
    if (children[i] != 0) {
      delete children[i];
    }
  }
}

void OctreeNode::subdivide(std::vector<Geometry*>* _objs) {
  Vec3d size = bb.getMax() - bb.getMin();
  Vec3d start = bb.getMin();
  Vec3d step = size / 2;
  int counter = 0;

  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        Vec3d newMin = start + prod(step, Vec3d(x, y, z));
        Vec3d newMax = start + prod(step, Vec3d(x + 1, y + 1, z + 1));
        BoundingBox childBB(newMin, newMax);
        children[counter] = new OctreeNode(this, childBB, depth + 1, &objects);
        counter++;
      }
    }
  }
}

bool OctreeNode::intersect(ray& r) {
  double t0, t1;
  return bb.intersect(r, t0, t1);
}

void OctreeNode::intersectObjects(ray& r, std::vector<Geometry*>* obj_list) {
  if (intersect(r)) {
    if (leaf) {
      bool copy = false;
      int i_limit = objects.size();
      for(int i = 0; i < i_limit; ++i) {
        int j_limit = obj_list->size();
        for(int j = 0; j < j_limit && !copy; ++j){
          if ( objects[i] == obj_list[j])
            copy = true;
        }
        if (!copy)
          obj_list->push_back(objects[i]);
      }
    } else {
      // Not a leaf, so we recurse
      for (int i = 0; i < 8; ++i) {
        if (children[i] != 0)
          children[i]->intersectObjects(r, obj_list);
      }
    }
  }
}



