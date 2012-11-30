/*
 * Octree.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: taylormckinney
 */

#include "octree.h"
#include "scene.h"

OctreeNode::OctreeNode() {}  // Blank constructor

OctreeNode::OctreeNode(std::vector<Geometry*>* _tree_objects, BoundingBox _bb, int _depth, std::vector<int> _objs)
: tree_objects(_tree_objects), node_box(_bb), depth(_depth) {

  // Set the children to null
  for(int i = 0; i < 8; ++i) {
    children[i] = NULL;
  }

  // Check to see which of the parent's objects we contain
  int i_limit = _objs.size();
  for (int i = 0; i < i_limit; ++i) {
    if (contains(_objs[i])) {
      objects.push_back(_objs[i]);
    }
  }

  // Check to see if we've reached our end criteria
  // otherwise, subdivide into smaller spaces and recurse
  if (objects.size() > MAX_OBJECTS && depth < MAX_DEPTH) {
    subdivide(objects);
  } else {
    leaf = true;
  }
}

bool OctreeNode::contains(int g) {
  return true;
}

OctreeNode::~OctreeNode() {
  if (!leaf) {
    for (int i = 0; i < 8; ++i) {
      delete children[i];
    }
  }
}

void OctreeNode::subdivide(std::vector<int> _objs) {
  Vec3d size = node_box.getMax() - node_box.getMin();
  Vec3d start = node_box.getMin();
  Vec3d step = size / 2;
  int counter = 0;

  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        Vec3d newMin = start + prod(step, Vec3d(x, y, z));
        Vec3d newMax = start + prod(step, Vec3d(x + 1, y + 1, z + 1));
        BoundingBox childBB(newMin, newMax);
        children[counter] = new OctreeNode(tree_objects, childBB, depth + 1, objects);
        counter++;
      }
    }
  }
}

bool OctreeNode::intersect(const ray& r) {
  double t0, t1;
  return node_box.intersect(r, t0, t1);
}

void OctreeNode::intersectObjects(const ray& r, std::vector<int> &obj_list) {
  if (intersect(r)) {
    if (leaf) {
      bool copy = false;
      int i_limit = objects.size();
      for(int i = 0; i < i_limit; ++i) {

        // Check to see if we already have this object included
        int j_limit = obj_list.size();
        for(int j = 0; j < j_limit && !copy; ++j){
          if ( objects[i] == obj_list[j])
            copy = true;
        }
        if (!copy)
          obj_list.push_back(objects[i]);
      }
    } else {
      // Not a leaf, so we recurse
      for (int i = 0; i < 8; ++i) {
        if (children[i] != NULL)
          children[i]->intersectObjects(r, obj_list);
      }
    }
  }
}

//---------------------------------------------------------------------------//
//                        Octree code                                        //
//---------------------------------------------------------------------------//
Octree::Octree()
: objects(), tree_box(Vec3d(0, 0, 0), Vec3d(0, 0, 0)) {}

Octree::~Octree() {}

std::vector<int> Octree::reducedObjectSet(const ray& r) {
  std::vector<int> result;
  root.intersectObjects(r, result);
  return result;
}

void Octree::build(std::vector<Geometry*> &objs, BoundingBox _bb) {
  int i_limit = objs.size();
  for (int i = 0; i < i_limit; ++i) objects.push_back(objs[i]);
  tree_box = BoundingBox(_bb);
  std::vector<int> root_objects;
  i_limit = objs.size();
  for (int i = 0; i < i_limit; ++i) root_objects.push_back(i);

  root = OctreeNode(&objects, tree_box, 0, root_objects);
}

std::vector<Geometry*> Octree::getObjects() { return objects; }


