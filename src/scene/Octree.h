/*
 * Octree.h
 *
 *  Created on: Nov 28, 2012
 *      Author: taylormckinney
 */

#ifndef OCTREE_H_
#define OCTREE_H_

//#include "./scene.h"
class Geometry;

#define MAX_OBJECTS 3
#define MAX_DEPTH   20


class OctreeNode {
    Scene* scene;
    OctreeNode* parent;
    BoundingBox bb;
    OctreeNode* children[8];
    std::vector<Geometry*> objects;
    int depth;
    bool leaf;
    std::vector<Geometry*>::iterator begin() { return objects.begin(); }
    std::vector<Geometry*>::iterator end() { return objects.end(); }

  public:
    OctreeNode(OctreeNode* _parent, BoundingBox _bb, int _depth, std::vector<Geometry*>* _objs);
    ~OctreeNode();

    void subdivide(std::vector<Geometry*>* _obs);
    bool isLeaf() { return leaf; }
};


class Octree {
  private:
    OctreeNode* root;

  public:
    Octree() {}
    ~Octree() {
      if (root != 0) {
        delete root;
      }
    }

    void build(std::vector<Geometry*>* objs, BoundingBox bb) {
      root = new OctreeNode(0, bb, 0, objs);
    }
};

#endif /* OCTREE_H_ */
