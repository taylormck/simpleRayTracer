/*
 * Octree.h
 *
 *  Created on: Nov 28, 2012
 *      Author: taylormckinney
 *
 *      The octree has been put on hold for now.
 *      This extra code shouldn't slow the renderer down at all when not using this,
 *      and I can't keep it from segfault or get it to play nice with
 *      Triangle Meshes
 */

#ifndef OCTREE_H_
#define OCTREE_H_

#define MAX_OBJECTS 3
#define MAX_DEPTH   10

#include "scene.h"

class OctreeNode {
    BoundingBox node_box;
    std::vector<Geometry*>* tree_objects;
    OctreeNode* children[8];
    std::vector<int> objects;
    int depth;
    bool leaf;

    bool intersect(const ray& r);
    bool contains(int g);

  public:
    OctreeNode();
    OctreeNode(std::vector<Geometry*>* _tree_objects, BoundingBox _bb, int _depth, std::vector<int> _objs);
    ~OctreeNode();

    void subdivide(std::vector<int> _obs);
    bool isLeaf() { return leaf; }
    void intersectObjects(const ray& r, std::vector<int> &obj_list);
};


class Octree {
  private:
    OctreeNode root;
    BoundingBox tree_box;
    std::vector<Geometry*> objects;

  public:
    Octree();
    ~Octree();

    void build(std::vector<Geometry*> &objs, BoundingBox bb);

    std::vector<int> reducedObjectSet(const ray& r);
    std::vector<Geometry*> getObjects();
};

#endif /* OCTREE_H_ */
