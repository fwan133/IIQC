#include <iostream>
#include "IQM_octomap/IQM_octree.h"

int main (int argc, char** argv){
    // Test the IQMOctree Initialisation from a point cloud
    octomap::IQMOcTree tree(0.1);
    std::cout << tree.getTreeType() << std::endl;


    // Test the IQMOctreeNode and colour visualisation
    tree.initialiseIQMNode(0, 0, 0, 129, 128, 128, "Girder");
    tree.updateNodeIQM(0, 0, 0, 0.5, 5, "Image_1", 3);
    tree.updateNodeIQM(0, 0, 0, 0.4, 125, "Image_2", 2);
    tree.updateNodeIQM(0, 0, 0, 0.4, 125, "Image_1", 1.5);
    tree.updateNodeIQMColor(0,0,0);
    tree.printNodeInfo(0,0,0);

}