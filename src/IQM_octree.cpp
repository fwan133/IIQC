#include "IQM_octomap/IQM_octree.h"

namespace octomap {
    IQMOcTree::IQMOcTree(double resolution) : OccupancyOcTreeBase<IQMOcTreeNode> (resolution){
        IQMOcTreeMemberInit.ensureLinking();
    }
}