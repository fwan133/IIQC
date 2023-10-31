#include <octomap/ColorOcTree.h>

#include "IQM_octomap/IQM_octree.h"

namespace octomap {

    class IQM_OctoMap {
    public:
        IQM_OctoMap(double voxel_size, double blur_threshold, double exposure_min, double exposure_max,
                    uint times_threshold, double ssd_threshold) {
            IQM_octree = new octomap::IQMOcTree(voxel_size);
            blur_probability_threshold = blur_threshold;
            exposure_threshold_max = exposure_max;
            exposure_threshold_min = exposure_min;
            captured_times_threshold = times_threshold;
            SSD_threshold = ssd_threshold;
        }

        ~IQM_OctoMap() {};

        const octomap::IQMOcTree* getIQMOctree() const { return IQM_octree; };

        void updateIQMOctree(float x, float y, float z) {
            IQM_octree->updateNode(octomap::point3d(x, y, z), true);
        }

        void updateIQMOctree(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
            IQM_octree->updateNode(octomap::point3d(x, y, z), true);
            IQM_octree->setNodeColor(x, y, z, r, g, b);
        }

        void updateIQMResult() {

        }

    protected:
        octomap::IQMOcTree *IQM_octree;
        double blur_probability_threshold;
        double exposure_threshold_min;
        double exposure_threshold_max;
        uint captured_times_threshold;
        double SSD_threshold;
    }

}