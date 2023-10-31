#ifndef IIQC_IQM_OCTREE_NODE_H
#define IIQC_IQM_OCTREE_NODE_H

#include <map>
#include <string>
#include <iostream>

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include "IQM_octomap/IQM_octree.h"

namespace octomap {
    class IQMOctreeNode : public OcTreeNode {
    public:
        friend class IQMOctree;

        class IQM {
        public:
            IQM() : is_updated(false), blur_probability_value(1.0), exposure_intensity_value(255), ssd_map() {};

            bool operator==(const IQM& IQM_) const {
                return (IQM_.is_updated==is_updated && IQM_.blur_probability_value == blur_probability_value && IQM_.exposure_intensity_value==exposure_intensity_value && IQM_.ssd_map == ssd_map);
            }

            bool is_updated;
            float blur_probability_value;
            uint8_t exposure_intensity_value;
            std::map<std::string, float> ssd_map;
        };

    public:
        ///Default constructor
        IQMOctreeNode(): OcTreeNode(){};

        IQMOctreeNode(const IQMOctreeNode& rhs) : OcTreeNode(rhs), semantic_label(rhs.semantic_label), IQM_value(rhs.IQM_value) {};

        bool operator==(const IQMOctreeNode& rhs) const {
            return (rhs.value == value && rhs.semantic_label == semantic_label && rhs.IQM_value==IQM_value);
        }

        void copyData(const IQMOctreeNode& from){
            OcTreeNode::copyData(from);
            this->semantic_label=from.getSemanticLabel();
            this->IQM_value=from.getIQM();
        }

        /// Set and Get Semantic Label
        inline void setSemanticLabel(std::string semantic_label_) { semantic_label = semantic_label_; };

        inline std::string getSemanticLabel() const { return semantic_label; };

        /// Set and Get IQM
        void setIQM(float blur_probability, uint8_t exposure_intensity, std::string image_id, float ssd) {
            IQM_value.blur_probability_value = blur_probability;
            IQM_value.exposure_intensity_value = exposure_intensity;
            IQM_value.ssd_map.insert(std::make_pair(image_id, ssd));
            IQM_value.is_updated = true;
        };

        inline IQM getIQM() const { return IQM_value; };

        /// Update the IQM result
        void updateIQMResult(){

        }

        /// Print IQC results
        void printNodeIQMResult() {
            std::cout << "The node at " << std::endl;
        }

    protected:
        std::string semantic_label;
        IQM IQM_value;
    };
}

#endif //IIQC_IQM_OCTREE_NODE_H
