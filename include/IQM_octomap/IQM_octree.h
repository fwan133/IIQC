#ifndef IIQC_IQM_OCTREE_H
#define IIQC_IQM_OCTREE_H

#include <iostream>
#include <cstdlib>
#include <string>

#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Vector3.h>

#include "IQM_image/IQM_image.h"

namespace octomap {
    // forward declaraton for "friend"
    class IQMOcTree;

    enum ColorType {
        DEFAULT, SEMANTIC, TRAFFIC, MULTIPLE
    };

    // node definition
    class IQMOcTreeNode : public octomap::ColorOcTreeNode {
    public:
        friend class IQMOcTree;  // needs access to node children (inherited)

        class IQM {
        public:
            enum IQMResultType {
                TRAFFIC, MULTIPLE
            };

        public:
            IQM() : is_updated(false), blur_probability_value(1.0), exposure_intensity_value(255), ssd_map() {};

            bool operator==(const IQM &IQM_) const {
                return (IQM_.is_updated == is_updated && IQM_.blur_probability_value == blur_probability_value &&
                        IQM_.exposure_intensity_value == exposure_intensity_value && IQM_.ssd_map == ssd_map);
            }

            void assessIQMLevel(float blur_probability_threshold, uint8_t exposure_threshold_min,
                                uint8_t exposure_threshold_max, uint captured_times_threshold, float SSD_threshold,
                                IQMResultType IQM_result_type = IQMResultType::MULTIPLE) {
                if (!is_updated) {    // IQM Not updated
                    IQM_result_level = 0;
                    return;
                }

                std::unordered_map<int, float> qualified_result;
                std::unordered_map<int, float> unqualified_result;
                for (const auto &pair: ssd_map) {
                    if (pair.second > SSD_threshold) {
                        unqualified_result[pair.first] = pair.second;
                    } else {
                        qualified_result[pair.first] = pair.second;
                    }
                }

                switch (IQM_result_type) {
                    case 0:
                        if (blur_probability_value >= blur_probability_threshold ||
                            exposure_intensity_value <= exposure_threshold_min ||
                            exposure_intensity_value >= exposure_threshold_max) {
                            IQM_result_level = 1;
                        } else if (qualified_result.size() < captured_times_threshold) {
                            IQM_result_level = 2;
                        } else {
                            IQM_result_level = 3;
                        }
                        break;
                    case 1: // Multiple
                        if (blur_probability_value >= blur_probability_threshold) {
                            IQM_result_level = 1;
                        } else if (exposure_intensity_value <= exposure_threshold_min) {
                            IQM_result_level = 2;
                        } else if (exposure_intensity_value >= exposure_threshold_max) {
                            IQM_result_level = 3;
                        } else if (qualified_result.size() == 0) {
                            if (unqualified_result.size() == 1) {
                                IQM_result_level = 4;
                            } else if (unqualified_result.size() == 2) {
                                IQM_result_level = 5;
                            } else if (unqualified_result.size() >= 3) {
                                IQM_result_level = 6;
                            }
                        } else {
                            if (qualified_result.size() == 1) {
                                IQM_result_level = 7;
                            } else if (qualified_result.size() == 2) {
                                IQM_result_level = 8;
                            } else if (qualified_result.size() >= 3) {
                                IQM_result_level = 9;
                            }
                        }
                        break;
                }
            }

            bool is_updated;
            float blur_probability_value;
            uint8_t exposure_intensity_value;
            std::unordered_map<int, float> ssd_map;
            uint8_t IQM_result_level = 0;
        };

    public:
        ///Default constructor
        IQMOcTreeNode() : ColorOcTreeNode() {};

        IQMOcTreeNode(const IQMOcTreeNode &rhs) : ColorOcTreeNode(rhs), semantic_label(rhs.semantic_label),
                                                  IQM_value(rhs.IQM_value) {};

        bool operator==(const IQMOcTreeNode &rhs) const {
            return (rhs.value == value && rhs.color == color && rhs.semantic_label == semantic_label &&
                    rhs.IQM_value == IQM_value);
        }

        void copyData(const IQMOcTreeNode &from) {
            ColorOcTreeNode::copyData(from);
            this->semantic_label = from.getSemanticLabel();
            this->IQM_value = from.getIQM();
        }

        /// Set and Get Semantic Label
        inline void setSemanticLabel(std::string semantic_label_) { semantic_label = semantic_label_; };

        inline std::string getSemanticLabel() const { return semantic_label; };

        /// Update IQM attributes
        /// Update the blur probability of node
        inline void updateNodeBlurProbability(float blur_value) {
            if (IQM_value.blur_probability_value > blur_value) { IQM_value.blur_probability_value = blur_value; };
        }

        /// Update the exposure intensity of node
        void updateNodeExposureIntensity(uint8_t exposure_value) {
            int diffA = std::abs((int) IQM_value.exposure_intensity_value - 128);
            int diffB = std::abs((int) exposure_value - 128);

            if (diffA > diffB) {
                IQM_value.exposure_intensity_value = exposure_value;
            };
        }

        /// Update the ssd of node
        void updateNodeSSD(int image_id, float ssd) {
            auto it = IQM_value.ssd_map.find(image_id);
            if (it != IQM_value.ssd_map.end()) { // Found the existing image_id
                if (ssd < it->second) it->second = ssd;
            } else {
//                std::cout << "[Add observation] \n";
                IQM_value.ssd_map[image_id]=ssd;
            }
        }

        /// Update IQM of node
        void updateNodeIQM(float blur_probability, uint8_t exposure_intensity, int image_id, float ssd) {
            if (!IQM_value.is_updated) {          // Update for the first time
                IQM_value.blur_probability_value = blur_probability;
                IQM_value.exposure_intensity_value = exposure_intensity;
                IQM_value.ssd_map[image_id]= ssd;
                IQM_value.is_updated = true;
            } else {                                // Update
                updateNodeBlurProbability(blur_probability);
                updateNodeExposureIntensity(exposure_intensity);
                updateNodeSSD(image_id, ssd);
                IQM_value.is_updated = true;
            }
        };

        inline IQM getIQM() const { return IQM_value; };

        const inline std::string getUpdateStatus() const {
            if (IQM_value.is_updated) { return "updated"; } else { return "not updated"; };
        }

        const float getMinSSD() const {
            if (!IQM_value.ssd_map.empty()) {
                float minValue = IQM_value.ssd_map.begin()->second; // Start with the maximum possible value

                for (const auto &pair: IQM_value.ssd_map) {
                    if (pair.second < minValue) {
                        minValue = pair.second;
                    }
                }
                return minValue;
            }
        };

        std::list<float> getSSD() const {
            std::list<float> SSDs;
            for (auto id_ssd: IQM_value.ssd_map) {
                SSDs.push_back(id_ssd.second);
            }
            return SSDs;
        }

        /// Update IQM colour of a node
        void updateIQMColour(float blur_probability_threshold, uint8_t exposure_threshold_min,
                             uint8_t exposure_threshold_max, uint captured_times_threshold, float SSD_threshold,
                             IQM::IQMResultType IQM_result_type) {
            // Calculate the IQM level
            IQM_value.assessIQMLevel(blur_probability_threshold, exposure_threshold_min, exposure_threshold_max,
                                     captured_times_threshold, SSD_threshold, IQM_result_type);

            if (IQM_value.IQM_result_level == 0) {
                this->setColor(191, 191, 191);                 // Level 0
                return;
            }

            switch (IQM_result_type) {
                case IQM::IQMResultType::TRAFFIC:
                    switch (IQM_value.IQM_result_level) {
                        case 1:
                            this->setColor(192, 0, 0);
                            break;
                        case 2:
                            this->setColor(255, 204, 0);
                            break;
                        case 3:
                            this->setColor(102, 158, 64);
                            break;
                    }
                    break;
                case IQM::IQMResultType::MULTIPLE:
                    switch (IQM_value.IQM_result_level) {
                        case 1:
                            this->setColor(255, 137, 109);
                            break;
                        case 2:
                            this->setColor(255, 185, 185);
                            break;
                        case 3:
                            this->setColor(192, 0, 0);
                            break;
                        case 4:
                            this->setColor(255, 238, 183);
                            break;
                        case 5:
                            this->setColor(255, 214, 83);
                            break;
                        case 6:
                            this->setColor(242, 184, 0);
                            break;
                        case 7:
                            this->setColor(187, 218, 166);
                            break;
                        case 8:
                            this->setColor(112, 173, 71);
                            break;
                        case 9:
                            this->setColor(102, 158, 64);
                            break;
                    }
                    break;
            }
        }

        /// File I/O
        std::ostream &writeData(std::ostream &s) const {
            s.write((const char *) &value, sizeof(value)); // occupancy
            s.write((const char *) &color, sizeof(Color)); // color
            return s;
        };

    protected:
        std::string semantic_label;
        IQM IQM_value;
    };

    // IQM OcTree Definition
    class IQMOcTree : public octomap::OccupancyOcTreeBase<IQMOcTreeNode> {
    public:
        /// Default constructor, set resolution of leafs
        IQMOcTree(double resolution);

        IQMOcTree *create() const { return new IQMOcTree(resolution); }

        std::string getTreeType() const { return "ColorOcTree"; }

        /// Set MetricsThreshold
        inline void setMetricsThreshold(float blur_probability_threshold, uint8_t exposure_threshold_min,
                                        uint8_t exposure_threshold_max, uint captured_times_threshold,
                                        float SSD_threshold) {
            metrics_threshold.blur_probability_threshold = blur_probability_threshold;
            metrics_threshold.exposure_threshold_min = exposure_threshold_min;
            metrics_threshold.exposure_threshold_max = exposure_threshold_max;
            metrics_threshold.captured_times_threshold = captured_times_threshold;
            metrics_threshold.SSD_threshold = SSD_threshold;
        }

        /// Update the node colour
        void setNodeColor(const OcTreeKey &key, uint8_t r, uint8_t g, uint8_t b) {
            IQMOcTreeNode *n = this->search(key);
            if (n != 0) {
                n->setColor(r, g, b);
            }
        };

        void setNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
            OcTreeKey key;
            if (this->coordToKeyChecked(point3d(x, y, z), key)) {
                this->setNodeColor(key, r, g, b);
            }
        };

        /// Update the node semantic label
        void setNodeSemantic(float x, float y, float z, std::string semantic_label) {
            OcTreeKey key;
            if (this->coordToKeyChecked(point3d(x, y, z), key)) {
                IQMOcTreeNode *n = this->search(key);
                if (n != 0) {
                    n->setSemanticLabel(semantic_label);
                }
            }
        }

        /// Initial the IQM node
        IQMOcTreeNode *
        initialiseIQMNode(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b, std::string semantic_label) {
            IQMOcTreeNode *n = this->updateNode(x, y, z, true);
            n->setColor(r, g, b);
            n->setSemanticLabel(semantic_label);
            return n;
        }

        /// Update the IQM value of a node
        bool updateNodeIQM(float x, float y, float z, float blur_probability, uint8_t exposure_intensity,
                           int image_id, float ssd) {
            OcTreeKey key;
            if (this->coordToKeyChecked(point3d(x, y, z), key)) {
                IQMOcTreeNode *node;
                node = this->search(key);
                node->updateNodeIQM(blur_probability, exposure_intensity, image_id, ssd);
                return true;
            } else {
                return false;
            }
        }

        /// Update from an IQM_Image object
        bool updateIQMfromImage(IQMImage IQM_image, double maximum_depth) {
            if (IQM_image.checkValidity()) {
                // Obtain reference
                cv::Mat *color_img = const_cast<cv::Mat *>(IQM_image.getColorImg());
                cv::Mat *blur_map = const_cast<cv::Mat *>(IQM_image.getBPM());
                cv::Mat *exposure_map = const_cast<cv::Mat *>(IQM_image.getEIM());

                // Go through each pixel
                for (int u = 0; u < IQM_image.imgSize().width; u += 5) {
                    for (int v = 0; v < IQM_image.imgSize().height; v += 5) {
//                        std::cout << "\n[Pixel Coordinate]: The pixel location is (" << u << ", " << v << ")\n";
                        // Obtain the start point and direction
                        Eigen::Vector3d point = IQM_image.backProjection(u, v);
                        Eigen::Vector3d direction_ = point - IQM_image.getPose().position;

                        // RayCasting
                        octomap::point3d origin(IQM_image.getPose().position.x(), IQM_image.getPose().position.y(),
                                                IQM_image.getPose().position.z());
                        octomap::point3d direction(direction_.x(), direction_.y(), direction_.z());
                        octomap::point3d end;
                        if (this->castRay(origin, direction, end, true, maximum_depth)) {
                            this->updateNodeIQM(end.x(), end.y(), end.z(), blur_map->at<float>(v, u),
                                                exposure_map->at<int8_t>(v, u), IQM_image.getImageID(),
                                                IQM_image.estimateSSD(end.x(), end.y(), end.z()));
                        }
                    }
                }
                return true;
            } else {
                std::cout << "The image is not valid!" << std::endl;
                return false;
            }
        }

        /// Update IQM colour of a node
        bool
        updateNodeIQMColor(float x, float y, float z,
                           IQMOcTreeNode::IQM::IQMResultType IQM_result_type = IQMOcTreeNode::IQM::IQMResultType::MULTIPLE) {
            OcTreeKey key;
            if (this->coordToKeyChecked(point3d(x, y, z), key)) {
                IQMOcTreeNode *node;
                node = this->search(key);
                node->updateIQMColour(metrics_threshold.blur_probability_threshold,
                                      metrics_threshold.exposure_threshold_min,
                                      metrics_threshold.exposure_threshold_max,
                                      metrics_threshold.captured_times_threshold, metrics_threshold.SSD_threshold,
                                      IQM_result_type);
                return true;
            } else {
                return false;
            }
        };

        /// Refresh the OcTree Color
        bool refreshTreeColor(octomap::ColorType color_type) {
            std::map<std::string, ColorOcTreeNode::Color> semantic_color_map;

            for (IQMOcTree::leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it) {
                if (this->isNodeOccupied(*it)) {
                    switch (color_type) {
                        case ColorType::DEFAULT:
                            it->setColor(191, 191, 191);
                            break;
                        case ColorType::SEMANTIC:
                            if (semantic_color_map.find(it->getSemanticLabel()) == semantic_color_map.end()) {
                                ColorOcTreeNode::Color color(std::rand() % 256, std::rand() % 256, std::rand() % 256);
                                it->setColor(color);
                                semantic_color_map.insert(
                                        std::pair<std::string, ColorOcTreeNode::Color>(it->getSemanticLabel(), color));
                            } else {
                                it->setColor(semantic_color_map.find(it->getSemanticLabel())->second);
                            }
                            break;
                        case ColorType::TRAFFIC:
                            it->updateIQMColour(metrics_threshold.blur_probability_threshold,
                                                metrics_threshold.exposure_threshold_min,
                                                metrics_threshold.exposure_threshold_max,
                                                metrics_threshold.captured_times_threshold,
                                                metrics_threshold.SSD_threshold,
                                                IQMOcTreeNode::IQM::IQMResultType::TRAFFIC);
                            break;
                        case ColorType::MULTIPLE:
                            it->updateIQMColour(metrics_threshold.blur_probability_threshold,
                                                metrics_threshold.exposure_threshold_min,
                                                metrics_threshold.exposure_threshold_max,
                                                metrics_threshold.captured_times_threshold,
                                                metrics_threshold.SSD_threshold,
                                                IQMOcTreeNode::IQM::IQMResultType::MULTIPLE);
                            break;
                    }

                    /// Print the node info
//                    if (it->getIQM().is_updated && it->getIQM().ssd_map.size()>=2) printNodeInfo(it);
                }
            }
            return true;
        }

        /// Print node info
        void printNodeInfo(IQMOcTree::leaf_iterator it) {
            std::string occupancy_status;
            if (this == nullptr) {
                occupancy_status = "Unknown";
                std::cout << "Centre location: " << ". Occupancy status: "
                          << occupancy_status << std::endl;
            } else {
                std::cout << "[Node Info] Location: " << keyToCoord(it.getKey()) << ". Colour: " << it->getColor()
                          << ". Semantic Label: " << it->getSemanticLabel() << "." << std::endl;

                if (it->getIQM().is_updated) {
                    std::cout << "BP: "
                              << it->getIQM().blur_probability_value << ". EI: "
                              << (int) it->getIQM().exposure_intensity_value << ". Captured times: "
                              << it->getIQM().ssd_map.size() << ". SSD value: ";
                    for (auto ssd: it->getSSD()) {
                        std::cout << ssd << " ";
                    }
                    std::cout << ". IQM Color Level: "
                              << (int) it->IQM_value.IQM_result_level << "." << std::endl;
                }
            }
        }

        void printNodeInfo(float x, float y, float z) {
            std::cout << "The info of IQMOcTree node at (" << x << ", " << y << ", " << z << ") is as follows:"
                      << std::endl;
            OcTreeKey key;
            IQMOcTreeNode *node;
            std::string occupancy_status;

            if (this->coordToKeyChecked(point3d(x, y, z), key)) {
                node = this->search(key);
                if (node == nullptr) {
                    occupancy_status = "Unknown";
                    std::cout << "Centre location: " << this->keyToCoord(key) << ". Occupancy status: "
                              << occupancy_status << std::endl;
                } else {
                    occupancy_status = isNodeOccupied(node) ? "Occupied." : "Free";
                    std::cout << "Centre location: " << this->keyToCoord(key) << ". Occupancy status: "
                              << occupancy_status << "Colour: " << node->getColor() << std::endl;
                    std::cout << "Semantic Label: " << node->getSemanticLabel() << "." << std::endl;

                    if (node->getIQM().is_updated) {
                        std::cout << "Image Quality Metric: " << node->getUpdateStatus() << ". Blur Probability: "
                                  << node->getIQM().blur_probability_value << ". Exposure Intensity: "
                                  << (int) node->getIQM().exposure_intensity_value << ". Coverage Times:"
                                  << node->getIQM().ssd_map.size() << ". Minimum SSD: " << node->getMinSSD()
                                  << ". IQM Color Level: " << (int) node->IQM_value.IQM_result_level
                                  << std::endl;
                    } else {
                        std::cout << "Image Quality Metric: " << node->getUpdateStatus() << "." << std::endl;
                    }
                }
            }
        }

        /// Write Data


    protected:
        /**
         * Static member object which ensures that this OcTree's prototype
         * ends up in the classIDMapping only once. You need this as a
         * static member in any derived octree class in order to read .ot
         * files through the AbstractOcTree factory. You should also call
         * ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer {
        public:
            StaticMemberInitializer() {
                IQMOcTree *tree = new IQMOcTree(0.1);
                tree->clearKeyRays();
                AbstractOcTree::registerTreeType(tree);
            }

            /**
             * Dummy function to ensure that MSVC does not drop the
             * StaticMemberInitializer, causing this tree failing to register.
             * Needs to be called from the constructor of this octree.
             */
            void ensureLinking() {}
        };

        class MetricsThreshold {
        public:
            float blur_probability_threshold = 0.8;
            uint8_t exposure_threshold_min = 20;
            uint8_t exposure_threshold_max = 235;
            uint captured_times_threshold = 2;
            float SSD_threshold = 0.0015;
        };

        /// static member to ensure static initialization (only once)
        static StaticMemberInitializer IQMOcTreeMemberInit;
        MetricsThreshold metrics_threshold;
    };
}

#endif //IIQC_IQM_OCTREE_H
